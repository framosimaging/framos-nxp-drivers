// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2024 Framos. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

// #define DEBUG 1
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>

#include "imx900_regs.h"
#include "max96792.h"
#include "max96793.h"

#define IMX900_MAX_RETRIES 10

#define IMX900_SENS_PAD_SOURCE	0
#define IMX900_SENS_PADS_NUM	1

#define IMX900_K_FACTOR 1000LL
#define IMX900_M_FACTOR 1000000LL
#define IMX900_G_FACTOR 1000000000LL
#define IMX900_T_FACTOR 1000000000000LL

#define IMX900_MAX_GAIN_DEC 480
#define IMX900_MAX_GAIN_DB  48

#define IMX900_MAX_BLACK_LEVEL			4095
#define IMX900_DEFAULT_BLACK_LEVEL_8BPP		15
#define IMX900_DEFAULT_BLACK_LEVEL_10BPP	60
#define IMX900_DEFAULT_BLACK_LEVEL_12BPP	240

#define IMX900_MIN_SHS_LENGTH 51
#define IMX900_INTEGRATION_OFFSET 2
#define IMX900_MIN_INTEGRATION_LINES 1

#define IMX900_MAX_CSI_LANES 2
#define IMX900_TWO_LANE_MODE 3
#define IMX900_ONE_LANE_MODE 4

#define IMX900_1ST_INCK 74250000LL
#define IMX900_2ND_INCK 72000000LL

#define IMX900_XCLK_MIN 37000000
#define IMX900_XCLK_MAX 37250000

#define IMX900_MAX_BOUNDS_WIDTH 2064
#define IMX900_MAX_BOUNDS_HEIGHT 1688
#define IMX900_LINE_TIME 8215 // hmax = 610

#define V4L2_CID_DATA_RATE		(V4L2_CID_USER_IMX_BASE + 1)
//#define V4L2_CID_SYNC_MODE		(V4L2_CID_USER_IMX_BASE + 2)
#define V4L2_CID_FRAME_RATE		(V4L2_CID_USER_IMX_BASE + 2)
#define V4L2_CID_SHUTTER_MODE	(V4L2_CID_USER_IMX_BASE + 3)

static const struct of_device_id imx900_of_match[] = {
	{ .compatible = "framos,imx900" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, imx900_of_match);

enum data_rate_mode {
	IMX900_2376_MBPS,
	IMX900_1485_MBPS,
	IMX900_1188_MBPS,
	IMX900_891_MBPS,
	IMX900_594_MBPS,
};

const char * const data_rate_menu[] = {
	[IMX900_2376_MBPS] = "2376 Mbps/lane",
	[IMX900_1485_MBPS] = "1485 Mbps/lane",
	[IMX900_1188_MBPS] = "1188 Mbps/lane",
	[IMX900_891_MBPS] = "891 Mbps/lane",
	[IMX900_594_MBPS] = "594 Mbps/lane",
};

static const char * const test_pattern_menu[] = {
	[0]   = "No pattern",
	[1]   = "Sequence Pattern 1",
	[2]   = "Sequence Pattern 2",
	[3]   = "Gradation Pattern",
	[4]   = "Color Bar Horizontally",
	[5]   = "Color Bar Vertically",
};
/*
 * Tranformation matrix from gain times used by isp to gain registers used
 * by Sony sensors by formula gain_time = 10**(gain_db / 20) * 1024
 * the resulting value is in range (0-480)
 */
#define IMX900_GAIN_REG_LEN 481
static const u32 gain_reg2times[IMX900_GAIN_REG_LEN] = {
	1024, 1035, 1047, 1059, 1072, 1084, 1097, 1109, 1122, 1135, 1148, 1162,
	1175, 1189, 1203, 1217, 1231, 1245, 1259, 1274, 1289, 1304, 1319, 1334,
	1349, 1365, 1381, 1397, 1413, 1429, 1446, 1463, 1480, 1497, 1514, 1532,
	1549, 1567, 1585, 1604, 1622, 1641, 1660, 1679, 1699, 1719, 1739, 1759,
	1779, 1800, 1820, 1842, 1863, 1884, 1906, 1928, 1951, 1973, 1996, 2019,
	2043, 2066, 2090, 2114, 2139, 2164, 2189, 2214, 2240, 2266, 2292, 2318,
	2345, 2373, 2400, 2428, 2456, 2484, 2513, 2542, 2572, 2601, 2632, 2662,
	2693, 2724, 2756, 2788, 2820, 2852, 2886, 2919, 2953, 2987, 3022, 3057,
	3092, 3128, 3164, 3201, 3238, 3275, 3313, 3351, 3390, 3430, 3469, 3509,
	3550, 3591, 3633, 3675, 3717, 3760, 3804, 3848, 3893, 3938, 3983, 4029,
	4076, 4123, 4171, 4219, 4268, 4318, 4368, 4418, 4469, 4521, 4574, 4627,
	4680, 4734, 4789, 4845, 4901, 4957, 5015, 5073, 5132, 5191, 5251, 5312,
	5374, 5436, 5499, 5562, 5627, 5692, 5758, 5825, 5892, 5960, 6029, 6099,
	6170, 6241, 6313, 6387, 6461, 6535, 6611, 6688, 6765, 6843, 6923, 7003,
	7084, 7166, 7249, 7333, 7418, 7504, 7591, 7678, 7767, 7857, 7948, 8040,
	8133, 8228, 8323, 8419, 8517, 8615, 8715, 8816, 8918, 9021, 9126, 9232,
	9338, 9447, 9556, 9667, 9779, 9892, 10006, 10122, 10240, 10358, 10478,
	10599, 10722, 10846, 10972, 11099, 11227, 11357, 11489, 11622, 11757,
	11893, 12030, 12170, 12311, 12453, 12597, 12743, 12891, 13040, 13191,
	13344, 13498, 13655, 13813, 13973, 14135, 14298, 14464, 14631, 14801,
	14972, 15146, 15321, 15498, 15678, 15859, 16043, 16229, 16417, 16607,
	16799, 16994, 17190, 17390, 17591, 17795, 18001, 18209, 18420, 18633,
	18849, 19067, 19288, 19511, 19737, 19966, 20197, 20431, 20668, 20907,
	21149, 21394, 21642, 21892, 22146, 22402, 22662, 22924, 23189, 23458,
	23730, 24004, 24282, 24564, 24848, 25136, 25427, 25721, 26019, 26320,
	26625, 26933, 27245, 27561, 27880, 28203, 28529, 28860, 29194, 29532,
	29874, 30220, 30570, 30924, 31282, 31644, 32011, 32381, 32756, 33135,
	33519, 33907, 34300, 34697, 35099, 35505, 35916, 36332, 36753, 37179,
	37609, 38045, 38485, 38931, 39382, 39838, 40299, 40766, 41238, 41715,
	42198, 42687, 43181, 43681, 44187, 44699, 45216, 45740, 46270, 46805,
	47347, 47896, 48450, 49011, 49579, 50153, 50734, 51321, 51915, 52517,
	53125, 53740, 54362, 54992, 55628, 56272, 56924, 57583, 58250, 58925,
	59607, 60297, 60995, 61702, 62416, 63139, 63870, 64610, 65358, 66114,
	66880, 67655, 68438, 69230, 70032, 70843, 71663, 72493, 73333, 74182,
	75041, 75910, 76789, 77678, 78577, 79487, 80408, 81339, 82281, 83233,
	84197, 85172, 86158, 87156, 88165, 89186, 90219, 91264, 92320, 93389,
	94471, 95565, 96671, 97791, 98923, 100069, 101227, 102400, 103585, 104785,
	105998, 107225, 108467, 109723, 110994, 112279, 113579, 114894, 116225,
	117570, 118932, 120309, 121702, 123111, 124537, 125979, 127438, 128913,
	130406, 131916, 133444, 134989, 136552, 138133, 139733, 141351, 142988,
	144643, 146318, 148013, 149726, 151460, 153214, 154988, 156783, 158598,
	160435, 162293, 164172, 166073, 167996, 169941, 171909, 173900, 175913,
	177950, 180011, 182095, 184204, 186337, 188495, 190677, 192885, 195119,
	197378, 199664, 201976, 204314, 206680, 209073, 211494, 213943, 216421,
	218927, 221462, 224026, 226620, 229245, 231899, 234584, 237301, 240049,
	242828, 245640, 248484, 251362, 254272, 257217};

const char * const shutter_mode_menu[] = {
	[NORMAL_EXPO]	= "Normal exposure",
	[SEQ_TRIGGER]	= "Sequential trigger",
	[FAST_TRIGGER]	= "Fast trigger",
};

static const struct v4l2_ctrl_ops imx900_ctrl_ops;
static struct v4l2_ctrl_config imx900_ctrl_data_rate[] = {
	{
		.ops = &imx900_ctrl_ops,
		.id = V4L2_CID_DATA_RATE,
		.name = "Data rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.max = IMX900_594_MBPS,
		.min = IMX900_2376_MBPS,
		.def = IMX900_1188_MBPS,
		.step = 1,
	},
};

static struct v4l2_ctrl_config imx900_ctrl_framerate[] = {
	{
		.ops = &imx900_ctrl_ops,
		.id = V4L2_CID_FRAME_RATE,
		.name = "Frame rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 205,
		.def = 42,
		.step = 1,
	},
};

static struct v4l2_ctrl_config imx900_ctrl_shutter_mode[] = {
	{
		.ops = &imx900_ctrl_ops,
		.id = V4L2_CID_SHUTTER_MODE,
		.name = "Shutter mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = NORMAL_EXPO,
		.max = FAST_TRIGGER,
		.def = NORMAL_EXPO,
		.step = 1,
	},
};

struct imx900_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *framerate;
	struct v4l2_ctrl *black_level;
	struct v4l2_ctrl *data_rate;
	//struct v4l2_ctrl *sync_mode;
	struct v4l2_ctrl *shutter_mode;
};

struct imx900 {
	struct i2c_client *i2c_client;
	unsigned int rst_gpio;
	unsigned int csi_id;
	unsigned int powered_on;

	struct v4l2_subdev sd;
	struct media_pad pads[IMX900_SENS_PADS_NUM];

	struct v4l2_mbus_framefmt format;
	vvcam_mode_info_t cur_mode;
	struct mutex lock;
	u32 stream_status;
	u32 resume_status;
	struct imx900_ctrls ctrls;
	u8 chromacity;
	struct regmap *regmap;
	const char *gmsl;
	struct device *ser_dev;
	struct device *dser_dev;
	struct gmsl_link_ctx g_ctx;
};

#define client_to_imx900(client)\
	container_of(i2c_get_clientdata(client), struct imx900, sd)

static inline struct imx900 *to_imx900_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx900, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct imx900,
				ctrls.handler)->sd;
}

static int imx900_set_dep_registers(struct imx900 *sensor);

static struct vvcam_mode_info_s pimx900_mode_info[] = {
	{
		.index			= 0,
		.size			= {
			.bounds_width  = IMX900_DEFAULT_WIDTH,
			.bounds_height = IMX900_DEFAULT_HEIGHT,
			.top		= 8,
			.left		= 8,
			.width		= 2048,
			.height		= 1536,
		},
		.hdr_mode		= SENSOR_MODE_LINEAR,
		.bit_width		= 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines	   = IMX900_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines	   = IMX900_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX900_LINE_TIME,

			.max_integration_line  = IMX900_MAX_BOUNDS_HEIGHT - 1,
			.min_integration_line  = IMX900_MIN_INTEGRATION_LINES,

			.max_again			= 16229, // 24db
			.min_again			= 1024,	 // 0 db
			.max_dgain			= 257217, // 48 db
			.min_dgain			= 1024,	 // 0db ,
			.gain_step			= 36,

			.start_exposure			= 1000 * 1024,
			.cur_fps			= 72 * 1024,
			.max_fps			= 72 * 1024,
			.min_fps			= 5 * 1024,
			.min_afps			= 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data	  = imx900_init_setting,
		.reg_data_count = ARRAY_SIZE(imx900_init_setting),
	},
	{
		.index			= 1,
		.size			= {
			.bounds_width  = IMX900_ROI_MODE_WIDTH,
			.bounds_height = IMX900_ROI_MODE_HEIGHT,
			.top		= 8,
			.left		= 8,
			.width		= 1920,
			.height		= 1080,
		},
		.hdr_mode		= SENSOR_MODE_LINEAR,
		.bit_width		= 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines	= IMX900_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines	= IMX900_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX900_LINE_TIME,

			.max_integration_line  = IMX900_MAX_BOUNDS_HEIGHT - 1,
			.min_integration_line  = IMX900_MIN_INTEGRATION_LINES,

			.max_again			= 16229,  // 24db
			.min_again			= 1024,	  // 0 db
			.max_dgain			= 257217, // 48 db
			.min_dgain			= 1024,   // 0db ,
			.gain_step			= 36,

			.start_exposure			= 500,	  //3 * 400 * 1024,
			.cur_fps			= 99 * 1024,
			.max_fps			= 99 * 1024,
			.min_fps			= 5 * 1024,
			.min_afps			= 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data	  = imx900_init_setting,
		.reg_data_count = ARRAY_SIZE(imx900_init_setting),
	},
	{
		.index		= 2,
		.size		= {
			.bounds_width  = IMX900_SUBSAMPLING2_MODE_WIDTH,
			.bounds_height = IMX900_SUBSAMPLING2_MODE_HEIGHT,
			.top		= 4,
			.left		= 4,
			.width		= 1024,
			.height		= 768,
		},
		.hdr_mode	   = SENSOR_MODE_LINEAR,
		.bit_width	  = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines	 = IMX900_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines	= IMX900_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX900_LINE_TIME,

			.max_integration_line  = IMX900_MAX_BOUNDS_HEIGHT - 1,
			.min_integration_line  = IMX900_MIN_INTEGRATION_LINES,

			.max_again			= 16229,	// 24db
			.min_again			= 1024,	 // 0 db
			.max_dgain			= 257217,   // 48 db
			.min_dgain			= 1024,	 // 0db ,
			.gain_step			= 36,

			.start_exposure			= 500,//3 * 400 * 1024,
			.cur_fps			= 135 * 1024,
			.max_fps			= 135 * 1024, // 240 for mono
			.min_fps			= 5 * 1024,
			.min_afps			= 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data	  = imx900_init_setting,
		.reg_data_count = ARRAY_SIZE(imx900_init_setting),
	},
	{
		.index			= 3,
		.size			= {
			.bounds_width  = IMX900_SUBSAMPLING10_MODE_WIDTH,
			.bounds_height = IMX900_SUBSAMPLING10_MODE_HEIGHT,
			.top			= 0,
			.left			= 8,
			.width			= 2048,
			.height			= 154,
		},
		.hdr_mode			= SENSOR_MODE_LINEAR,
		.bit_width			= 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines	 = IMX900_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines	= IMX900_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX900_LINE_TIME,

			.max_integration_line  = IMX900_MAX_BOUNDS_HEIGHT - 1,
			.min_integration_line  = IMX900_MIN_INTEGRATION_LINES,

			.max_again		 = 16229,	// 24db
			.min_again		 = 1024,	 // 0 db
			.max_dgain		 = 257217,   // 48 db
			.min_dgain		 = 1024,	 // 0db ,
			.gain_step		 = 36,

			.start_exposure		 = 500,//3 * 400 * 1024,
			.cur_fps		 = 450 * 1024,
			.max_fps		 = 450 * 1024,
			.min_fps		 = 5 * 1024,
			.min_afps		 = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data	  = imx900_init_setting,
		.reg_data_count = ARRAY_SIZE(imx900_init_setting),
	},
	{
		.index		  = 4,
		.size		   = {
			.bounds_width  = IMX900_BINNING_CROP_MODE_WIDTH,
			.bounds_height = IMX900_BINNING_CROP_MODE_HEIGHT,
			.top			= 8,
			.left			= 8,
			.width			= 1008,
			.height			= 704,
		},
		.hdr_mode			= SENSOR_MODE_LINEAR,
		.bit_width			= 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines	= IMX900_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines	= IMX900_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns   = IMX900_LINE_TIME,

			.max_integration_line   = IMX900_MAX_BOUNDS_HEIGHT - 1,
			.min_integration_line   = IMX900_MIN_INTEGRATION_LINES,

			.max_again		= 16229,  // 24db
			.min_again		= 1024,	  // 0 db
			.max_dgain		= 257217, // 48 db
			.min_dgain		= 1024,	  // 0db ,
			.gain_step		= 36,

			.start_exposure		= 500,//3 * 400 * 1024,
			.cur_fps		= 249 * 1024,
			.max_fps		= 249 * 1024,
			.min_fps		= 5 * 1024,
			.min_afps		= 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data		= imx900_init_setting,
		.reg_data_count = ARRAY_SIZE(imx900_init_setting),
	},
};

static int imx900_write_reg(struct imx900 *sensor, u16 reg, u8 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8Buf[3] = { 0 };
	int ret = 0;
	int num_retry = 0;

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	/*
	 * i2c communication occasionally fails with sensor sending a NACK without a clear reason.
	 * Retry sending a message for IMX900_MAX_RETRIES and report a problem.
	 */
	for (num_retry = 0; num_retry < IMX900_MAX_RETRIES; num_retry++) {
		ret = i2c_master_send(sensor->i2c_client, au8Buf, 3);
		if (ret >= 0)
			break;
		}

	if (ret < 0) {
		dev_err(dev, "Write reg error: reg=%x, val=%x, error= %d\n", reg, val, ret);
		return ret;
	}

	if (num_retry > 0)
		dev_warn(dev, "i2c communication passed after %d retries: reg=%x", num_retry, reg);

	return 0;
}

static int imx900_read_reg(struct imx900 *sensor, u16 reg, u8 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8RegBuf[2] = { 0 };
	u8 u8RdVal = 0;
	int num_retry = 0;
	int ret = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	for (num_retry = 0; num_retry < IMX900_MAX_RETRIES; num_retry++) {
		ret = i2c_master_send(sensor->i2c_client, au8RegBuf, 2);
		if (ret == 2)
			break;
	}

	if (ret != 2) {
		dev_err(dev, "Read reg error: reg=%x, error= %d\n", reg, ret);
		return ret;
	}
	if (num_retry > 0)
		dev_warn(dev, "i2c communication passed after %d retries: reg=%x", num_retry, reg);

	if (i2c_master_recv(sensor->i2c_client, &u8RdVal, 1) != 1) {
		dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return 0;
}

/*
 * i2c communication occasionally fails with sensor sending a NACK without a clear reason.
 * Retry sending a message for IMX900_MAX_RETRIES and report a problem.
 */
static int imx900_i2c_transfer(const struct i2c_client *const i2c_client,
				u8 *send_buf,
				const u8 send_buf_len)
{
	struct i2c_msg msg;
	int num_retry = 0;
	int ret = 0;

	msg.addr  = i2c_client->addr;
	msg.flags = i2c_client->flags;
	msg.buf   = send_buf;
	msg.len   = send_buf_len;

	for (num_retry = 0; num_retry < IMX900_MAX_RETRIES; num_retry++) {
		ret = i2c_transfer(i2c_client->adapter, &msg, 1);
		if (ret >= 0)
			break;
	}

	if (ret < 0) {
		pr_err("%s:i2c transfer error address= %d, error=%d\n", __func__, msg.addr, ret);
		return ret;
	}

	if (num_retry > 0) {
		u32 error_addr = (u32)(send_buf[0] << 8) + send_buf[1];

		pr_warn("%s: i2c communication passed after %d retries: reg=%x\n", __func__, num_retry, error_addr);
	}

	return 0;
}

static int imx900_write_reg_arry(struct imx900 *sensor,
				 struct vvcam_sccb_data_s *reg_arry,
				 u32 size)
{
	u8 *send_buf;
	u8 send_buf_len = 0;
	const u8 max_send_buff = 8;
	struct i2c_client *i2c_client = sensor->i2c_client;
	int i = 0;
	int ret = 0;

	send_buf = kmalloc(size + 2, GFP_KERNEL);

	if (!send_buf) {
		// checkpatch:ignore
		pr_err("enter %s failed to allocate memory for send_buffer\n", __func__);
		return -ENOMEM;
	}

	send_buf[send_buf_len++] = (reg_arry[0].addr >> 8) & 0xff;
	send_buf[send_buf_len++] = reg_arry[0].addr & 0xff;
	send_buf[send_buf_len++] = reg_arry[0].data & 0xff;

	for (i = 1; i < size; i++) {
		// To avoid i2c send errors limit the size of the buffer to 8
		if ((send_buf_len < max_send_buff) && (reg_arry[i].addr == (reg_arry[i-1].addr + 1))) {
			send_buf[send_buf_len++] = reg_arry[i].data & 0xff;
		} else {
			ret = imx900_i2c_transfer(i2c_client, send_buf, send_buf_len);
			if (ret < 0) {
				kfree(send_buf);
				return ret;
			}

			send_buf_len = 0;
			send_buf[send_buf_len++] =
				(reg_arry[i].addr >> 8) & 0xff;
			send_buf[send_buf_len++] =
				reg_arry[i].addr & 0xff;
			send_buf[send_buf_len++] =
				reg_arry[i].data & 0xff;
		}
	}

	if (send_buf_len > 0) {
		ret = imx900_i2c_transfer(i2c_client, send_buf, send_buf_len);
		if (ret < 0) {
			kfree(send_buf);
			return ret;
		}
	}

	kfree(send_buf);
	return ret;
}

/**
 * Image sensor chromacity probing
 *	  0: Color
 *	  1: Monochrome
 * Register access is possible after power-on, standby cancel and wait for 11.5 ms
 * Described in the IMX900_SupportPackage documentation - chapter "How to get sensor information"
 *
 */
static int imx900_chromacity_mode(struct imx900 *sensor)
{
	int err = 0;
	u8 chromacity;

	pr_debug("enter %s function\n", __func__);

	err = imx900_write_reg(sensor, STANDBY, 0x00);
	if (err) {
		pr_err("%s: error canceling standby mode\n", __func__);
		return err;
	}
	// "Internal regulator stabilization" time
	usleep_range(15000, 20000);

	err = imx900_read_reg(sensor, CHROMACITY, &chromacity);
	if (err) {
		pr_err("%s: error reading chromacity information register\n", __func__);
		return err;
	}

	err = imx900_write_reg(sensor, STANDBY, 0x01);
	if (err) {
		pr_err("%s: error setting standby mode\n", __func__);
		return err;
	}
	// "Internal regulator stabilization" time
	usleep_range(15000, 20000);

	chromacity = chromacity >> 7;
	sensor->chromacity = chromacity;

	pr_debug("%s: sensor is color(0)/monochrome(1): %d\n", __func__, chromacity);

	return err;
}

/**
 * Set supported shutter mode
 */
static int imx900_set_shutter_mode(struct imx900 *sensor, u32 val)
{
	u8 xmsta;
	int ret = 0;

	ret = imx900_read_reg(sensor, XMSTA, &xmsta);

	if (xmsta == MASTER_MODE && val == SEQ_TRIGGER) {
		pr_warn("%s: Sequential trigger isn't supported in master mode\n", __func__);
		goto default_state;
	}

	if (xmsta == SLAVE_MODE && val == FAST_TRIGGER) {
		pr_warn("%s: Fast trigger isn't supported in slave mode\n", __func__);
		goto default_state;
	}

	return ret;

default_state:
	ret  = imx900_write_reg(sensor, TRIGMODE, 0);
	ret |= imx900_write_reg(sensor, VINT_EN, 0x1E);
	if (ret)
		pr_err("%s: error setting exposure mode\n", __func__);

	return ret;
}

static int imx900_power_on(struct imx900 *sensor)
{
	pr_debug("enter %s function\n", __func__);
	mutex_lock(&sensor->lock);
	if (strcmp(sensor->gmsl, "gmsl")) {
		if (!gpio_is_valid(sensor->rst_gpio)) {
			pr_err("%s:reset pin is not valid\n", __func__);
			return -1;
		}
		gpio_set_value_cansleep(sensor->rst_gpio, 1);
	} else {
		/* For now no separate power on required for serializer device */
		pr_debug("%s: max96792_power_on\n", __func__);
		max96792_power_on(sensor->dser_dev, &sensor->g_ctx);
	}

	sensor->powered_on = 1;
	msleep(35);
	mutex_unlock(&sensor->lock);

	return 0;
}

static int imx900_power_off(struct imx900 *sensor)
{
	pr_debug("enter %s function\n", __func__);

	mutex_lock(&sensor->lock);

	if (strcmp(sensor->gmsl, "gmsl")) {
		if (!gpio_is_valid(sensor->rst_gpio)) {
			pr_err("%s:reset pin is not valid\n", __func__);
			return -1;
		}
		gpio_set_value_cansleep(sensor->rst_gpio, 0);
	} else {
		pr_debug("%s: max96792_power_off\n", __func__);
		max96792_power_off(sensor->dser_dev, &sensor->g_ctx);
	}

	sensor->powered_on = 0;
	msleep(128);

	mutex_unlock(&sensor->lock);
	return 0;
}

static int imx900_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx900 *sensor = client_to_imx900(client);
	int ret = 0;

	pr_debug("enter %s function\n", __func__);
	if (on)
		ret = imx900_power_on(sensor);
	else
		ret = imx900_power_off(sensor);

	if (ret < 0)
		return ret;
	return 0;
}

static int imx900_query_capability(struct imx900 *sensor, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strscpy((char *)pcap->driver, "imx900", sizeof(pcap->driver));
	sprintf((char *)pcap->bus_info, "csi%d", sensor->csi_id);
	if (sensor->i2c_client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}
	return 0;
}

static int imx900_query_supports(struct imx900 *sensor, void *parry)
{
	int ret = 0;
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
	uint32_t support_counts = ARRAY_SIZE(pimx900_mode_info);

	pr_debug("enter %s function\n", __func__);

	ret = copy_to_user(&psensor_mode_arry->count, &support_counts, sizeof(support_counts));
	ret |= copy_to_user(&psensor_mode_arry->modes, pimx900_mode_info,
			   sizeof(pimx900_mode_info));
	if (ret != 0) {
		pr_err("enter %s failed to allocate memory\n", __func__);
		ret = -ENOMEM;
	}
	return ret;

}

static int imx900_get_sensor_id(struct imx900 *sensor, void *pchip_id)
{
	int ret = 0;
	u16 chip_id = 900;

	pr_debug("enter %s function\n", __func__);
	ret = copy_to_user(pchip_id, &chip_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int imx900_get_reserve_id(struct imx900 *sensor, void *preserve_id)
{
	int ret = 0;
	u16 reserve_id = 900;

	pr_debug("enter %s function\n", __func__);
	ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	return ret;
}

static int imx900_get_sensor_mode(struct imx900 *sensor, void *pmode)
{
	int ret = 0;

	pr_debug("enter %s function\n", __func__);
	ret = copy_to_user(pmode, &sensor->cur_mode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0) {
		ret = -ENOMEM;
		pr_warn("error getting sensor mode %s\n", __func__);
	}
	return ret;
}

static int imx900_set_sensor_mode(struct imx900 *sensor, void *pmode)
{
	int ret = 0;
	int i = 0;
	struct vvcam_mode_info_s sensor_mode;

	pr_debug("enter %s function\n", __func__);
	ret = copy_from_user(&sensor_mode, pmode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0) {
		pr_err("enter %s: Failed to get sensor mode\n", __func__);
		return -ENOMEM;
	}
	for (i = 0; i < ARRAY_SIZE(pimx900_mode_info); i++) {
		if (pimx900_mode_info[i].index == sensor_mode.index) {
			memcpy(&sensor->cur_mode, &pimx900_mode_info[i],
				sizeof(struct vvcam_mode_info_s));
			return 0;
		}
	}

	pr_err("enter %s: Failed to set current sensor mode\n", __func__);
	return -ENXIO;
}

/**
 * Adjust HMAX register, and other properties for selected data rate
 */
static int imx900_calculate_line_time(struct imx900 *sensor)
{
	u8 hmax_low, hmax_high;
	u16 hmax;
	int err;

	pr_debug("enter %s function\n", __func__);

	err = imx900_read_reg(sensor, HMAX_HIGH, &hmax_high);
	err |= imx900_read_reg(sensor, HMAX_LOW, &hmax_low);
	if (err < 0) {
		pr_err("%s: unable to read hmax\n", __func__);
		return err;
	}
	hmax = ((u16)(hmax_high) << 8) | hmax_low;

	sensor->cur_mode.ae_info.one_line_exp_time_ns = (hmax*IMX900_G_FACTOR) / (IMX900_1ST_INCK);

	return 0;
}

static int imx900_get_current_datarate(struct imx900 *sensor, u8 *data_rate)
{
	u8 ths_reg;
	int err = 0;

	pr_debug("enter %s function\n", __func__);
	err = imx900_read_reg(sensor, THS_PREPARE_LOW, &ths_reg);

	if (err < 0) {
		pr_err("%s: could not read from ths register\n", __func__);
		return -1;
	}

	switch (ths_reg) {
	case 0x9F:
		*data_rate = IMX900_2376_MBPS;
		break;
	case 0x5F:
		*data_rate = IMX900_1485_MBPS;
		break;
	case 0x4F:
		*data_rate = IMX900_1188_MBPS;
		break;
	case 0x3F:
		*data_rate = IMX900_891_MBPS;
		break;
	case 0x2F:
		*data_rate = IMX900_594_MBPS;
		break;
	default:
		pr_err("%s: invalid value in ths registed, could not get data rate mode\n", __func__);
		break;
	}
	return 0;
}

/**
 * Adjust HMAX register, and other properties for selected data rate
 */
static int imx900_adjust_hmax_register(struct imx900 *sensor)
{
	int err = 0;
	u32 hmax = 0x262;
	u8 data_rate, numlanes;

	pr_debug("%s:++\n", __func__);

	err = imx900_get_current_datarate(sensor, &data_rate);
	pr_debug("%s: current datarate is equal to %d\n", __func__, data_rate);
	if (err < 0) {
		pr_err("%s: failed to get current datarate\n", __func__);
		return err;
	}

	err = imx900_read_reg(sensor, LANESEL, &numlanes);
	if (err < 0) {
		pr_err("%s: failed to get lane number selected\n", __func__);
		return err;
	}

	if ((sensor->cur_mode.bit_width == 12) && (sensor->cur_mode.bayer_pattern == BAYER_RGGB))
		sensor->format.code = MEDIA_BUS_FMT_SRGGB12_1X12;
	else if ((sensor->cur_mode.bit_width == 12) && (sensor->cur_mode.bayer_pattern == BAYER_GBRG))
		sensor->format.code = MEDIA_BUS_FMT_SGBRG12_1X12;
	else if ((sensor->cur_mode.bit_width == 10) && (sensor->cur_mode.bayer_pattern == BAYER_RGGB))
		sensor->format.code = MEDIA_BUS_FMT_SRGGB10_1X10;
	else if ((sensor->cur_mode.bit_width == 10) && (sensor->cur_mode.bayer_pattern == BAYER_GBRG))
		sensor->format.code = MEDIA_BUS_FMT_SGBRG10_1X10;
	else {
		pr_err("%s Invalid sensor->format.code: %d or bayer pattern : %d\n",
			__func__,
			sensor->format.code,
			sensor->cur_mode.bayer_pattern);
		return -EINVAL;
	}


	switch (data_rate) {
	case IMX900_2376_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0X152 : 0x22A;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x128 : 0x22A;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x152;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0A9 : 0x152;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x152;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0A9 : 0x152;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x16C : 0x2AB;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x168 : 0x2AB;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x16C;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0C7 : 0x16C;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x16C;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0B6 : 0x16C;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x262 : 0x32C;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x1A9 : 0x32C;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x262;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x131 : 0x262;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x262;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x131 : 0x262;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	case IMX900_1485_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x1CC : 0x369;
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x152 : 0x1CC;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0FE : 0x1CC;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x152;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0A9 : 0x152;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x234 : 0x438;
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x16C : 0x234;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x131 : 0x234;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x16C;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0B6 : 0x16C;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x29B : 0x506;
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x262 : 0x29B;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x165 : 0x29B;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x262;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x131 : 0x262;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	case IMX900_1188_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x23B : 0x43F;
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x152 : 0x23B;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x139 : 0x23B;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x152;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0B8 : 0x152;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x2BC : 0x541;
			break;
			case IMX900_TWO_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x179 : 0x2BC;
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x16C : 0x17A;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0D8 : 0x17A;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x33D : 0x643;
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x262 : 0x33D;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x1BA : 0x33D;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x262;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x131 : 0x262;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	case IMX900_891_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x2F4 : 0x5A4;
			break;
			case IMX900_TWO_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x19C : 0x2F4;
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x152 : 0x19C;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x0F0 : 0x19C;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x3A0 : 0x6FC;
			break;
			case IMX900_TWO_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x1F2 : 0x3A0;
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x16C : 0x1F3;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x11B : 0x1F3;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x44C : 0x854;
			break;
			case IMX900_TWO_LANE_MODE:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x262 : 0x44C;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x248 : 0x44C;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = 0x262;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x147 : 0x262;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	case IMX900_594_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x45E : 0x866;
			break;
			case IMX900_TWO_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x258 : 0x45C;
			break;
			case IMX900_MAX_CSI_LANES:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x158 : 0x25A;
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x560 : 0xA6A;
			break;
			case IMX900_TWO_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x2DA : 0x55E;
			break;
			case IMX900_MAX_CSI_LANES:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x198 : 0x2DA;
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x662 : 0xC6E;
			break;
			case IMX900_TWO_LANE_MODE:
				hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x35A : 0x660;
			break;
			case IMX900_MAX_CSI_LANES:
				if (sensor->chromacity == IMX900_COLOR) {
					hmax = (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) ? 0x262 : 0x35C;
				} else {//monochrome
					hmax = ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) || (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT)) ? 0x1D8 : 0x35C;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	default:
		/* Adjusment isn't needed */
		return 0;
	}

	err = imx900_write_reg(sensor, REGHOLD, 1);
	err |= imx900_write_reg(sensor, HMAX_LOW, hmax & 0xff);
	err |= imx900_write_reg(sensor, HMAX_HIGH, (hmax >> 8) & 0xff);
	err |= imx900_write_reg(sensor, REGHOLD, 0);
	if (err) {
		pr_err("%s: failed to set HMAX register\n", __func__);
		return err;
	}

	sensor->cur_mode.ae_info.one_line_exp_time_ns = (hmax*IMX900_G_FACTOR) / IMX900_1ST_INCK;

	pr_debug("%s:  HMAX: %u\n", __func__, hmax);

	return 0;
}

static int imx900_change_data_rate(struct imx900 *sensor, u32 data_rate)
{
	int ret = 0;
	u8 current_lane_mode;

	pr_debug("%s++\n", __func__);

	ret = imx900_read_reg(sensor, LANESEL, &current_lane_mode);

	if (current_lane_mode == IMX900_ONE_LANE_MODE || current_lane_mode == IMX900_TWO_LANE_MODE) {
		pr_warn("%s: 1 and 2 lane modes are not supported, switching to 4 lane mode\n", __func__);
		imx900_write_reg(sensor, LANESEL, IMX900_MAX_CSI_LANES);
	}

	pr_warn("%s: Setting data rate to value: %u\n", __func__, data_rate);
	if ((data_rate == IMX900_2376_MBPS) || (data_rate == IMX900_1485_MBPS)) {
		pr_warn("%s: Selected data rate is not supported, switching to 1188 data rate!\n", __func__);
		data_rate = IMX900_1188_MBPS;
	}
	switch (data_rate) {
	case IMX900_1188_MBPS:
		ret = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx900_1188_mbps, ARRAY_SIZE(imx900_1188_mbps));
		break;
	case IMX900_891_MBPS:
		ret = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx900_891_mbps, ARRAY_SIZE(imx900_891_mbps));
		break;
	case IMX900_594_MBPS:
		ret = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx900_594_mbps, ARRAY_SIZE(imx900_594_mbps));
		break;
	}
	if (!strcmp(sensor->gmsl, "gmsl")) {
		pr_debug("%s: Setting dser clock for data rate %u\n", __func__, data_rate);
		ret |= max96792_set_deser_clock(sensor->dser_dev, data_rate);
	}

	return ret;
}


static int imx900_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx900 *sensor = client_to_imx900(client);
	int err = 0;

	pr_debug("enter %s function\n", __func__);
	sensor->stream_status = enable;
	if (enable) {
		pr_info("Enable stream\n");
		if (!(strcmp(sensor->gmsl, "gmsl"))) {
			err = max96793_setup_streaming(sensor->ser_dev, sensor->format.code);
			if (err) {
				pr_err("%s: Unable to setup streaming for serializer max96793\n", __func__);
				goto exit;
			}
			err = max96792_setup_streaming(sensor->dser_dev, &sensor->i2c_client->dev);
			if (err) {
				pr_err("%s: Unable to setup streaming for deserializer max96792\n", __func__);
				goto exit;
			}
			err = max96792_start_streaming(sensor->dser_dev, &sensor->i2c_client->dev);
			if (err) {
				pr_err("%s: Unable to start gmsl streaming\n", __func__);
				goto exit;
			}
		}
		imx900_write_reg(sensor, STANDBY, 0x00);
		msleep(30);
		imx900_write_reg(sensor, XMSTA, 0x00);
		// 8 frame stabilisation - remove this?
		msleep(300);
	} else  {
		pr_info("Disable stream\n");
		if (!(strcmp(sensor->gmsl, "gmsl")))
			max96792_stop_streaming(sensor->dser_dev, &sensor->i2c_client->dev);

		imx900_write_reg(sensor, STANDBY, 0x01);
		msleep(30);
		imx900_write_reg(sensor, XMSTA, 0x01);
	}

	return 0;
exit:
	pr_err("%s: error setting stream\n", __func__);

	return err;
}

static int imx900_set_data_rate(struct imx900 *sensor, u32 data_rate)
{
	int ret = 0;
	bool stream_enabled = sensor->stream_status;

	pr_debug("enter %s data rate received: %u\n", __func__, data_rate);

	if (stream_enabled)
		imx900_s_stream(&sensor->sd, 0);

	ret = imx900_change_data_rate(sensor, data_rate);
	if (ret) {
		pr_err("%s: unable to set data rate\n", __func__);
		return ret;
	}

	ret = imx900_adjust_hmax_register(sensor);
	if (ret) {
		pr_err("%s: unable to adjust hmax\n", __func__);
		return ret;
	}

	ret = imx900_set_dep_registers(sensor);
	if (ret < 0) {
		pr_err("%s:unable to write dep registers to image sensor\n", __func__);
		return ret;
	}

	if (stream_enabled)
		imx900_s_stream(&sensor->sd, 1);

	return ret;
}

/**
 * Configure Global Shutter Operation
 * V interrupt is disabled in init mode table
 */
static int imx900_configure_shutter(struct imx900 *sensor)
{
	int err = 0;
	/* Default is Normal exposure */
	u8 trigen = 0;
	u8 vint_en = 0;
	u8 xmsta;

	pr_debug("enter %s function\n", __func__);

	err = imx900_read_reg(sensor, XMSTA, &xmsta);
	err |= imx900_read_reg(sensor, TRIGMODE, &trigen);

	switch (trigen) {
	case NORMAL_EXPO:
		trigen = 0;
		vint_en = 2;
		pr_debug("%s: Sensor is in Normal Exposure Mode\n", __func__);
		break;

	case SEQ_TRIGGER:
		if (xmsta == MASTER_MODE) {
			pr_warn("%s: Sequential Trigger Mode not supported in Master mode, switchig to default\n", __func__);
			break;
		}
		trigen = 9;
		vint_en = 1;
		pr_debug("%s: Sensor is in Sequential Trigger Mode\n", __func__);
		break;

	case FAST_TRIGGER:
		if (xmsta == SLAVE_MODE) {
			pr_warn("%s: Fast Trigger Mode not supported in Slave mode, switchig to default\n", __func__);
			break;
		}
		trigen = 10;
		pr_debug("%s: Sensor is in Fast Trigger Mode\n", __func__);
		break;

	default:
		pr_err("%s: unknown exposure mode.\n", __func__);
		return -EINVAL;
	}

	switch (sensor->cur_mode.size.bounds_height) {
	case IMX900_DEFAULT_HEIGHT:
		vint_en |= 0x1C;
		break;
	case IMX900_ROI_MODE_HEIGHT:
		vint_en |= 0x1C;
		break;
	case IMX900_SUBSAMPLING2_MODE_HEIGHT:
		vint_en |= (sensor->chromacity == IMX900_COLOR) ? 0x14 : 0x18;
		break;
	case IMX900_SUBSAMPLING10_MODE_HEIGHT:
		vint_en |= 0x14;
		break;
	case IMX900_BINNING_CROP_MODE_HEIGHT:
		vint_en |= 0x18;
		break;
	}

	err = imx900_write_reg(sensor, TRIGMODE, trigen);
	err |= imx900_write_reg(sensor, VINT_EN, vint_en);
	if (err) {
		pr_err("%s: error setting exposure mode\n", __func__);
		return err;
	}

	return 0;
}

//not defined in current version of datashet. Address and settings are selected based on IMX530 datasheet
/**
 * XVS & XHS are synchronizing/triggering pins
 *		XVS	XHS
 * 0xC0 - output, output
 * 0xF0 - hi-z,   hi-z
 */
static int imx900_configure_triggering_pins(struct imx900 *sensor)
{
	int err = 0;
	u8 xmsta;
	u8 sync_sel = 0xF0;

	pr_debug("enter %s function\n", __func__);

	err = imx900_write_reg(sensor, XMSTA, 0);
	err = imx900_read_reg(sensor, XMSTA, &xmsta);

	switch (xmsta) {
	case MASTER_MODE:
		/* XVS - output, XHS - output */
		sync_sel = 0xC0;
		pr_debug("%s: Sensor is in Master mode\n", __func__);
		break;

	case SLAVE_MODE:
		/* XVS - hi-z, XHS - hi-z */
		sync_sel = 0xF0;
		pr_debug("%s: Sensor is in Slave mode\n", __func__);
		break;

	default:
		pr_err("%s: unknown operation mode.\n", __func__);
		return -EINVAL;

	}

	err = imx900_write_reg(sensor, SYNCSEL, sync_sel);
	if (err) {
		pr_err("%s: error setting Slave mode\n", __func__);
		return err;
	}

	pr_debug("%s: XVS_XHS driver register: %x\n", __func__, sync_sel);

	return 0;
}

static int imx900_set_exp(struct imx900 *sensor, u32 exp, unsigned int which_control)
{
	int ret = 0;
	s32 integration_time_line;
	s32 frame_length;
	u32 integration_offset = IMX900_INTEGRATION_OFFSET;
	s32 reg_shs;
	u8 min_reg_shs;
	u8 reg_gmrwt2, reg_gmtwt;

	pr_debug("enter %s exposure received: %u\n", __func__, exp);

	frame_length = sensor->cur_mode.ae_info.curr_frm_len_lines;

	// from ISP driver
	if (which_control == 0)
		integration_time_line = (((exp >> 10) - integration_offset)
				* IMX900_K_FACTOR) / sensor->cur_mode.ae_info.one_line_exp_time_ns;
	else // from V4L2 control
		integration_time_line = (exp - integration_offset) * IMX900_K_FACTOR / sensor->cur_mode.ae_info.one_line_exp_time_ns;

	if (integration_time_line > sensor->cur_mode.ae_info.max_integration_line) {
		pr_info("%s: setting integration time to max value %u\n", __func__,
			sensor->cur_mode.ae_info.max_integration_line);
		integration_time_line = sensor->cur_mode.ae_info.max_integration_line;
		}

	if (integration_time_line < sensor->cur_mode.ae_info.min_integration_line) {
		pr_info("%s: setting integration time to min value %u\n", __func__,
			sensor->cur_mode.ae_info.min_integration_line);
		integration_time_line = sensor->cur_mode.ae_info.min_integration_line;
	}

	reg_shs = frame_length - integration_time_line;

	imx900_read_reg(sensor, GMTWT, &reg_gmtwt);
	imx900_read_reg(sensor, GMRWT2, &reg_gmrwt2);

	min_reg_shs = reg_gmtwt + reg_gmrwt2;

	if (reg_shs < min_reg_shs)
		reg_shs = min_reg_shs;
	else if (reg_shs > (frame_length - IMX900_MIN_INTEGRATION_LINES))
		reg_shs = frame_length - IMX900_MIN_INTEGRATION_LINES;

	pr_debug("enter %s exposure register: %u integration_time_line: %u frame lenght %u\n", __func__, reg_shs, integration_time_line, frame_length);
	ret = imx900_write_reg(sensor, REGHOLD, 1);
	ret |= imx900_write_reg(sensor, SHS_HIGH, (reg_shs >> 16) & 0xff);
	ret |= imx900_write_reg(sensor, SHS_MID, (reg_shs >> 8) & 0xff);
	ret |= imx900_write_reg(sensor, SHS_LOW, reg_shs & 0xff);
	ret |= imx900_write_reg(sensor, REGHOLD, 0);

	if (ret < 0)
		pr_err("%s Failed to set exposure exp: %u, shs register:  %u\n", __func__, exp, reg_shs);
	return ret;
}

/*
 * Gain in Sony sensors is measured in decibels [0-72]db, however, NXP
 * ISP pipeline uses voltages in fixed point format so one needs to convert
 * values with formula gain_db = 20 * (log(isp_gain >> 10)).

 * Gain step in sensor equals 0.3db with corresponding
 * register values in [0-240] range, so gain_reg = gain_db * 10 /3

 * Since math funcions are avoided in linux kernel we provide the table for
 * direct 1-1 tranformation between isp gains and gain register. This
 *approach is simpler and avoids some subtle numerical approximation errors.
 */
static u32 imx900_get_gain_reg(u32 gain)
{
	u32 l = 0;
	u32 r = IMX900_GAIN_REG_LEN - 1;
	u32 mid;
	u32 ret = 0;

	// check if the gain value is outside the isp bounds, this should never happen
	if (gain < gain_reg2times[0]) {
		pr_warn("%s:isp returned too small gain value: %u, setting to min gain\n",
			__func__, gain);
		return 0;
	} else if (gain > gain_reg2times[IMX900_GAIN_REG_LEN-1]) {
		pr_warn("%s:isp returned too large gain value: %u, setting to max gain\n",
			__func__, gain);
		return IMX900_GAIN_REG_LEN - 1;
	}

	// for given gain use binary search to find neighbours in the isp gain table
	while ((l + 1) < r) {
		mid = (l + r) / 2;
		if (gain_reg2times[mid] > gain)
			r = mid;
		else
			l = mid;
	}
	// return closest value
	ret = ((gain - gain_reg2times[l]) < (gain_reg2times[r] - gain)) ? l : r;
	return ret;
}

static int imx900_set_gain(struct imx900 *sensor, u32 gain, unsigned int which_control)
{
	int ret = 0;
	u32 gain_reg = 0;

	pr_debug("enter %s: gain received: %u control: %u\n", __func__, gain, which_control);

	// from ISP
	if (which_control == 0) {
		gain_reg = imx900_get_gain_reg(gain);
	} else { // from V4L2 control
		gain_reg = gain * IMX900_MAX_GAIN_DEC /
				(IMX900_MAX_GAIN_DB * 10);
	}

	pr_debug("%s: gain register: %u\n", __func__, gain_reg);
	ret = imx900_write_reg(sensor, REGHOLD, 1);
	ret |= imx900_write_reg(sensor, GAIN_HIGH, (gain_reg>>8) & 0xff);
	ret |= imx900_write_reg(sensor, GAIN_LOW, gain_reg & 0xff);
	ret |= imx900_write_reg(sensor, REGHOLD, 0);

	return ret;
}

static int imx900_set_black_level(struct imx900 *sensor, s64 val, u32 which_control)
{
	int ret = 0;

	pr_debug("enter %s black level: %lld from %u\n",  __func__, val, which_control);

	ret = imx900_write_reg(sensor, REGHOLD, 1);
	ret |= imx900_write_reg(sensor, BLKLEVEL_HIGH, (val>>8) & 0xff);
	ret |= imx900_write_reg(sensor, BLKLEVEL_LOW, val & 0xff);
	ret |= imx900_write_reg(sensor, REGHOLD, 0);
	if (ret) {
		pr_err("%s: BLACK LEVEL control error\n", __func__);
		return ret;
	}

	return 0;
}

static int imx900_set_fps(struct imx900 *sensor, u32 fps, u8 which_control)
{
	u32 fps_reg;
	u32 line_time;
	int ret = 0;
	u64 exposure_max_range, exposure_min_range;
	u8 min_reg_shs;
	u8 reg_gmrwt2, reg_gmtwt;

	pr_debug("enter %s fps received: %u\n", __func__, fps);
	if (which_control == 1)
		fps = fps << 10;

	line_time = sensor->cur_mode.ae_info.one_line_exp_time_ns;

	pr_debug("%s line_time: %u\n", __func__, line_time);


	if (fps > sensor->cur_mode.ae_info.max_fps)
		fps = sensor->cur_mode.ae_info.max_fps;
	else if (fps < sensor->cur_mode.ae_info.min_fps)
		fps = sensor->cur_mode.ae_info.min_fps;

	fps_reg = IMX900_G_FACTOR / ((fps >> 10) * line_time);
	pr_debug("enter %s vmax register: %u\n", __func__, fps_reg);

	imx900_read_reg(sensor, GMTWT, &reg_gmtwt);
	imx900_read_reg(sensor, GMRWT2, &reg_gmrwt2);

	min_reg_shs = reg_gmtwt + reg_gmrwt2;

	/* Update exposure range, before writing the new frame length */
	exposure_min_range = IMX900_MIN_INTEGRATION_LINES * line_time / IMX900_K_FACTOR;
	exposure_min_range += IMX900_INTEGRATION_OFFSET;
	exposure_max_range = (fps_reg - min_reg_shs) * line_time / IMX900_K_FACTOR;
	exposure_max_range += IMX900_INTEGRATION_OFFSET;

	ret = imx900_write_reg(sensor, REGHOLD, 1);
	ret |= imx900_write_reg(sensor, VMAX_HIGH, (u8)(fps_reg >> 16) & 0xff);
	ret |= imx900_write_reg(sensor, VMAX_MID, (u8)(fps_reg >> 8) & 0xff);
	ret |= imx900_write_reg(sensor, VMAX_LOW, (u8)(fps_reg & 0xff));
	ret |= imx900_write_reg(sensor, REGHOLD, 0);

	sensor->cur_mode.ae_info.cur_fps = fps;

	if (sensor->cur_mode.hdr_mode == SENSOR_MODE_LINEAR) {
		sensor->cur_mode.ae_info.max_integration_line = fps_reg - 4;
	} else {
		if (sensor->cur_mode.stitching_mode ==
			SENSOR_STITCHING_DUAL_DCG){
			sensor->cur_mode.ae_info.max_vsintegration_line = 44;
			sensor->cur_mode.ae_info.max_integration_line = fps_reg -
				4 - sensor->cur_mode.ae_info.max_vsintegration_line;
		} else {
			sensor->cur_mode.ae_info.max_integration_line = fps_reg - 4;
		}
	}
	sensor->cur_mode.ae_info.curr_frm_len_lines = fps_reg;
	return ret;
}

static int imx900_get_fps(struct imx900 *sensor, u32 *pfps)
{
	pr_debug("enter %s function\n", __func__);
	*pfps = sensor->cur_mode.ae_info.cur_fps;
	return 0;
}

static int imx900_set_test_pattern(struct imx900 *sensor, u32 val)
{
	int err;

	pr_debug("enter %s, pattern = %u\n", __func__, val);

	if (val) {
		err = imx900_write_reg(sensor, 0x3550, 0x07);
		if (err)
			goto fail;

		if (val == 4) {
			err = imx900_write_reg(sensor, 0x3551, 0x0A);
			if (err)
				goto fail;
		} else if (val == 5) {
			err = imx900_write_reg(sensor, 0x3551, 0x0B);
			if (err)
				goto fail;
		} else {
			err = imx900_write_reg(sensor, 0x3551, (u8)(val));
			if (err)
				goto fail;
		}
	} else {
		err = imx900_write_reg(sensor, 0x3550, 0x06);
		if (err)
			goto fail;
	}

	return 0;

fail:
	pr_err("%s: error setting test pattern\n", __func__);
	return err;
}

static int imx900_update_framerate_range(struct imx900 *sensor)
{
	u8 gmrwt, gmrwt2, gmtwt, gsdly;
	int err;

	err = imx900_read_reg(sensor, GMRWT, &gmrwt);
	err |= imx900_read_reg(sensor, GMRWT2, &gmrwt2);
	err |= imx900_read_reg(sensor, GMTWT, &gmtwt);
	err |= imx900_read_reg(sensor, GSDLY, &gsdly);
	pr_debug("enter %s function\n", __func__);

	if (err) {
		pr_err("%s: error reading gmrtw, gmrtw2, gmtwt, gsdly registers\n", __func__);
		return err;
	}

	switch (sensor->cur_mode.size.bounds_height) {
	case IMX900_DEFAULT_HEIGHT:
		sensor->cur_mode.ae_info.curr_frm_len_lines = IMX900_DEFAULT_HEIGHT + gmrwt + gmrwt2*2 + gmtwt + gsdly + 56;
		break;
	case IMX900_ROI_MODE_HEIGHT:
		sensor->cur_mode.ae_info.curr_frm_len_lines = IMX900_ROI_MODE_HEIGHT + gmrwt + gmrwt2*2 + gmtwt + gsdly + 56;
		break;
	case IMX900_SUBSAMPLING2_MODE_HEIGHT:
		if (sensor->chromacity == IMX900_COLOR)
			sensor->cur_mode.ae_info.curr_frm_len_lines = IMX900_SUBSAMPLING2_MODE_HEIGHT + gmrwt + gmrwt2*2 + gmtwt + gsdly + 34;
		else
			sensor->cur_mode.ae_info.curr_frm_len_lines = IMX900_SUBSAMPLING2_MODE_HEIGHT + gmrwt + gmrwt2*2 + gmtwt + gsdly + 38;
		break;
	case IMX900_SUBSAMPLING10_MODE_HEIGHT:
		sensor->cur_mode.ae_info.curr_frm_len_lines = IMX900_SUBSAMPLING10_MODE_HEIGHT + gmrwt + gmrwt2*2 + gmtwt + gsdly + 34;
		break;
	case IMX900_BINNING_CROP_MODE_HEIGHT:
		sensor->cur_mode.ae_info.curr_frm_len_lines = IMX900_BINNING_CROP_MODE_HEIGHT + gmrwt + gmrwt2*2 + gmtwt + gsdly + 38;
		break;
	}

	sensor->cur_mode.ae_info.max_fps = (IMX900_G_FACTOR * IMX900_M_FACTOR) /
									 (sensor->cur_mode.ae_info.curr_frm_len_lines * sensor->cur_mode.ae_info.one_line_exp_time_ns);

	return 0;

}

static int imx900_set_ratio(struct imx900 *sensor, void *pratio)
{
	int ret = 0;
	struct sensor_hdr_artio_s hdr_ratio;
	struct vvcam_ae_info_s *pae_info = &sensor->cur_mode.ae_info;

	pr_debug("enter %s function\n", __func__);
	ret = copy_from_user(&hdr_ratio, pratio, sizeof(hdr_ratio));

	if ((hdr_ratio.ratio_l_s != pae_info->hdr_ratio.ratio_l_s) ||
		(hdr_ratio.ratio_s_vs != pae_info->hdr_ratio.ratio_s_vs) ||
		(hdr_ratio.accuracy != pae_info->hdr_ratio.accuracy)) {
		pae_info->hdr_ratio.ratio_l_s = hdr_ratio.ratio_l_s;
		pae_info->hdr_ratio.ratio_s_vs = hdr_ratio.ratio_s_vs;
		pae_info->hdr_ratio.accuracy = hdr_ratio.accuracy;
		/*imx900 vs exp is limited for isp,so no need update max exp*/
	}

	return 0;
}

static int imx900_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct imx900 *sensor = to_imx900_dev(sd);
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */
	if (sensor->powered_on == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		ret = imx900_set_gain(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx900_set_exp(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx900_set_test_pattern(sensor, ctrl->val);
		break;
	case V4L2_CID_FRAME_RATE:
		ret = imx900_set_fps(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_BLACK_LEVEL:
		ret = imx900_set_black_level(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_DATA_RATE:
		ret = imx900_set_data_rate(sensor, ctrl->val);
		break;
	case V4L2_CID_SHUTTER_MODE:
		ret = imx900_set_shutter_mode(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops imx900_ctrl_ops = {
	.s_ctrl = imx900_s_ctrl,
};

static int imx900_get_format_code(struct imx900 *sensor, u32 *code)
{
	pr_debug("enter %s function\n", __func__);
	switch (sensor->cur_mode.bayer_pattern) {
	case BAYER_RGGB:
		if (sensor->cur_mode.bit_width == 8)
			*code = MEDIA_BUS_FMT_SRGGB8_1X8;
		else if (sensor->cur_mode.bit_width == 10)
			*code = MEDIA_BUS_FMT_SRGGB10_1X10;
		else
			*code = MEDIA_BUS_FMT_SRGGB12_1X12;
		break;
	case BAYER_GRBG:
		if (sensor->cur_mode.bit_width == 8)
			*code = MEDIA_BUS_FMT_SGRBG8_1X8;
		else if (sensor->cur_mode.bit_width == 10)
			*code = MEDIA_BUS_FMT_SGRBG10_1X10;
		else
			*code = MEDIA_BUS_FMT_SGRBG12_1X12;
		break;
	case BAYER_GBRG:
		if (sensor->cur_mode.bit_width == 8)
			*code = MEDIA_BUS_FMT_SGBRG8_1X8;
		else if (sensor->cur_mode.bit_width == 10)
			*code = MEDIA_BUS_FMT_SGBRG10_1X10;
		else
			*code = MEDIA_BUS_FMT_SGBRG12_1X12;
		break;
	case BAYER_BGGR:
		if (sensor->cur_mode.bit_width == 8)
			*code = MEDIA_BUS_FMT_SBGGR8_1X8;
		else if (sensor->cur_mode.bit_width == 10)
			*code = MEDIA_BUS_FMT_SBGGR10_1X10;
		else
			*code = MEDIA_BUS_FMT_SBGGR12_1X12;
		break;
	default:
		/*nothing need to do*/
		break;
	}
	return 0;
}

static int imx900_parse_dt(struct imx900 *sensor, struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	const struct of_device_id *match;
	const char *gmsl;
	int err;

	pr_debug("enter %s function\n", __func__);

	if (!node) {
		pr_err("%s: Node is empty\n", __func__);
		return -EINVAL;
	}

	match = of_match_device(imx900_of_match, &client->dev);
	if (!match) {
		pr_err("Failed to find matching dt id\n");
		return -EFAULT;
	}

	err = of_property_read_string(node, "gmsl", &gmsl);
	if (err) {
		pr_warn("initializing mipi...\n");
		sensor->gmsl = "mipi";
	} else if (!strcmp(gmsl, "gmsl")) {
		pr_warn("initializing GMSL...\n");
		sensor->gmsl = "gmsl";
	}

	pr_debug("%s: Succesfully parsed device tree\n", __func__);
	return 0;
}

static int imx900_set_pixel_format(struct imx900 *sensor)
{
	int err;
	u8 adbit_monosel;

	pr_debug("enter %s function\n", __func__);

	switch (sensor->format.code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
		adbit_monosel = (sensor->chromacity == IMX900_COLOR) ? 0x21 : 0x25;
		err = imx900_write_reg(sensor, ADBIT_MONOSEL, adbit_monosel);
		if (err) {
			pr_err("%s: error setting chromacity pixel format\n", __func__);
			return err;
		}
		err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx900_8bit_mode, ARRAY_SIZE(imx900_8bit_mode));
		break;
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
		adbit_monosel = (sensor->chromacity == IMX900_COLOR) ? 0x01 : 0x05;
		err = imx900_write_reg(sensor, ADBIT_MONOSEL, adbit_monosel);
		if (err) {
			pr_err("%s: error setting chromacity pixel format\n", __func__);
			return err;
		}
		err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx900_10bit_mode, ARRAY_SIZE(imx900_10bit_mode));
		break;
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		adbit_monosel = (sensor->chromacity == IMX900_COLOR) ? 0x11 : 0x15;
		err = imx900_write_reg(sensor, ADBIT_MONOSEL, adbit_monosel);
		if (err) {
			pr_err("%s: error setting chromacity pixel format\n", __func__);
			return err;
		}
		err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx900_12bit_mode, ARRAY_SIZE(imx900_12bit_mode));
		break;
	default:
		pr_err("%s: unknown pixel format\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int imx900_set_mode_additional(struct imx900 *sensor)
{
	int err;

	pr_debug("enter %s function\n", __func__);

	switch (sensor->cur_mode.size.bounds_height) {
	case IMX900_DEFAULT_HEIGHT:
		err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_allPixel_roi, ARRAY_SIZE(mode_allPixel_roi));
	break;
	case IMX900_ROI_MODE_HEIGHT:
		err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_allPixel_roi, ARRAY_SIZE(mode_allPixel_roi));
	break;
	case IMX900_SUBSAMPLING2_MODE_HEIGHT:
		if (sensor->chromacity == IMX900_COLOR)
			err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_subsampling2_color, ARRAY_SIZE(mode_subsampling2_color));
		else
			err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_subsampling2_binning_mono, ARRAY_SIZE(mode_subsampling2_binning_mono));
	break;
	case IMX900_SUBSAMPLING10_MODE_HEIGHT:
		err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_subsampling10, ARRAY_SIZE(mode_subsampling10));
	break;
	case IMX900_BINNING_CROP_MODE_HEIGHT:
		err = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_subsampling2_binning_mono, ARRAY_SIZE(mode_subsampling2_binning_mono));
	break;
	}

	if (err) {
		pr_err("%s: unable to set additional mode registers\n", __func__);
		return err;
	}

	return 0;

}

static int imx900_write_mode_dep_chromacity(struct imx900 *sensor, struct vvcam_sccb_data_s *table1, struct vvcam_sccb_data_s *table2, struct vvcam_sccb_data_s *table3)
{
	int err = 0;

	if (sensor->chromacity == IMX900_COLOR) {
		if (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT)
			err = imx900_write_reg_arry(sensor, table1, 8);
		else
			err = imx900_write_reg_arry(sensor, table2, 8);
	} else {//monochrome
		if ((sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT)
		|| (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT))
			err = imx900_write_reg_arry(sensor, table3, 8);
		else
			err = imx900_write_reg_arry(sensor, table2, 8);
	}

	return err;
}

static int imx900_set_dep_registers(struct imx900 *sensor)
{
	int err = 0;
	u8 data_rate;
	u8 numlanes;

	pr_debug("enter %s function\n", __func__);

	err = imx900_get_current_datarate(sensor, &data_rate);
	if (err < 0)
		pr_err("%s: could not determine data rate\n", __func__);

	err = imx900_read_reg(sensor, LANESEL, &numlanes);
	if (err < 0)
		pr_err("%s: could not get number of lines\n", __func__);

	switch (data_rate) {
	case IMX900_2376_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x8_1LANE,
						allpixel_roi_subsampling10_2376MBPS_1x8_1lane,
						subsampling2_binning_MONO_2376MBPS_1x8_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x8_2LANE,
						allpixel_roi_subsampling10_2376MBPS_1x8_2lane,
						subsampling2_binning_MONO_2376MBPS_1x8_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x8_4LANE,
						allpixel_roi_subsampling10_2376MBPS_1x8_4lane,
						subsampling2_binning_MONO_2376MBPS_1x8_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x10_1LANE,
						allpixel_roi_subsampling10_2376MBPS_1x10_1lane,
						subsampling2_binning_MONO_2376MBPS_1x10_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x10_2LANE,
						allpixel_roi_subsampling10_2376MBPS_1x10_2lane,
						subsampling2_binning_MONO_2376MBPS_1x10_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x10_4LANE,
						allpixel_roi_subsampling10_2376MBPS_1x10_4lane,
						subsampling2_binning_MONO_2376MBPS_1x10_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x12_1LANE,
						allpixel_roi_subsampling10_2376MBPS_1x12_1lane,
						subsampling2_binning_MONO_2376MBPS_1x12_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x12_2LANE,
						allpixel_roi_subsampling10_2376MBPS_1x12_2lane,
						subsampling2_binning_MONO_2376MBPS_1x12_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_2376MBPS_1x12_4LANE,
						allpixel_roi_subsampling10_2376MBPS_1x12_4lane,
						subsampling2_binning_MONO_2376MBPS_1x12_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
			break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
	break;
	case IMX900_1485_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x8_1LANE,
						allpixel_roi_subsampling10_1485MBPS_1x8_1lane,
						subsampling2_binning_MONO_1485MBPS_1x8_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x8_2LANE,
						allpixel_roi_subsampling10_1485MBPS_1x8_2lane,
						subsampling2_binning_MONO_1485MBPS_1x8_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x8_4LANE,
						allpixel_roi_subsampling10_1485MBPS_1x8_4lane,
						subsampling2_binning_MONO_1485MBPS_1x8_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x10_1LANE,
						allpixel_roi_subsampling10_1485MBPS_1x10_1lane,
						subsampling2_binning_MONO_1485MBPS_1x10_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x10_2LANE,
						allpixel_roi_subsampling10_1485MBPS_1x10_2lane,
						subsampling2_binning_MONO_1485MBPS_1x10_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x10_4LANE,
						allpixel_roi_subsampling10_1485MBPS_1x10_4lane,
						subsampling2_binning_MONO_1485MBPS_1x10_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x12_1LANE,
						allpixel_roi_subsampling10_1485MBPS_1x12_1lane,
						subsampling2_binning_MONO_1485MBPS_1x12_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x12_2LANE,
						allpixel_roi_subsampling10_1485MBPS_1x12_2lane,
						subsampling2_binning_MONO_1485MBPS_1x12_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1485MBPS_1x12_4LANE,
						allpixel_roi_subsampling10_1485MBPS_1x12_4lane,
						subsampling2_binning_MONO_1485MBPS_1x12_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	case IMX900_1188_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x8_1LANE,
						allpixel_roi_subsampling10_1188MBPS_1x8_1lane,
						subsampling2_binning_MONO_1188MBPS_1x8_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x8_2LANE,
						allpixel_roi_subsampling10_1188MBPS_1x8_2lane,
						subsampling2_binning_MONO_1188MBPS_1x8_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x8_4LANE,
						allpixel_roi_subsampling10_1188MBPS_1x8_4lane,
						subsampling2_binning_MONO_1188MBPS_1x8_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x10_1LANE,
						allpixel_roi_subsampling10_1188MBPS_1x10_1lane,
						subsampling2_binning_MONO_1188MBPS_1x10_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x10_2LANE,
						allpixel_roi_subsampling10_1188MBPS_1x10_2lane,
						subsampling2_binning_MONO_1188MBPS_1x10_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x10_4LANE,
						allpixel_roi_subsampling10_1188MBPS_1x10_4lane,
						subsampling2_binning_MONO_1188MBPS_1x10_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x12_1LANE,
						allpixel_roi_subsampling10_1188MBPS_1x12_1lane,
						subsampling2_binning_MONO_1188MBPS_1x12_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x12_2LANE,
						allpixel_roi_subsampling10_1188MBPS_1x12_2lane,
						subsampling2_binning_MONO_1188MBPS_1x12_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_1188MBPS_1x12_4LANE,
						allpixel_roi_subsampling10_1188MBPS_1x12_4lane,
						subsampling2_binning_MONO_1188MBPS_1x12_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	case IMX900_891_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x8_1LANE,
						allpixel_roi_subsampling10_891MBPS_1x8_1lane,
						subsampling2_binning_MONO_891MBPS_1x8_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x8_2LANE,
						allpixel_roi_subsampling10_891MBPS_1x8_2lane,
						subsampling2_binning_MONO_891MBPS_1x8_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x8_4LANE,
						allpixel_roi_subsampling10_891MBPS_1x8_4lane,
						subsampling2_binning_MONO_891MBPS_1x8_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x10_1LANE,
						allpixel_roi_subsampling10_891MBPS_1x10_1lane,
						subsampling2_binning_MONO_891MBPS_1x10_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x10_2LANE,
						allpixel_roi_subsampling10_891MBPS_1x10_2lane,
						subsampling2_binning_MONO_891MBPS_1x10_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x10_4LANE,
						allpixel_roi_subsampling10_891MBPS_1x10_4lane,
						subsampling2_binning_MONO_891MBPS_1x10_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x12_1LANE,
						allpixel_roi_subsampling10_891MBPS_1x12_1lane,
						subsampling2_binning_MONO_891MBPS_1x12_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x12_2LANE,
						allpixel_roi_subsampling10_891MBPS_1x12_2lane,
						subsampling2_binning_MONO_891MBPS_1x12_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_891MBPS_1x12_4LANE,
						allpixel_roi_subsampling10_891MBPS_1x12_4lane,
						subsampling2_binning_MONO_891MBPS_1x12_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	case IMX900_594_MBPS:
		switch (sensor->format.code) {
		case MEDIA_BUS_FMT_SRGGB8_1X8:
		case MEDIA_BUS_FMT_SGBRG8_1X8:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x8_1LANE,
						allpixel_roi_subsampling10_594MBPS_1x8_1lane,
						subsampling2_binning_MONO_594MBPS_1x8_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x8_2LANE,
						allpixel_roi_subsampling10_594MBPS_1x8_2lane,
						subsampling2_binning_MONO_594MBPS_1x8_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x8_4LANE,
						allpixel_roi_subsampling10_594MBPS_1x8_4lane,
						subsampling2_binning_MONO_594MBPS_1x8_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB10_1X10:
		case MEDIA_BUS_FMT_SGBRG10_1X10:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x10_1LANE,
						allpixel_roi_subsampling10_594MBPS_1x10_1lane,
						subsampling2_binning_MONO_594MBPS_1x10_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x10_2LANE,
						allpixel_roi_subsampling10_594MBPS_1x10_2lane,
						subsampling2_binning_MONO_594MBPS_1x10_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x10_4LANE,
						allpixel_roi_subsampling10_594MBPS_1x10_4lane,
						subsampling2_binning_MONO_594MBPS_1x10_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		case MEDIA_BUS_FMT_SRGGB12_1X12:
		case MEDIA_BUS_FMT_SGBRG12_1X12:
			switch (numlanes) {
			case IMX900_ONE_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x12_1LANE,
						allpixel_roi_subsampling10_594MBPS_1x12_1lane,
						subsampling2_binning_MONO_594MBPS_1x12_1LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_TWO_LANE_MODE:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x12_2LANE,
						allpixel_roi_subsampling10_594MBPS_1x12_2lane,
						subsampling2_binning_MONO_594MBPS_1x12_2LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			case IMX900_MAX_CSI_LANES:
				err = imx900_write_mode_dep_chromacity(sensor,
						subsampling2_COLOR_594MBPS_1x12_4LANE,
						allpixel_roi_subsampling10_594MBPS_1x12_4lane,
						subsampling2_binning_MONO_594MBPS_1x12_4LANE);
				if (err) {
					pr_err("%s: error setting dep register table\n", __func__);
					return err;
				}
			break;
			default:
				 pr_err("%s: unknown lane mode\n", __func__);
				return 0;
			}
		break;
		default:
			pr_err("%s: unknown pixel format\n", __func__);
			return 0;
		}
		break;
	default:
		/* Adjusment isn't needed */
		return 0;
	}

	return err;
}

static int imx900_gmsl_serdes_setup(struct imx900 *priv)
{
	int err = 0;
	int des_err = 0;
	struct device *dev;

	pr_debug("enter %s function\n", __func__);

	if (!priv || !priv->ser_dev || !priv->dser_dev || !priv->i2c_client)
		return -EINVAL;

	dev = &priv->i2c_client->dev;

	mutex_lock(&priv->lock);

	err = max96792_reset_control(priv->dser_dev, &priv->i2c_client->dev);

	err = max96792_gmsl3_setup(priv->dser_dev);
	if (err) {
		pr_err("deserializer gmsl setup failed\n");//
		goto error;
	}

	err = max96793_gmsl3_setup(priv->ser_dev);
	if (err) {
		pr_err("serializer gmsl setup failed\n");
		goto error;
	}

	/* setup serdes addressing and control pipeline */
	err = max96792_setup_link(priv->dser_dev, &priv->i2c_client->dev);
	if (err) {
		pr_err("gmsl deserializer link config failed\n");
		goto error;
	}

	pr_debug("%s: max96793_setup_control\n", __func__);
	err = max96793_setup_control(priv->ser_dev);

	/* proceed even if ser setup failed, to setup deser correctly */
	if (err)
		pr_err("gmsl serializer setup failed\n");

	err = max96793_gpio10_xtrig1_setup(priv->ser_dev, "mipi");
	if (err) {
		pr_err("gmsl serializer gpio10/xtrig1 pin config failed\n");
		goto error;
	}

	dev_dbg(dev, "%s: max96792_setup_control\n", __func__);
	des_err = max96792_setup_control(priv->dser_dev, &priv->i2c_client->dev);
	if (des_err) {
		pr_err("gmsl deserializer setup failed\n");
		/* overwrite err only if deser setup also failed */
		// err = des_err;
	}

error:
	mutex_unlock(&priv->lock);
	return err;
}


static void imx900_gmsl_serdes_reset(struct imx900 *priv)
{
	mutex_lock(&priv->lock);

	/* reset serdes addressing and control pipeline */
	max96793_reset_control(priv->ser_dev);
	max96792_reset_control(priv->dser_dev, &priv->i2c_client->dev);

	max96792_power_off(priv->dser_dev, &priv->g_ctx);

	mutex_unlock(&priv->lock);
}

static int imx900_enum_mbus_code(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *state,
					struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx900 *sensor = client_to_imx900(client);
	u32 cur_code = MEDIA_BUS_FMT_SRGGB12_1X12;

	pr_debug("enter %s function\n", __func__);
	if (code->index > 0)
		return -EINVAL;
	imx900_get_format_code(sensor, &cur_code);
	code->code = cur_code;

	return 0;
}

static int imx900_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx900 *sensor = client_to_imx900(client);

	mutex_lock(&sensor->lock);
	pr_debug("enter %s function\n", __func__);
	if ((fmt->format.width != sensor->cur_mode.size.bounds_width) ||
		(fmt->format.height != sensor->cur_mode.size.bounds_height)) {
		pr_err("%s:set sensor format %dx%d error\n",
			__func__, fmt->format.width, fmt->format.height);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	imx900_get_format_code(sensor, &fmt->format.code);
	fmt->format.field = V4L2_FIELD_NONE;
	sensor->format = fmt->format;

	ret = imx900_write_reg_arry(sensor,
		(struct vvcam_sccb_data_s *)sensor->cur_mode.preg_data,
		sensor->cur_mode.reg_data_count);
	
	if (ret < 0) {
		pr_err("%s:imx900_write_reg_arry error\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx900_chromacity_mode(sensor);
	if (ret < 0) {
		pr_err("%s:unable to get chromacity information\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx900_set_pixel_format(sensor);
	if (ret < 0) {
		pr_err("%s:imx900_write_reg_arry error, failed to set pixel format\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx900_set_mode_additional(sensor);
	if (ret < 0) {
		pr_err("%s:unable to set additional sensor mode settings\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx900_configure_triggering_pins(sensor);
	if (ret < 0) {
		pr_err("%s:imx900_write_reg_arry error, unable configure XVS/XHS pins\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	if (sensor->cur_mode.size.bounds_height == IMX900_DEFAULT_HEIGHT) {
		ret = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_2064x1552, ARRAY_SIZE(mode_2064x1552));
		if (ret < 0) {
			pr_err("%s:imx900_write_reg_arry error, failed to set up resolution\n", __func__);
			return -EINVAL;
		}
	} else if (sensor->cur_mode.size.bounds_height == IMX900_ROI_MODE_HEIGHT) {
		ret = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_1920x1080, ARRAY_SIZE(mode_1920x1080));
		if (ret < 0) {
			pr_err("%s:imx900_write_reg_arry error, failed to set up resolution\n", __func__);
			return -EINVAL;
		}
	} else if (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING2_MODE_HEIGHT) {
		ret = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_1032x776, ARRAY_SIZE(mode_1032x776));
		if (ret < 0) {
			pr_err("%s:imx900_write_reg_arry error, failed to set up resolution\n", __func__);
			return -EINVAL;
		}
	} else if (sensor->cur_mode.size.bounds_height == IMX900_SUBSAMPLING10_MODE_HEIGHT) {
		ret = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_2064x154, ARRAY_SIZE(mode_2064x154));
		if (ret < 0) {
			pr_err("%s:imx900_write_reg_arry error, failed to set up resolution\n", __func__);
			return -EINVAL;
		}
	} else if (sensor->cur_mode.size.bounds_height == IMX900_BINNING_CROP_MODE_HEIGHT) {
		ret = imx900_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)mode_1024x720, ARRAY_SIZE(mode_1024x720));
		if (ret < 0) {
			pr_err("%s:imx900_write_reg_arry error, failed to set up resolution\n", __func__);
			return -EINVAL;
		}
	}

	ret = imx900_s_ctrl(sensor->ctrls.data_rate);
	if (ret < 0) {
		pr_err("%s:unable to set data rate\n", __func__);
		return -EINVAL;
	}

	ret = imx900_set_dep_registers(sensor);
	if (ret < 0) {
		pr_err("%s:unable to write dep registers to image sensor\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx900_configure_shutter(sensor);
	if (ret < 0) {
		pr_err("%s:unable to set mode\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx900_calculate_line_time(sensor);
	if (ret < 0) {
		pr_err("%s:unable to calculate line time\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx900_update_framerate_range(sensor);
	if (ret < 0) {
		pr_err("%s:unable to update framerate range\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	mutex_unlock(&sensor->lock);
	return 0;
}

static int imx900_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx900 *sensor = client_to_imx900(client);

	pr_debug("enter %s function\n", __func__);
	mutex_lock(&sensor->lock);
	fmt->format = sensor->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

static long imx900_priv_ioctl(struct v4l2_subdev *sd,
							  unsigned int cmd,
							  void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx900 *sensor = client_to_imx900(client);
	long ret = 0;
	struct vvcam_sccb_data_s sensor_reg;

	pr_debug("enter %s %u\n", __func__, cmd);
	mutex_lock(&sensor->lock);
	switch (cmd) {
	case VVSENSORIOC_S_POWER:
		ret = 0;
		break;
	case VVSENSORIOC_S_CLK:
		ret = 0;
		break;
	case VVSENSORIOC_G_CLK:
		ret = 0;
		break;
	case VVSENSORIOC_RESET:
		ret = 0;
		break;
	case VIDIOC_QUERYCAP:
		ret = imx900_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		ret = imx900_query_supports(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = imx900_get_sensor_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID:
		ret = imx900_get_reserve_id(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = imx900_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = imx900_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		ret = imx900_s_stream(&sensor->sd, *(int *)arg);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx900_write_reg(sensor, sensor_reg.addr,
			sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx900_read_reg(sensor, sensor_reg.addr,
			(u8 *)&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg,
			sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_S_LONG_EXP:
		ret = 0;
		break;
	case VVSENSORIOC_S_EXP:
		ret = imx900_set_exp(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_S_VSEXP:
		ret = 0;
		break;
	case VVSENSORIOC_S_LONG_GAIN:
		ret = 0;
		break;
	case VVSENSORIOC_S_GAIN:
		ret = imx900_set_gain(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_S_VSGAIN:
		ret = 0;
		break;
	case VVSENSORIOC_S_FPS:
		ret = imx900_set_fps(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_G_FPS:
		ret = imx900_get_fps(sensor, (u32 *)arg);
		break;
	case VVSENSORIOC_S_HDR_RADIO:
		ret = imx900_set_ratio(sensor, arg);
		break;
	case VVSENSORIOC_S_BLC:
		ret = imx900_set_black_level(sensor, *(s64 *)arg, 0);
		break;
	case VVSENSORIOC_S_WB:
		ret = 0;
		break;
	case VVSENSORIOC_G_EXPAND_CURVE:
		ret = 0;
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		ret = imx900_set_test_pattern(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_DATA_RATE:
		ret = imx900_set_data_rate(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_SHUTTER_MODE:
		ret = imx900_set_shutter_mode(sensor, *(u32 *)arg);
		break;
	default:
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_video_ops imx900_subdev_video_ops = {
	.s_stream = imx900_s_stream,
};

static const struct v4l2_subdev_pad_ops imx900_subdev_pad_ops = {
	.enum_mbus_code = imx900_enum_mbus_code,
	.set_fmt = imx900_set_fmt,
	.get_fmt = imx900_get_fmt,
};

static const struct v4l2_subdev_core_ops imx900_subdev_core_ops = {
	.s_power = imx900_s_power,
	.ioctl = imx900_priv_ioctl,
};

static const struct v4l2_subdev_ops imx900_subdev_ops = {
	.core  = &imx900_subdev_core_ops,
	.video = &imx900_subdev_video_ops,
	.pad   = &imx900_subdev_pad_ops,
};

static int imx900_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations imx900_sd_media_ops = {
	.link_setup = imx900_link_setup,
};

static int imx900_probe(struct i2c_client *client)
{
	int retval;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct imx900 *sensor;

	struct device_node *node = dev->of_node;
	struct device_node *ser_node;
	struct i2c_client *ser_i2c = NULL;
	struct device_node *dser_node;
	struct i2c_client *dser_i2c = NULL;
	struct device_node *gmsl;
	int value = 0xFFFF;
	const char *str_value;
	const char *str_value1[2];
	int  i;
	int err = 0;
	int default_black_level;

	pr_debug("enter %s function\n", __func__);

	sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor) {
		//checkpatch: ignore
		pr_err(" could not allocate memory for sensor\n");
		return -ENOMEM;
	}
	memset(sensor, 0, sizeof(*sensor));

	err = imx900_parse_dt(sensor, client);
	if (err < 0) {
		pr_err("could not parse dt\n");
		return err;
	}

	mutex_init(&sensor->lock);

	sensor->i2c_client = client;
	if (strcmp(sensor->gmsl, "gmsl")) {
		sensor->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
		if (!gpio_is_valid(sensor->rst_gpio))
			dev_warn(dev, "No sensor reset pin available");
		else {
			// TODO: add separate reset pin in dual mode
			retval = devm_gpio_request_one(dev,
							sensor->rst_gpio,
							GPIOF_OUT_INIT_LOW,
							"imx900_mipi_reset");
			if (retval < 0) {
				dev_warn(dev, "Failed to set reset pin\n");
				//return retval;
			}
		}
	}

	retval = of_property_read_u32(dev->of_node, "csi_id", &(sensor->csi_id));
	if (retval) {
		dev_err(dev, "csi id missing or invalid\n");
		return retval;
	}
	if (!(strcmp(sensor->gmsl, "gmsl"))) {

		err = of_property_read_u32(node, "reg", &sensor->g_ctx.sdev_reg);
		if (err < 0) {
			dev_err(dev, "reg not found\n");
			return err;
		}

		err = of_property_read_u32(node, "def-addr",
						&sensor->g_ctx.sdev_def);
		if (err < 0) {
			dev_err(dev, "def-addr not found\n");
			return err;
		}

		ser_node = of_parse_phandle(node, "gmsl-ser-device", 0);
		if (ser_node == NULL) {
			dev_err(dev,
				"missing %s handle\n",
					"gmsl-ser-device");
			return err;
		}

		err = of_property_read_u32(ser_node, "reg", &sensor->g_ctx.ser_reg);
		if (err < 0) {
			dev_err(dev, "serializer reg not found\n");
			return err;
		}

		ser_i2c = of_find_i2c_device_by_node(ser_node);
		of_node_put(ser_node);

		if (ser_i2c == NULL) {
			dev_err(dev, "missing serializer dev handle\n");
			return err;
		}
		if (ser_i2c->dev.driver == NULL) {
			dev_err(dev, "missing serializer driver\n");
			return err;
		}

		sensor->ser_dev = &ser_i2c->dev;

		dser_node = of_parse_phandle(node, "gmsl-dser-device", 0);
		if (dser_node == NULL) {
			dev_err(dev,
				"missing %s handle\n",
					"gmsl-dser-device");
			return err;
		}

		dser_i2c = of_find_i2c_device_by_node(dser_node);
		of_node_put(dser_node);

		if (dser_i2c == NULL) {
			dev_err(dev, "missing deserializer dev handle\n");
			return err;
		}
		if (dser_i2c->dev.driver == NULL) {
			dev_err(dev, "missing deserializer driver\n");
			return err;
		}

		sensor->dser_dev = &dser_i2c->dev;

		/* populate g_ctx from DT */
		gmsl = of_get_child_by_name(node, "gmsl-link");
		if (gmsl == NULL) {
			dev_err(dev, "missing gmsl-link device node\n");
			err = -EINVAL;
			return err;
		}

		err = of_property_read_string(gmsl, "dst-csi-port", &str_value);
		if (err < 0) {
			dev_err(dev, "No dst-csi-port found\n");
			return err;
		}
		sensor->g_ctx.dst_csi_port =
			(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

		err = of_property_read_string(gmsl, "src-csi-port", &str_value);
		if (err < 0) {
			dev_err(dev, "No src-csi-port found\n");
			return err;
		}
		sensor->g_ctx.src_csi_port =
			(!strcmp(str_value, "a")) ? GMSL_CSI_PORT_A : GMSL_CSI_PORT_B;

		err = of_property_read_string(gmsl, "csi-mode", &str_value);
		if (err < 0) {
			dev_err(dev, "No csi-mode found\n");
			return err;
		}

		if (!strcmp(str_value, "1x4")) {
			sensor->g_ctx.csi_mode = GMSL_CSI_1X4_MODE;
		} else if (!strcmp(str_value, "2x4")) {
			sensor->g_ctx.csi_mode = GMSL_CSI_2X4_MODE;
		} else if (!strcmp(str_value, "2x2")) {
			sensor->g_ctx.csi_mode = GMSL_CSI_2X2_MODE;
		} else {
			dev_err(dev, "invalid csi mode\n");
			return err;
		}

		err = of_property_read_string(gmsl, "serdes-csi-link", &str_value);
		if (err < 0) {
			dev_err(dev, "No serdes-csi-link found\n");
			return err;
		}
		sensor->g_ctx.serdes_csi_link =
			(!strcmp(str_value, "a")) ?
				GMSL_SERDES_CSI_LINK_A : GMSL_SERDES_CSI_LINK_B;

		err = of_property_read_u32(gmsl, "st-vc", &value);
		if (err < 0) {
			dev_err(dev, "No st-vc info\n");
			return err;
		}
		sensor->g_ctx.st_vc = value;

		err = of_property_read_u32(gmsl, "vc-id", &value);
		if (err < 0) {
			dev_err(dev, "No vc-id info\n");
			return err;
		}
		sensor->g_ctx.dst_vc = value;

		err = of_property_read_u32(gmsl, "num-lanes", &value);
		if (err < 0) {
			dev_err(dev, "No num-lanes info\n");
			return err;
		}
		sensor->g_ctx.num_csi_lanes = value;

		sensor->g_ctx.num_streams =
				of_property_count_strings(gmsl, "streams");
		if (sensor->g_ctx.num_streams <= 0) {
			dev_err(dev, "No streams found\n");
			err = -EINVAL;
			return err;
		}

		for (i = 0; i < sensor->g_ctx.num_streams; i++) {
			of_property_read_string_index(gmsl, "streams", i,
							&str_value1[i]);
			if (!str_value1[i]) {
				dev_err(dev, "invalid stream info\n");
				return err;
			}
			if (!strcmp(str_value1[i], "raw12")) {
				sensor->g_ctx.streams[i].st_data_type =
								GMSL_CSI_DT_RAW_12;
			} else if (!strcmp(str_value1[i], "embed")) {
				sensor->g_ctx.streams[i].st_data_type =
								GMSL_CSI_DT_EMBED;
			} else if (!strcmp(str_value1[i], "ued-u1")) {
				sensor->g_ctx.streams[i].st_data_type =
								GMSL_CSI_DT_UED_U1;
			} else {
				dev_err(dev, "invalid stream data type\n");
				return err;
			}
		}

		sensor->g_ctx.s_dev = dev;

		//TODO:?mutex_init(&serdes_lock__);
		/* Pair sensor to serializer dev */
		err = max96793_sdev_pair(sensor->ser_dev, &sensor->g_ctx);
		if (err) {
			dev_err(dev, "gmsl ser pairing failed\n");
			return err;
		}

		/* Register sensor to deserializer dev */
		err = max96792_sdev_register(sensor->dser_dev, &sensor->g_ctx);
		if (err) {
			dev_err(dev, "gmsl deserializer register failed\n");
			return err;
		}

		/*
		 * gmsl serdes setup
		 *
		 * Sensor power on/off should be the right place for serdes
		 * setup/reset. But the problem is, the total required delay
		 * in serdes setup/reset exceeds the frame wait timeout, looks to
		 * be related to multiple channel open and close sequence
		 * issue (#BUG 200477330).
		 * Once this bug is fixed, these may be moved to power on/off.
		 * The delays in serdes is as per guidelines and can't be reduced,
		 * so it is placed in probe/remove, though for that, deserializer
		 * would be powered on always post boot, until 1.2v is supplied
		 * to deserializer from CVB.
		 */

		err = imx900_gmsl_serdes_setup(sensor);
		if (err) {
			dev_err(dev, "%s gmsl serdes setup failed\n", __func__);
			return err;
		}
	}

	retval = imx900_power_on(sensor);
	if (retval < 0) {
		dev_err(dev, "%s: sensor power on fail\n", __func__);
		goto probe_err_power_off;
	}

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &imx900_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.ops = &imx900_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[IMX900_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity,
				IMX900_SENS_PADS_NUM,
				sensor->pads);
	if (retval < 0)
		goto probe_err_power_off;

	memcpy(&sensor->cur_mode, &pimx900_mode_info[0],
		sizeof(struct vvcam_mode_info_s));

	/* initialize controls */
	retval = v4l2_ctrl_handler_init(&sensor->ctrls.handler, 7);
	if (retval < 0) {
		dev_err(&client->dev,
			"%s : ctrl handler init Failed\n", __func__);
		goto probe_err_power_off;
	}

	sensor->ctrls.handler.lock = &sensor->lock;

	// add new controls
	sensor->ctrls.exposure = v4l2_ctrl_new_std(&sensor->ctrls.handler, &imx900_ctrl_ops, V4L2_CID_EXPOSURE,
					3, 30000, 1, 1000);
	sensor->ctrls.gain = v4l2_ctrl_new_std(&sensor->ctrls.handler, &imx900_ctrl_ops, V4L2_CID_GAIN,
					0, 480, 1, 0);
	
	if (sensor->cur_mode.bit_width == 12) {
		default_black_level = IMX900_DEFAULT_BLACK_LEVEL_12BPP;
	}
	else {
		default_black_level = IMX900_DEFAULT_BLACK_LEVEL_10BPP;
	}

	sensor->ctrls.black_level = v4l2_ctrl_new_std(&sensor->ctrls.handler, &imx900_ctrl_ops, V4L2_CID_BLACK_LEVEL,
					0, IMX900_MAX_BLACK_LEVEL, 1, default_black_level);
	sensor->ctrls.data_rate = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx900_ctrl_data_rate, NULL);
	//sensor->ctrls.sync_mode = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx900_ctrl_sync_mode, NULL);
	sensor->ctrls.framerate = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx900_ctrl_framerate, NULL);
	sensor->ctrls.shutter_mode = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx900_ctrl_shutter_mode, NULL);
	sensor->ctrls.test_pattern = v4l2_ctrl_new_std_menu_items(&sensor->ctrls.handler, &imx900_ctrl_ops, V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1, 0, 0, test_pattern_menu);

	sensor->sd.ctrl_handler = &sensor->ctrls.handler;
	if (sensor->ctrls.handler.error) {
		retval = sensor->ctrls.handler.error;
		goto free_ctrls;
	}

	// setup default controls
	retval = v4l2_ctrl_handler_setup(&sensor->ctrls.handler);
	if (retval) {
		dev_err(&client->dev,
			"Error %d setup default controls\n", retval);
		goto free_ctrls;
	}

	retval = v4l2_async_register_subdev_sensor(sd);
	if (retval < 0) {
		dev_err(&client->dev, "%s--Async register failed, ret=%d\n",
			__func__, retval);
		goto probe_err_free_entiny;
	}

	pr_debug("%s camera mipi imx900, is found\n", __func__);

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);

probe_err_free_entiny:
	media_entity_cleanup(&sd->entity);

probe_err_power_off:
	imx900_power_off(sensor);

	return retval;
}

static void imx900_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx900 *sensor = client_to_imx900(client);
	int err = 0;

	pr_debug("enter %s function\n", __func__);

	err = imx900_write_reg(sensor, SYNCSEL, 0xF0);
	if (err < 0)
		pr_warn("%s: failed to set XVS XHS to Hi-Z\n", __func__);

	if (!(strcmp(sensor->gmsl, "gmsl"))) {
		imx900_gmsl_serdes_reset(sensor);
		max96792_sdev_unregister(sensor->dser_dev, &sensor->i2c_client->dev);
		max96793_sdev_unpair(sensor->ser_dev, &sensor->i2c_client->dev);
	}

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx900_power_off(sensor);
	mutex_destroy(&sensor->lock);
}

static int __maybe_unused imx900_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx900 *sensor = client_to_imx900(client);

	sensor->resume_status = sensor->stream_status;
	if (sensor->resume_status)
		imx900_s_stream(&sensor->sd, 0);

	return 0;
}

static int __maybe_unused imx900_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx900 *sensor = client_to_imx900(client);

	if (sensor->resume_status)
		imx900_s_stream(&sensor->sd, 1);

	return 0;
}

static const struct dev_pm_ops imx900_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx900_suspend, imx900_resume)
};

static const struct i2c_device_id imx900_id[] = {
	{"imx900", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, imx900_id);

static struct i2c_driver imx900_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "imx900",
		.pm = &imx900_pm_ops,
		.of_match_table	= imx900_of_match,
	},
	.probe  = imx900_probe,
	.remove = imx900_remove,
	.id_table = imx900_id,
};

module_i2c_driver(imx900_i2c_driver);
MODULE_DESCRIPTION("IMX900 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
