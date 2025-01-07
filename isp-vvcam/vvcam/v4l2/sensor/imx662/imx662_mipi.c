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

//#define DEBUG 1
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

#include "vvsensor.h"
#include "imx662_regs.h"
#include "max96792.h"
#include "max96793.h"

#define IMX662_MAX_RETRIES               10

#define IMX662_SENS_PAD_SOURCE           0
#define IMX662_SENS_PADS_NUM             1

#define IMX662_K_FACTOR                  1000LL
#define IMX662_M_FACTOR                  1000000LL
#define IMX662_G_FACTOR                  1000000000LL
#define IMX662_T_FACTOR                  1000000000000LL

#define IMX662_MAX_GAIN_DEC              240
#define IMX662_MAX_GAIN_DB               72

#define IMX662_MAX_BLACK_LEVEL_10BPP     1023
#define IMX662_MAX_BLACK_LEVEL_12BPP     4095

#define IMX662_DEFAULT_BLACK_LEVEL_10BPP 50
#define IMX662_DEFAULT_BLACK_LEVEL_12BPP 200

#define IMX662_MIN_SHR0_LENGTH           4
#define IMX662_MIN_SHR0_CLEAR_LENGTH     8

#define IMX662_MIN_SHR0_RHS1_DIST        5
#define IMX662_MIN_SHR1_LENGTH           5
#define IMX662_MIN_INTEGRATION_LINES     1

//official NXP condition
#define IMX662_MAX_VS_INTEGRATION_LINES  66
#define IMX662_MIN_VS_INTEGRATION_LINES  2

#define IMX662_BRL                       1120

#define IMX662_MAX_BOUNDS_WIDTH          1936
#define IMX662_MAX_BOUNDS_HEIGHT         1250

#define IMX662_TWO_LANE_MODE             1
#define IMX662_FOUR_LANE_MODE            3
#define IMX662_BINNING_MODE              1

#define IMX662_INCK                      74250000LL
/*
 * Line time = hmax / IMX662_INCK * 10**6
 */
#define IMX662_LINE_TIME_H990            13333 // in ns
#define IMX662_LINE_TIME_H660            8904  //in ns

#define V4L2_CID_DATA_RATE              (V4L2_CID_USER_IMX_BASE + 1)
#define V4L2_CID_SYNC_MODE              (V4L2_CID_USER_IMX_BASE + 2)
#define V4L2_CID_FRAME_RATE             (V4L2_CID_USER_IMX_BASE + 3)
#define V4L2_CID_VS_EXP                 (V4L2_CID_USER_IMX_BASE + 4)
#define V4L2_CID_VS_GAIN                (V4L2_CID_USER_IMX_BASE + 5)
#define V4L2_CID_EXP_GAIN               (V4L2_CID_USER_IMX_BASE + 6)
#define V4L2_NUM_CTRLS                  10

enum mode_index {
	IMX662_ALL_PIXEL_INDEX,
	IMX662_CROP_INDEX,
	IMX662_BINNING_INDEX,
	IMX662_BINNING_CROP_INDEX,
	IMX662_DOL_INDEX,
	IMX662_CLEAR_INDEX,
	IMX662_MAX_INDEX
};

static const struct of_device_id imx662_of_match[] = {
	{ .compatible = "framos,imx662" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx662_of_match);

enum data_rate_mode {
	IMX662_2376_MBPS,
	IMX662_2079_MBPS,
	IMX662_1782_MBPS,
	IMX662_1440_MBPS,
	IMX662_1188_MBPS,
	IMX662_891_MBPS,
	IMX662_720_MBPS,
	IMX662_594_MBPS,
};

static const char * const imx662_test_pattern_menu[] = {
	[0] = "No pattern",
	[1] = "000h Pattern",
	[2] = "3FF(FFFh) Pattern",
	[3] = "155(555h) Pattern",
	[4] = "2AA(AAAh) Pattern",
	[5] = "555/AAAh Pattern",
	[6] = "AAA/555h Pattern",
	[7] = "000/555h Pattern",
	[8] = "555/000h Pattern",
	[9] = "000/FFFh Pattern",
	[10] = "FFF/000h Pattern",
	[11] = "H Color-bar",
	[12] = "V Color-bar",
};

/*
 * Tranformation matrix from gain times used by isp to gain registers used
 * by Sony sensors by formula gain_time = 10**(gain_db / 20) * 1024
 * the resulting value is in range (0-240)
 */
#define IMX662_GAIN_REG_LEN 241
static const u32 gain_reg2times[IMX662_GAIN_REG_LEN] = {
	1024, 1060, 1097, 1136, 1176, 1217, 1260, 1304, 1350, 1397, 1446, 1497,
	1550, 1604, 1661, 1719, 1780, 1842, 1907, 1974, 2043, 2115, 2189, 2266,
	2346, 2428, 2514, 2602, 2693, 2788, 2886, 2987, 3092, 3201, 3314, 3430,
	3551, 3675, 3805, 3938, 4077, 4220, 4368, 4522, 4681, 4845, 5015, 5192,
	5374, 5563, 5758, 5961, 6170, 6387, 6611, 6844, 7084, 7333, 7591, 7858,
	8134, 8420, 8716, 9022, 9339, 9667, 10007, 10359, 10723, 11099, 11489,
	11893, 12311, 12744, 13192, 13655, 14135, 14632, 15146, 15678, 16229,
	16800, 17390, 18001, 18634, 19289, 19966, 20668, 21394, 22146, 22925,
	23730, 24564, 25427, 26321, 27246, 28203, 29194, 30220, 31282, 32382,
	33520, 34698, 35917, 37179, 38486, 39838, 41238, 42687, 44188, 45740,
	47348, 49012, 50734, 52517, 54363, 56273, 58251, 60298, 62417, 64610,
	66881, 69231, 71664, 74182, 76789, 79488, 82281, 85173, 88166, 91264,
	94471, 97791, 101228, 104785, 108468, 112279, 116225, 120310, 124537,
	128914, 133444, 138134, 142988, 148013, 153215, 158599, 164172, 169942,
	175914, 182096, 188495, 195119, 201976, 209074, 216421, 224027, 231900,
	240049, 248485, 257217, 266256, 275613, 285299, 295325, 305703, 316446,
	327567, 339078, 350994, 363329, 376097, 389314, 402995, 417157, 431817,
	446992, 462700, 478961, 495793, 513216, 531251, 549921, 569246, 589250,
	609958, 631393, 653582, 676550, 700326, 724936, 750412, 776783, 804081,
	832338, 861589, 891867, 923209, 955652, 989236, 1024000, 1059986, 1097236,
	1135795, 1175709, 1217026, 1259795, 1304067, 1349895, 1397333, 1446438,
	1497269, 1549887, 1604353, 1660734, 1719095, 1779508, 1842044, 1906777,
	1973786, 2043149, 2114949, 2189273, 2266209, 2345848, 2428287, 2513622,
	2601956, 2693394, 2788046, 2886024, 2987445, 3092431, 3201105, 3313599,
	3430046, 3550585, 3675361, 3804521, 3938220, 4076617};

/*
 * Exponential gain can take values {0, 6, 12, 18, 24, 30} dbs which equals
 * gain times of {1024, 2043, 4077, 8134, 16229, 32382}
 * Bounds are derived as arithmetic mean of two boundaries {(1024 + 2043) / 2, etc.}
 */
static const u32 exp_gain_bounds[5] = {1534, 3060, 6106, 12182, 24306};

enum sync_mode {
	NO_SYNC,
	INTERNAL_SYNC,
	EXTERNAL_SYNC,
};

const char * const imx662_sync_mode_menu[] = {
	[NO_SYNC]       = "No Sync",
	[INTERNAL_SYNC] = "Internal sync",
	[EXTERNAL_SYNC] = "External sync",
};

static const struct v4l2_ctrl_ops imx662_ctrl_ops;

static struct v4l2_ctrl_config imx662_ctrl_data_rate[] = {
	{
		.ops = &imx662_ctrl_ops,
		.id = V4L2_CID_DATA_RATE,
		.name = "Data rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = IMX662_2376_MBPS,
		.max = IMX662_594_MBPS,
		.def = IMX662_594_MBPS,
		.step = 1,
	},
};

static struct v4l2_ctrl_config imx662_ctrl_sync_mode[] = {
	{
		.ops = &imx662_ctrl_ops,
		.id = V4L2_CID_SYNC_MODE,
		.name = "Sync mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = NO_SYNC,
		.max = EXTERNAL_SYNC,
		.def = NO_SYNC,
		.step = 1,
	},
};

static struct v4l2_ctrl_config imx662_ctrl_framerate[] = {
	{
		.ops = &imx662_ctrl_ops,
		.id = V4L2_CID_FRAME_RATE,
		.name = "Frame rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 90,
		.def = 60,
		.step = 1,
	},
};

static struct v4l2_ctrl_config imx662_ctrl_vs_exp[] = {
	{
		.ops = &imx662_ctrl_ops,
		.id = V4L2_CID_VS_EXP,
		.name = "VS exposure",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 10000,
		.def = 100,
		.step = 1,
	},
};

static struct v4l2_ctrl_config imx662_ctrl_vs_gain[] = {
	{
		.ops = &imx662_ctrl_ops,
		.id = V4L2_CID_VS_GAIN,
		.name = "VS gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 720,
		.def = 0,
		.step = 1,
	},
};

static struct v4l2_ctrl_config imx662_ctrl_exp_gain[] = {
	{
		.ops = &imx662_ctrl_ops,
		.id = V4L2_CID_EXP_GAIN,
		.name = "Exponential gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 5,
		.def = 2,
		.step = 1,
	},
};

struct imx662_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *framerate;
	struct v4l2_ctrl *black_level;
	struct v4l2_ctrl *data_rate;
	struct v4l2_ctrl *sync_mode;
	struct v4l2_ctrl *vs_exp;
	struct v4l2_ctrl *vs_gain;
	struct v4l2_ctrl *exp_gain;
};

struct imx662 {
	struct i2c_client *i2c_client;
	unsigned int rst_gpio;
	unsigned int csi_id;
	unsigned int powered_on;

	struct v4l2_subdev sd;
	struct media_pad pads[IMX662_SENS_PADS_NUM];

	struct v4l2_mbus_framefmt format;
	vvcam_mode_info_t cur_mode;
	struct mutex lock;
	u32 stream_status;
	u32 resume_status;
	struct imx662_ctrls ctrls;
	const char *gmsl;
	struct device *ser_dev;
	struct device *dser_dev;
	struct gmsl_link_ctx g_ctx;
};

#define client_to_imx662(client)\
	container_of(i2c_get_clientdata(client), struct imx662, sd)

static inline struct imx662 *to_imx662_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct imx662, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct imx662,
				ctrls.handler)->sd;
}

static struct vvcam_mode_info_s pimx662_mode_info[] = {
	{
		.index          = IMX662_ALL_PIXEL_INDEX,
		.size           = {
			.bounds_width  = IMX662_DEFAULT_WIDTH,
			.bounds_height = IMX662_DEFAULT_HEIGHT,
			.top           = 12,
			.left          = 8,
			.width         = 1920,
			.height        = 1080
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = IMX662_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines    = IMX662_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX662_LINE_TIME_H990,

			.max_integration_line  = IMX662_MAX_BOUNDS_HEIGHT - 4,
			.min_integration_line  = IMX662_MIN_INTEGRATION_LINES,

			.max_again             = 32382,    // 30 db
			.min_again             = 1 * 1024, // 0 db
			.max_dgain             = 4044235,  // 42 db
			.min_dgain             = 1 * 1024, // 0 db
			.gain_step             = 36,

			.start_exposure        = 3000 * 1024, // 3000 * 1024,
			.cur_fps               = 30 * 1024,
			.max_fps               = 60 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx662_init_setting,
		.reg_data_count = ARRAY_SIZE(imx662_init_setting),
	},
	{
		.index          = IMX662_CROP_INDEX,
		.size           = {
			.bounds_width  = 1296,
			.bounds_height = 740,
			.top           = 12,
			.left          = 8,
			.width         = 1280,
			.height        = 720,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = IMX662_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines    = IMX662_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX662_LINE_TIME_H990,

			.max_integration_line  = IMX662_MAX_BOUNDS_HEIGHT - 4,
			.min_integration_line  = IMX662_MIN_INTEGRATION_LINES,

			.max_again             = 32382,    // 30 db
			.min_again             = 1 * 1024, // 0 db
			.max_dgain             = 4044235,  // 42 db
			.min_dgain             = 1 * 1024, // 0 db
			.gain_step             = 36,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 60 * 1024,
			.max_fps               = 60 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx662_init_setting,
		.reg_data_count = ARRAY_SIZE(imx662_init_setting),
	},
	{
		.index          = IMX662_BINNING_INDEX,
		.size           = {
			.bounds_width  = IMX662_BINNING_WIDTH,
			.bounds_height = IMX662_BINNING_HEIGHT,
			.top           = 6,
			.left          = 4,
			.width         = 960,
			.height        = 540,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = IMX662_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines    = IMX662_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX662_LINE_TIME_H660,

			.max_integration_line  = IMX662_MAX_BOUNDS_HEIGHT - 4,
			.min_integration_line  = IMX662_MIN_INTEGRATION_LINES,

			.max_again             = 32382,    // 30 db
			.min_again             = 1 * 1024, // 0 db
			.max_dgain             = 4044235,  // 42 db
			.min_dgain             = 1 * 1024, // 0 db
			.gain_step             = 36,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 90 * 1024,
			.max_fps               = 90 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,

			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx662_init_setting,
		.reg_data_count = ARRAY_SIZE(imx662_init_setting),
	},
	{
		.index          = IMX662_BINNING_CROP_INDEX,
		.size           = {
			.bounds_width  = 648,
			.bounds_height = 490,
			.top           = 6,
			.left          = 4,
			.width         = 640,
			.height        = 480,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = IMX662_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines    = IMX662_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX662_LINE_TIME_H660,

			.max_integration_line  = IMX662_MAX_BOUNDS_HEIGHT - 4,
			.min_integration_line  = IMX662_MIN_INTEGRATION_LINES,

			.max_again             = 32382,    // 30 db
			.min_again             = 1 * 1024, // 0 db
			.max_dgain             = 4044235,  // 42 db
			.min_dgain             = 1 * 1024, // 0 db
			.gain_step             = 36,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 60 * 1024,
			.max_fps               = 100 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx662_init_setting,
		.reg_data_count = ARRAY_SIZE(imx662_init_setting),
	},
	{
		.index          = IMX662_DOL_INDEX,
		/* Return to all pixel size settings once NXP updates software */
		.size           = {
			.bounds_width  = IMX662_DEFAULT_WIDTH,
			.bounds_height = IMX662_DEFAULT_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX662_DEFAULT_WIDTH,
			.height        = IMX662_DEFAULT_HEIGHT,
		},

		.hdr_mode       = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_L_AND_S,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},

		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = IMX662_MAX_BOUNDS_HEIGHT,
			.curr_frm_len_lines    = IMX662_MAX_BOUNDS_HEIGHT,
			.one_line_exp_time_ns  = IMX662_LINE_TIME_H990,

			.max_vsintegration_line = IMX662_MAX_VS_INTEGRATION_LINES,
			.min_vsintegration_line = IMX662_MIN_VS_INTEGRATION_LINES,

			// reduce max integration line for short exposure to allow vs exposure
			.max_integration_line  = 2 * IMX662_MAX_BOUNDS_HEIGHT - IMX662_MAX_VS_INTEGRATION_LINES,
			.min_integration_line  = IMX662_MIN_INTEGRATION_LINES,

			.max_again             = 32382,    // 30 db
			.min_again             = 1 * 1024, // 0 db
			.max_dgain             = 4044235,  // 42 db
			.min_dgain             = 1 * 1024, // 0 db

			.max_short_again       = 32382,    // 30 db
			.min_short_again       = 1 * 1024, // 0 db
			.max_short_dgain       = 4044235,  // 42 db,
			.min_short_dgain       = 1 * 1024, // 0 db,

			// not used at the moment, read from xml
			.hdr_ratio = {
				.ratio_s_vs = 8 * 1024,  // was 8
				//.ratio_l_s = 0 * 1024,   // was 0
				.accuracy = 1024,
			},
			.start_exposure        = 1000 * 1024,
			.cur_fps               = 30 * 1024,
			.max_fps               = 30 * 1024,
			.min_fps               = 1 * 1024,
			.min_afps              = 1 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx662_init_setting,
		.reg_data_count = ARRAY_SIZE(imx662_init_setting),
	},
	{
		.index          = IMX662_CLEAR_INDEX,
		/* Return to all pixel size settings once NXP updates software */
		.size           = {
			.bounds_width  = IMX662_DEFAULT_WIDTH,
			.bounds_height = IMX662_DEFAULT_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = IMX662_DEFAULT_WIDTH,
			.height        = IMX662_DEFAULT_HEIGHT,
		},
		.hdr_mode       = SENSOR_MODE_HDR_STITCH,
		.stitching_mode = SENSOR_STITCHING_DUAL_DCG_NOWAIT,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_RGGB,
		.ae_info = {
			.def_frm_len_lines     = IMX662_MAX_BOUNDS_HEIGHT * 2,
			.curr_frm_len_lines    = IMX662_MAX_BOUNDS_HEIGHT * 2,
			.one_line_exp_time_ns  = IMX662_LINE_TIME_H990,

			.max_integration_line  = IMX662_MAX_BOUNDS_HEIGHT * 2 - 4,
			.min_integration_line  = 8,

			.max_again             = 16229,    // 24 db
			.min_again             = 1 * 1024, // 0 db
			.max_dgain             = 1 * 1024, // 0 db
			.min_dgain             = 1 * 1024, // 0 db
			.gain_step             = 36,

			.max_long_again        = 1 * 1024, // 0 db
			.min_long_again        = 1 * 1024, // 0 db
			.max_long_dgain        = 32382,    // 30 db
			.min_long_dgain        = 1 * 1024, // 0 db

			.hdr_ratio = {
				//.ratio_l_s = 8 * 1024,
				.ratio_s_vs = 8 * 1024,
				.accuracy = 1024,
			},

			.start_exposure        = 1000 * 1024,
			.cur_fps               = 30 * 1024,
			.max_fps               = 30 * 1024,
			.min_fps               = 5 * 1024,
			.min_afps              = 5 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = imx662_init_setting,
		.reg_data_count = ARRAY_SIZE(imx662_init_setting),
	},
};

static int imx662_write_reg(struct imx662 *sensor, u16 reg, u8 val)
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
	 * Retry sending a message for IMX662_MAX_RETRIES and report a problem.
	 */
	for (num_retry = 0; num_retry < IMX662_MAX_RETRIES; num_retry++) {
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

static int imx662_read_reg(struct imx662 *sensor, u16 reg, u8 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8RegBuf[2] = { 0 };
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (i2c_master_send(sensor->i2c_client, au8RegBuf, 2) != 2) {
		dev_err(dev, "Read reg error: reg=%x\n", reg);
		return -1;
	}

	if (i2c_master_recv(sensor->i2c_client, &u8RdVal, 1) != 1) {
		dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return 0;
}

/*
 * i2c communication occasionally fails with sensor sending a NACK without a clear reason.
 * Retry sending a message for IMX662_MAX_RETRIES and report a problem.
 */
static int imx662_i2c_transfer(const struct i2c_client *const i2c_client,
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

	for (num_retry = 0; num_retry < IMX662_MAX_RETRIES; num_retry++) {
		ret = i2c_transfer(i2c_client->adapter, &msg, 1);
		if (ret >= 0)
			break;
	}

	if (ret < 0) {
		pr_err("%s:i2c transfer error address= %d, error=%d\n",
		__func__, msg.addr, ret);
		return ret;
	}

	if (num_retry > 0) {
		u32 error_addr = (u32)(send_buf[0] << 8) + send_buf[1];

		pr_warn("%s: i2c communication passed after %d retries: reg=%x\n",
		__func__, num_retry, error_addr);
	}

	return 0;
}

static int imx662_write_reg_arry(struct imx662 *sensor,
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
		// checkpatch: ignore
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
			ret = imx662_i2c_transfer(i2c_client, send_buf, send_buf_len);
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
		ret = imx662_i2c_transfer(i2c_client, send_buf, send_buf_len);
		if (ret < 0) {
			kfree(send_buf);
			return ret;
		}
	}

	kfree(send_buf);
	return ret;
}

static int imx662_power_on(struct imx662 *sensor)
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

static int imx662_power_off(struct imx662 *sensor)
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

static int imx662_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx662 *sensor = client_to_imx662(client);
	int ret = 0;

	pr_debug("enter %s function\n", __func__);
	if (on)
		ret = imx662_power_on(sensor);
	else
		ret = imx662_power_off(sensor);

	if (ret < 0)
		return ret;
	return 0;
}

static int imx662_query_capability(struct imx662 *sensor, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strscpy((char *)pcap->driver, "imx662", sizeof(pcap->driver));
	sprintf((char *)pcap->bus_info, "csi%d", sensor->csi_id);
	if (sensor->i2c_client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}
	return 0;
}

static int imx662_query_supports(struct imx662 *sensor, void *parry)
{
	int ret = 0;
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
	uint32_t support_counts = ARRAY_SIZE(pimx662_mode_info);

	pr_debug("enter %s function\n", __func__);

	ret = copy_to_user(&psensor_mode_arry->count, &support_counts, sizeof(support_counts));
	ret |= copy_to_user(&psensor_mode_arry->modes, pimx662_mode_info,
			   sizeof(pimx662_mode_info));
	if (ret != 0) {
		pr_err("enter %s failed to allocate memory\n", __func__);
		ret = -ENOMEM;
	}
	return ret;
}

static int imx662_get_sensor_id(struct imx662 *sensor, void *pchip_id)
{
	int ret = 0;
	u16 chip_id = 662;

	ret = copy_to_user(pchip_id, &chip_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	pr_debug("enter %s function\n", __func__);
	return ret;
}

static int imx662_get_reserve_id(struct imx662 *sensor, void *preserve_id)
{
	int ret = 0;
	u16 reserve_id = 662;

	ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	pr_debug("enter %s function\n", __func__);
	return ret;
}

static int imx662_get_sensor_mode(struct imx662 *sensor, void *pmode)
{
	int ret = 0;

	pr_debug("enter %s function\n", __func__);
	ret = copy_to_user(pmode, &sensor->cur_mode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0) {
		pr_warn("error getting sensor mode %s\n", __func__);
		ret = -ENOMEM;
	}
	pr_debug("enter %s function\n", __func__);
	return ret;
}

static int imx662_set_sensor_mode(struct imx662 *sensor, void *pmode)
{
	int ret = 0;
	int i = 0;
	struct vvcam_mode_info_s sensor_mode;

	pr_debug("enter %s function\n", __func__);
	ret = copy_from_user(&sensor_mode, pmode, sizeof(struct vvcam_mode_info_s));
	if (ret != 0) {
		pr_err("enter %s: Failed to get sensor mode\n", __func__);
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(pimx662_mode_info); i++) {
		if (pimx662_mode_info[i].index == sensor_mode.index) {
			memcpy(&sensor->cur_mode, &pimx662_mode_info[i],
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
static int imx662_adjust_hmax_register(struct imx662 *sensor)
{

	int ret = 0;
	u32 hmax = 990;
	u8 data_rate = 255;
	u8 binning_mode = 255;

	pr_debug("%s:++\n", __func__);

	ret = imx662_read_reg(sensor, DATARATE_SEL, &data_rate);
	if (ret < 0) {
		pr_err("%s: Failed to read data rate.\n", __func__);
		return ret;
	}

	ret = imx662_read_reg(sensor, ADDMODE, &binning_mode);
	if (ret < 0) {
		pr_err("%s: Failed to read binning mode\n", __func__);
		return ret;
	}

	if (data_rate == IMX662_720_MBPS && ~binning_mode) {
		hmax = 660;
	}
	else if (data_rate == IMX662_594_MBPS) {
		hmax = binning_mode ? 660 : 990;
	} else {
		pr_err("%s: Invalid data rate %u and binning %d combination.\n", __func__, data_rate, binning_mode);
		return -1;
	}

	ret = imx662_write_reg(sensor, REGHOLD, 1);
	ret |= imx662_write_reg(sensor, HMAX_HIGH, (hmax >> 8) & 0xff);
	ret |= imx662_write_reg(sensor, HMAX_LOW, hmax & 0xff);
	ret |= imx662_write_reg(sensor, REGHOLD, 0);
	if (ret) {
		pr_err("%s: failed to set HMAX register\n", __func__);
		return ret;
	}

	sensor->cur_mode.ae_info.one_line_exp_time_ns = (u32) ((hmax * IMX662_G_FACTOR) / IMX662_INCK);
	pr_debug("%s:  one line : %u\n", __func__, sensor->cur_mode.ae_info.one_line_exp_time_ns);
	pr_debug("%s:  HMAX: %u\n", __func__, hmax);

	return 0;
}

static int imx662_change_data_rate(struct imx662 *sensor, u8 data_rate)
{
	int ret = 0;
	u8 current_lane_mode, current_binning_mode;

	pr_debug("%s++\n", __func__);

	ret = imx662_read_reg(sensor, LANEMODE, &current_lane_mode);
	if (ret < 0) {
		pr_err("%s: Could not read lane mode\n", __func__);
		return ret;
	}

	if (current_lane_mode == IMX662_TWO_LANE_MODE) {
		pr_warn("%s: 2 lane mode is not supported, switching to 4 lane mode\n", __func__);
		ret = imx662_write_reg(sensor, LANEMODE, IMX662_FOUR_LANE_MODE);
		if (ret < 0) {
			pr_err("%s: Could not set to 4 lane mode\n", __func__);
			return ret;
			}
	}

	ret = imx662_read_reg(sensor, ADDMODE, &current_binning_mode);
	if (ret < 0) {
		pr_err("%s: Could not read ADDMODE\n", __func__);
		return ret;
	}

	if (current_binning_mode == IMX662_BINNING_MODE) {
		if (data_rate != IMX662_594_MBPS) {
			pr_warn("%s: Selected data rate is not supported in 4 lane binning mode, switching to 594 data rate!\n", __func__);
			data_rate = IMX662_594_MBPS;
			goto change_datarate;
		}
	} else { // non binning mode
		if ((data_rate != IMX662_720_MBPS) && (data_rate != IMX662_594_MBPS)) {
			pr_warn("%s: Selected data rate is not supported in 4 lane non binning mode, switching to 594 data rate!\n", __func__);
			data_rate = IMX662_594_MBPS;
			goto change_datarate;
		}
	}
	pr_debug("%s: Setting data rate to value: %u\n", __func__, data_rate);
	ret = imx662_write_reg(sensor, DATARATE_SEL, data_rate);

	if (ret < 0) {
		pr_err("%s: Could not set data rate\n", __func__);
		return ret;
	}
	return 0;

change_datarate:
	ret = imx662_write_reg(sensor, DATARATE_SEL, data_rate);
	sensor->ctrls.data_rate->val = data_rate;
	sensor->ctrls.data_rate->cur.val = data_rate;
	return ret;
}

static int imx662_set_data_rate(struct imx662 *sensor, u8 data_rate)
{
	int ret = 0;

	pr_debug("enter %s data rate received: %u\n", __func__, data_rate);
	ret = imx662_change_data_rate(sensor, data_rate);
	if (ret < 0) {
		pr_err("%s: unable to set data rate\n", __func__);
		return ret;
	}

	ret = imx662_adjust_hmax_register(sensor);
	if (ret < 0) {
		pr_err("%s: unable to adjust hmax\n", __func__);
		return ret;
	}
	return ret;
}

/**
 * XVS & XHS are synchronizing/triggering pins
 * This sensor supports - Internal and External synchronization in master mode
 *                      - External synchronization in slave mode
 *       XVS     XHS
 * 0x0 - output, output
 * 0x3 - hi-z,   output
 * 0xC - output, hi-z
 * 0xF - hi-z,   hi-z
 */
static int imx662_configure_triggering_pins(struct imx662 *sensor)
{
	int err = 0;
	u8 extmode;
	u8 xvs_xhs_drv = 0xF;

	pr_debug("enter %s function\n", __func__);
	err = imx662_read_reg(sensor, EXTMODE, &extmode);

	if (extmode == INTERNAL_SYNC) {
		/* XVS - output, XHS - output */
		xvs_xhs_drv = 0x0;
		pr_debug("%s: Sensor is in - Internal sync Master mode\n", __func__);
	} else if (extmode == EXTERNAL_SYNC) {
		/* XVS - hi-z, XHS - output */
		xvs_xhs_drv = 0x3;
		pr_debug("%s: Sensor is in - External sync Master mode\n", __func__);
	} else {
		/* XVS - hi-z, XHS - hi-z */
		xvs_xhs_drv = 0xF;
		pr_debug("%s: Sensor is in - No sync Master mode\n", __func__);
	}

	err = imx662_write_reg(sensor, XVS_DRV_XHS_DRV, xvs_xhs_drv);
	if (err) {
		pr_err("%s: error configuring Triggering pins\n", __func__);
		return err;
	}

	pr_debug("%s: XVS_XHS driver register: %x\n", __func__, xvs_xhs_drv);

	return 0;
}

/**
 * Synchronization mode is for Master mode
 * Sensor can be synchronized Externaly and Internaly in Master mode
 */
static int imx662_set_sync_mode(struct imx662 *sensor, u32 val)
{
	int err = 0;
	u8 extmode = 0;

	pr_debug("enter %s sync mode %u\n", __func__, val);

	if (val == EXTERNAL_SYNC)
		extmode = 1;
	else
		extmode = 0;

	if (sensor->powered_on == 1) {
		err = imx662_write_reg(sensor, EXTMODE, extmode);
		if (err < 0) {
			pr_err("%s: error setting sync mode\n", __func__);
			return err;
		}
	}

	err = imx662_configure_triggering_pins(sensor);
	if (err < 0) {
		pr_err("%s:unable to configure XVS/XHS pins\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int imx662_set_exp(struct imx662 *sensor, u32 exp, u8 which_control)
{
	int ret = 0;
	u32 integration_time_line;
	u32 reg_shr0 = 0;
	u32 min_shr0 = IMX662_MIN_SHR0_LENGTH;
	u32 frame_length;

	pr_debug("enter %s exposure received: %u control: %u\n", __func__, exp, which_control);

	frame_length = sensor->cur_mode.ae_info.curr_frm_len_lines;

	if (which_control == 0) { // from ISP driver
		integration_time_line = ((exp >> 10)
		   * IMX662_K_FACTOR) / sensor->cur_mode.ae_info.one_line_exp_time_ns;
	} else {  // from V4L2 control
		integration_time_line = (exp * IMX662_K_FACTOR) / sensor->cur_mode.ae_info.one_line_exp_time_ns;
	}

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

	if (sensor->cur_mode.index == IMX662_DOL_INDEX) {
		reg_shr0 = 2 * frame_length - integration_time_line;
		// must be even in dol case
		reg_shr0 = (reg_shr0 % 2) ? reg_shr0 - 1 : reg_shr0;
		// this should never happen - keep as sanity check
		if (reg_shr0 > (2 * frame_length - IMX662_MIN_INTEGRATION_LINES)) {
			pr_err("%s reg_shr0 too large: %u\n", __func__, reg_shr0);
			reg_shr0 = 2 * frame_length - IMX662_MIN_INTEGRATION_LINES;
			pr_err("%s setting reg_shr0 to : %u\n", __func__, reg_shr0);
		}
	} else {
		reg_shr0 = frame_length - integration_time_line;
		// this should never happen - keep as sanity check
		if (reg_shr0 > (frame_length - IMX662_MIN_INTEGRATION_LINES)) {
			pr_err("%s reg_shr0 too large: %u\n", __func__, reg_shr0);
			reg_shr0 = frame_length - IMX662_MIN_INTEGRATION_LINES;
			pr_err("%s setting reg_shr0 to : %u\n", __func__, reg_shr0);
		}
	}

	// Sanity check
	switch (sensor->cur_mode.index) {
	case IMX662_CLEAR_INDEX:
		min_shr0 = IMX662_MIN_SHR0_CLEAR_LENGTH;
		break;
	default:
		min_shr0 = IMX662_MIN_SHR0_LENGTH;
		break;
	}

	reg_shr0 = max_t(u32, min_shr0, reg_shr0);

	pr_debug("%s: exposure register: %u integration_time_line: %u\n", __func__, reg_shr0, integration_time_line);
	ret = imx662_write_reg(sensor, REGHOLD, 1);
	ret |= imx662_write_reg(sensor, SHR0_HIGH, (reg_shr0 >> 16) & 0xff);
	ret |= imx662_write_reg(sensor, SHR0_MID, (reg_shr0 >> 8) & 0xff);
	ret |= imx662_write_reg(sensor, SHR0_LOW, reg_shr0 & 0xff);
	ret |= imx662_write_reg(sensor, REGHOLD, 0);

	if (ret < 0)
		pr_err("%s Failed to set exposure exp: %u, shr register:  %u\n", __func__, exp, reg_shr0);

	return ret;
}

static int imx662_get_exp_register(struct imx662 *sensor, u32 *reg_shr0)
{
	int ret = 0;
	u8 val = 0;

	ret = imx662_read_reg(sensor, SHR0_HIGH, &val);
	*reg_shr0 = val;
	ret |= imx662_read_reg(sensor, SHR0_MID, &val);
	*reg_shr0 = (*reg_shr0 << 8) + val;
	ret |= imx662_read_reg(sensor, SHR0_LOW, &val);
	*reg_shr0 = (*reg_shr0 << 8) + val;
	return ret;
}

static int imx662_set_vs_exp(struct imx662 *sensor, u32 exp, unsigned int which_control)
{
	int ret = 0;
	u32 reg_shr0 = 0;
	u32 integration_time_line;
	u32 frame_length;
	u32 reg_rhs1 = 5;
	u32 reg_shr1 = IMX662_MIN_SHR1_LENGTH;

	pr_debug("enter %s vs exposure received: %u\n", __func__, exp);

	ret = imx662_get_exp_register(sensor, &reg_shr0);
	if (ret < 0) {
		pr_err("%s Failed to read short exposure: unable to set vs exposure\n", __func__);
		return ret;
	}

	pr_debug("%s: reg_shr0 equal to: %u\n", __func__, reg_shr0);
	frame_length = sensor->cur_mode.ae_info.curr_frm_len_lines;

	if (which_control == 0) { // from ISP driver
		pr_debug("%s: vs_exposure %u\n", __func__, (exp >> 10));
		integration_time_line = ((exp >> 10)
		   * IMX662_K_FACTOR) / sensor->cur_mode.ae_info.one_line_exp_time_ns;
	} else {  // from V4L2 control
		pr_debug("%s: vs_exposure: %u\n", __func__, exp);
		integration_time_line = (exp * IMX662_K_FACTOR) / sensor->cur_mode.ae_info.one_line_exp_time_ns;
	}
	pr_debug("%s: vs integration_time_line: %u\n", __func__, integration_time_line);

	if (integration_time_line < sensor->cur_mode.ae_info.min_vsintegration_line) {
		pr_warn("%s vs integration line too small: setting to %u\n", __func__,
			sensor->cur_mode.ae_info.min_vsintegration_line);
		integration_time_line = sensor->cur_mode.ae_info.min_vsintegration_line;
	}

	if (integration_time_line > sensor->cur_mode.ae_info.max_vsintegration_line) {
		pr_warn("%s vs integration line too large: setting to %u\n", __func__,
			sensor->cur_mode.ae_info.max_vsintegration_line);
		integration_time_line = sensor->cur_mode.ae_info.max_vsintegration_line;
	}

	// set rhs1 register to maximal value by datasheet conditions
	reg_rhs1 = reg_shr0 - IMX662_MIN_SHR0_RHS1_DIST;
	reg_rhs1 = max_t(u32, 2 * IMX662_BRL - 1, reg_rhs1);

	// sanity check this should never happen
	if (reg_shr0 <= reg_rhs1) {
		pr_warn("%s Invalid values for reg_rhs1 %u, reg_shr0: %u  :\n", __func__, reg_rhs1, reg_shr0);
		reg_rhs1 = reg_shr0 - 5;
	}

	if (reg_rhs1 - reg_shr1 > integration_time_line) {
		reg_rhs1 = integration_time_line + reg_shr1;
		reg_rhs1 = (reg_rhs1 % 2) ? reg_rhs1 : reg_rhs1 - 1;
	} else {
		pr_warn(" %s: integration time for vs exposure %d too large\n", __func__, integration_time_line);
	}

	pr_debug("%s: changed vs_exposure:  register values shr1: %u rhs1: %u\n", __func__, reg_shr1, reg_rhs1);
	ret = imx662_write_reg(sensor, REGHOLD, 1);
	ret |= imx662_write_reg(sensor, SHR1_LOW, reg_shr1);
	ret |= imx662_write_reg(sensor, RHS1_LOW, reg_rhs1 & 0xff);
	ret |= imx662_write_reg(sensor, RHS1_MID, (reg_rhs1 >> 8) & 0xff);
	ret |= imx662_write_reg(sensor, RHS1_HIGH, (reg_rhs1 >> 16) & 0xff);
	ret |= imx662_write_reg(sensor, REGHOLD, 0);

	if (ret < 0) {
		pr_err("%s Failed to set vs exposure :\n", __func__);
		return ret;
	}

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
 * approach is simpler and avoids some subtle numerical approximation errors.
 */
static u32 imx662_get_gain_reg(u32 gain)
{
	u32 l = 0;
	u32 r = IMX662_GAIN_REG_LEN - 1;
	u32 mid;
	u32 ret = 0;

	// check if the gain value is outside the isp bounds, this should never happen
	if (gain < gain_reg2times[0]) {
		pr_warn("%s:isp returned too small gain value: %u, setting to min gain\n",
			__func__, gain);
		return 0;
	} else if (gain > gain_reg2times[IMX662_GAIN_REG_LEN-1]) {
		pr_warn("%s: isp returned too large gain value: %u, setting to max gain\n",
			__func__, gain);
		return 240;
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

static int imx662_set_gain(struct imx662 *sensor, u32 gain, u8 which_control)
{
	int ret = 0;
	u32 gain_reg = 0;
	u32 max_clear_hdr_gain = 80;

	pr_debug("enter %s: gain received: %u control: %u\n", __func__, gain, which_control);

	if (which_control == 0) { // from isp
		gain_reg = imx662_get_gain_reg(gain);
	} else { // from v4l2 control
		gain_reg = gain * IMX662_MAX_GAIN_DEC /
				 (IMX662_MAX_GAIN_DB * 10);
	}

	if (sensor->cur_mode.index == IMX662_CLEAR_INDEX) {
		if (gain_reg > max_clear_hdr_gain) {
			pr_warn("%s: gain setting for clear hdr too large setting to 80\n", __func__);
			gain_reg = max_clear_hdr_gain;
		}
	}

	pr_debug("enter %s gain register: %u\n", __func__, gain_reg);
	ret = imx662_write_reg(sensor, REGHOLD, 1);
	ret |= imx662_write_reg(sensor, GAIN_HIGH, (gain_reg>>8) & 0xff);
	ret |= imx662_write_reg(sensor, GAIN_LOW, gain_reg & 0xff);
	ret |= imx662_write_reg(sensor, REGHOLD, 0);

	return ret;
}

static int imx662_set_vs_gain(struct imx662 *sensor, u32 gain, u8 which_control)
{
	int ret = 0;
	u32 gain_reg = 0;
	const u32 max_vs_gain = 200;

	pr_debug("enter %s: gain received: %u control: %u\n", __func__, gain, which_control);

	if (which_control == 0) {
		gain_reg = imx662_get_gain_reg(gain);
	} else { // from v4l2 control
		gain_reg = gain * IMX662_MAX_GAIN_DEC /
				 (IMX662_MAX_GAIN_DB * 10);
	}

	// when vs gain is too large lines occurs on a screen
	if (gain_reg > max_vs_gain) {
		gain_reg = max_vs_gain;
		pr_info("%s: gain register too large, setting gain register to: %u\n", __func__, gain_reg);
	}

	pr_debug("%s: vs gain register: %u\n", __func__, gain_reg);
	ret = imx662_write_reg(sensor, REGHOLD, 1);
	ret |= imx662_write_reg(sensor, GAIN_1_HIGH, (gain_reg>>8) & 0xff);
	ret |= imx662_write_reg(sensor, GAIN_1_LOW, gain_reg & 0xff);
	ret |= imx662_write_reg(sensor, REGHOLD, 0);

	return ret;
}

static int imx662_set_exp_gain(struct imx662 *sensor, u32 gain, u8 which_control)
{
	int ret = 0;
	u32 gain_reg;
	const u32 max_exp_gain = 5;

	pr_debug("enter %s: exp gain received: %u control: %u\n", __func__, gain, which_control);
	if (which_control == 0) { // from isp
		if (gain < exp_gain_bounds[0])
			gain_reg = 0;
		else if (gain < exp_gain_bounds[1])
			gain_reg = 1;
		else if (gain < exp_gain_bounds[2])
			gain_reg = 2;
		else if (gain < exp_gain_bounds[3])
			gain_reg = 3;
		else if (gain < exp_gain_bounds[4])
			gain_reg = 4;
		else
			gain_reg = 5;
	} else { // from v4l2 control
		gain_reg = gain;
	}
	gain_reg = min_t(u32, max_exp_gain, gain_reg);

	pr_debug("%s: exp gain register: %u\n", __func__, gain_reg);
	ret = imx662_write_reg(sensor, EXP_GAIN, gain_reg);
	if (ret < 0) {
		pr_err(" %s: failed to set exp gain: %u\n", __func__, gain);
		return ret;
	}
	return ret;
}

static int imx662_set_black_level(struct imx662 *sensor, s64 val, u32 which_control)
{
	int ret = 0;
	s64 black_level_reg;

	pr_debug("enter %s black level: %lld\n",  __func__, val);
	if (sensor->format.code == MEDIA_BUS_FMT_SRGGB10_1X10)
		black_level_reg = val;
	else
		black_level_reg = val >> 2;

	ret = imx662_write_reg(sensor, REGHOLD, 1);
	ret |= imx662_write_reg(sensor, BLKLEVEL_HIGH, (black_level_reg>>8) & 0xff);
	ret |= imx662_write_reg(sensor, BLKLEVEL_LOW, black_level_reg & 0xff);
	ret |= imx662_write_reg(sensor, REGHOLD, 0);

	if (ret < 0) {
		pr_err("%s: BLACK LEVEL control error\n", __func__);
		return ret;
	}

	return 0;
}

static int imx662_set_fps(struct imx662 *sensor, u32 fps, u8 which_control)
{
	u32 fps_reg;
	u32 line_time;
	int ret = 0;

	pr_debug("enter %s fps received: %u\n", __func__, fps);
	if (which_control == 1)
		fps = fps << 10;

	line_time = sensor->cur_mode.ae_info.one_line_exp_time_ns;

	if (fps > sensor->cur_mode.ae_info.max_fps) {
		pr_warn("fps %u too large setting to %u\n", fps, sensor->cur_mode.ae_info.max_fps);
		fps = sensor->cur_mode.ae_info.max_fps;
	} else if (fps < sensor->cur_mode.ae_info.min_fps)
		fps = sensor->cur_mode.ae_info.min_fps;

	fps_reg = IMX662_G_FACTOR / ((fps >> 10) * line_time);

	if (sensor->cur_mode.index == IMX662_DOL_INDEX) {
		// in dol case there are two times more lines to read
		fps_reg = fps_reg / 2;
	}
	/* Value must be multiple of 2 */
	fps_reg = (fps_reg % 2) ? fps_reg + 1 : fps_reg;
	pr_debug("enter %s vmax register: %u line_time %u\n", __func__, fps_reg, line_time);
	ret = imx662_write_reg(sensor, REGHOLD, 1);
	ret |= imx662_write_reg(sensor, VMAX_HIGH, (u8)(fps_reg >> 16) & 0xff);
	ret |= imx662_write_reg(sensor, VMAX_MID, (u8)(fps_reg >> 8) & 0xff);
	ret |= imx662_write_reg(sensor, VMAX_LOW, (u8)(fps_reg & 0xff));
	ret |= imx662_write_reg(sensor, REGHOLD, 0);

	if (ret < 0) {
		pr_err("%s: failed to set VMAX register\n", __func__);
		return ret;
	}

	sensor->cur_mode.ae_info.cur_fps = fps;

	if (sensor->cur_mode.index == IMX662_DOL_INDEX)
		sensor->cur_mode.ae_info.max_integration_line = 2 * fps_reg - 2 - sensor->cur_mode.ae_info.max_vsintegration_line;
	else if (sensor->cur_mode.index == IMX662_CLEAR_INDEX)
		sensor->cur_mode.ae_info.max_integration_line = fps_reg - sensor->cur_mode.ae_info.min_integration_line;
	else
		sensor->cur_mode.ae_info.max_integration_line = fps_reg - sensor->cur_mode.ae_info.min_integration_line;

	sensor->cur_mode.ae_info.curr_frm_len_lines = fps_reg;
	return ret;
}

static int imx662_get_fps(struct imx662 *sensor, u32 *pfps)
{
	pr_debug("enter %s function\n", __func__);
	*pfps = sensor->cur_mode.ae_info.cur_fps;
	return 0;
}

static int imx662_set_test_pattern(struct imx662 *sensor, u32 pattern)
{
	int ret;

	pr_debug("enter %s function\n", __func__);

	if (pattern > 0 && pattern < ARRAY_SIZE(imx662_test_pattern_menu)) {
		ret = imx662_write_reg_arry(sensor,
					(struct vvcam_sccb_data_s *)mode_enable_pattern_generator,
					ARRAY_SIZE(mode_enable_pattern_generator));
		if (ret < 0) {
			pr_err("%s:imx662_write_reg_arry error\n", __func__);
			return -EINVAL;
		}
		ret = imx662_write_reg(sensor, TPG_PATSEL_DUOUT, pattern - 1);
	} else {
		ret = imx662_write_reg_arry(sensor,
			(struct vvcam_sccb_data_s *)mode_disable_pattern_generator,
			ARRAY_SIZE(mode_disable_pattern_generator));
		if (ret < 0) {
			pr_err("%s:imx662_write_reg_arry error\n", __func__);
			return -EINVAL;
		}
	}
	return ret;
}

// Should be refactored with new NXP version, not used at the moment
static int imx662_set_ratio(struct imx662 *sensor, void *pratio)
{
	int ret = 0;
	struct sensor_hdr_artio_s hdr_ratio;
	struct vvcam_ae_info_s *pae_info = &sensor->cur_mode.ae_info;
	pr_debug("enter %s\n", __func__);

	ret = copy_from_user(&hdr_ratio, pratio, sizeof(hdr_ratio));

	// this values are not used at the moment, l_s and s_vs should be the same
	pae_info->hdr_ratio.ratio_l_s = hdr_ratio.ratio_l_s;
	pae_info->hdr_ratio.ratio_s_vs = hdr_ratio.ratio_s_vs;
	// value for passed accuracy is wrong
	pae_info->hdr_ratio.accuracy = 1024;

	return ret;
}

static int imx662_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct imx662 *sensor = to_imx662_dev(sd);
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
		ret = imx662_set_gain(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_EXPOSURE:
		ret = imx662_set_exp(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = imx662_set_test_pattern(sensor, ctrl->val);
		break;
	case V4L2_CID_FRAME_RATE:
		ret = imx662_set_fps(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_BLACK_LEVEL:
		ret = imx662_set_black_level(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_DATA_RATE:
		ret = imx662_set_data_rate(sensor, ctrl->val);
		break;
	case V4L2_CID_SYNC_MODE:
		ret = imx662_set_sync_mode(sensor, ctrl->val);
		break;
	case V4L2_CID_VS_EXP:
		ret = imx662_set_vs_exp(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_VS_GAIN:
		ret = imx662_set_vs_gain(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_EXP_GAIN:
		ret = imx662_set_exp_gain(sensor, ctrl->val, 1);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops imx662_ctrl_ops = {
	.s_ctrl = imx662_s_ctrl,
};

static int imx662_get_format_code(struct imx662 *sensor, u32 *code)
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

static int imx662_parse_dt(struct imx662 *sensor, struct i2c_client *client)
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

	match = of_match_device(imx662_of_match, &client->dev);
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

static int imx662_set_pixel_format(struct imx662 *sensor)
{
	int err = 0;

	if ((sensor->cur_mode.bit_width == 10) && (sensor->cur_mode.index == IMX662_CLEAR_INDEX))
		err = imx662_write_reg_arry(
			sensor,
			(struct vvcam_sccb_data_s *)imx662_10bit_mode_clearHDR,
			ARRAY_SIZE(imx662_10bit_mode_clearHDR));
	else if (sensor->cur_mode.bit_width == 10)
		err = imx662_write_reg_arry(
			sensor,
			(struct vvcam_sccb_data_s *)imx662_10bit_mode,
			ARRAY_SIZE(imx662_10bit_mode));
	else if (sensor->cur_mode.bit_width == 12)
		err = imx662_write_reg_arry(
			sensor,
			(struct vvcam_sccb_data_s *)imx662_12bit_mode,
			ARRAY_SIZE(imx662_12bit_mode));
	else {
		pr_err("%s: unknown pixel format\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int imx662_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx662 *sensor = client_to_imx662(client);
	int err = 0;

	pr_debug("enter function %s\n", __func__);
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
		imx662_write_reg(sensor, STANDBY, 0x00);
		msleep(30);
		imx662_write_reg(sensor, XMSTA, 0x00);
		// 8 frame stabilisation - remove this?
		msleep(300);
	} else  {
		pr_info("Disable stream\n");
		if (!(strcmp(sensor->gmsl, "gmsl"))) {
			/* disable serdes streaming */

			max96792_stop_streaming(sensor->dser_dev, &sensor->i2c_client->dev);
		}
		imx662_write_reg(sensor, STANDBY, 0x01);
		msleep(30);
		imx662_write_reg(sensor, XMSTA, 0x01);
	}

	return 0;
exit:
	pr_err("%s: error setting stream\n", __func__);

	return err;
}

static int imx662_gmsl_serdes_setup(struct imx662 *priv)
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


	pr_debug("%s: max96792_setup_link\n", __func__);
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

static void imx662_gmsl_serdes_reset(struct imx662 *priv)
{
	mutex_lock(&priv->lock);

	/* reset serdes addressing and control pipeline */
	max96793_reset_control(priv->ser_dev);
	max96792_reset_control(priv->dser_dev, &priv->i2c_client->dev);

	max96792_power_off(priv->dser_dev, &priv->g_ctx);

	mutex_unlock(&priv->lock);
}

static int imx662_enum_mbus_code(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *state,
					 struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx662 *sensor = client_to_imx662(client);
	u32 cur_code = MEDIA_BUS_FMT_SRGGB12_1X12;

	pr_debug("enter %s function\n", __func__);
	if (code->index > 0)
		return -EINVAL;
	imx662_get_format_code(sensor, &cur_code);
	code->code = cur_code;

	return 0;
}

static int imx662_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx662 *sensor = client_to_imx662(client);

	mutex_lock(&sensor->lock);

	pr_debug("enter %s function\n", __func__);
	if ((fmt->format.width != sensor->cur_mode.size.bounds_width) ||
		(fmt->format.height != sensor->cur_mode.size.bounds_height)) {
		pr_err("%s:set sensor format %dx%d error\n",
			__func__, fmt->format.width, fmt->format.height);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}
	imx662_get_format_code(sensor, &fmt->format.code);
	fmt->format.field = V4L2_FIELD_NONE;
	sensor->format = fmt->format;

	ret = imx662_write_reg_arry(sensor,
		(struct vvcam_sccb_data_s *)sensor->cur_mode.preg_data,
		sensor->cur_mode.reg_data_count);
	if (ret < 0) {
		pr_err("%s:imx662_write_reg_arry error, error when setting initial data\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx662_set_pixel_format(sensor);
	if (ret < 0) {
		pr_err("%s:imx662_write_reg_arry error, failed to set pixel format\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	switch (sensor->cur_mode.index)	{
	case IMX662_ALL_PIXEL_INDEX:
		pr_info("%s:Setting mode 0 ", __func__);
		ret = imx662_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx662_setting_all_pixel, ARRAY_SIZE(imx662_setting_all_pixel));
		break;
	case IMX662_CROP_INDEX:
		pr_info("%s:Setting mode 1 ", __func__);
		ret = imx662_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx662_setting_crop, ARRAY_SIZE(imx662_setting_crop));
		break;
	case IMX662_BINNING_INDEX:
		pr_info("%s:Setting mode 2 ", __func__);
		ret = imx662_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx662_setting_binning, ARRAY_SIZE(imx662_setting_binning));
		break;
	case IMX662_BINNING_CROP_INDEX:
		pr_info("%s:Setting mode 3 ", __func__);
		ret = imx662_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx662_setting_binning_crop, ARRAY_SIZE(imx662_setting_binning_crop));
		break;
	case IMX662_DOL_INDEX:
		pr_info("%s:Setting mode 4 ", __func__);
		ret = imx662_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx662_setting_dol_hdr, ARRAY_SIZE(imx662_setting_dol_hdr));
		break;
	case IMX662_CLEAR_INDEX:
		pr_info("%s:Setting mode 5 ", __func__);
		ret = imx662_write_reg_arry(sensor, (struct vvcam_sccb_data_s *)imx662_setting_clear_hdr, ARRAY_SIZE(imx662_setting_clear_hdr));
		break;
	default:
		pr_err("%s:Invalid mode\n", __func__);
		ret = -1;
	}
	if (ret < 0)
		pr_err("%s:Failed to initialize settings for mode. Error while writing to setting to sensors/\n", __func__);

	ret = imx662_s_ctrl(sensor->ctrls.data_rate);
	if (ret < 0) {
		pr_err("%s:unable to set data rate\n", __func__);
		return -EINVAL;
	}
	mutex_unlock(&sensor->lock);

	return 0;
}

static int imx662_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx662 *sensor = client_to_imx662(client);

	pr_debug("enter %s function\n", __func__);
	mutex_lock(&sensor->lock);
	fmt->format = sensor->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

static long imx662_priv_ioctl(struct v4l2_subdev *sd,
				unsigned int cmd,
				void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx662 *sensor = client_to_imx662(client);
	long ret = 0;
	struct vvcam_sccb_data_s sensor_reg;

	pr_info("enter %s %u\n", __func__, cmd);
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
		ret = imx662_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		ret = imx662_query_supports(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = imx662_get_sensor_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID:
		ret = imx662_get_reserve_id(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = imx662_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = imx662_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		ret = imx662_s_stream(&sensor->sd, *(int *)arg);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx662_write_reg(sensor, sensor_reg.addr,
			sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx662_read_reg(sensor, sensor_reg.addr,
			(u8 *)&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg,
			sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_S_LONG_EXP:
		ret = 0;
		break;
	case VVSENSORIOC_S_EXP:
		ret = imx662_set_exp(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_S_VSEXP:
		ret = imx662_set_vs_exp(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_S_LONG_GAIN:
		ret = imx662_set_exp_gain(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_S_GAIN:
		ret = imx662_set_gain(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_S_VSGAIN:
		ret = imx662_set_vs_gain(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_S_FPS:
		ret = imx662_set_fps(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_G_FPS:
		ret = imx662_get_fps(sensor, (u32 *)arg);
		break;
	case VVSENSORIOC_S_HDR_RADIO:
		ret = imx662_set_ratio(sensor, arg);
		break;
	case VVSENSORIOC_S_BLC:
		ret = imx662_set_black_level(sensor, *(s64 *)arg, 0);
		break;
	case VVSENSORIOC_S_WB:
		ret = 0;
		break;
	case VVSENSORIOC_G_EXPAND_CURVE:
		ret = 0;
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		ret = imx662_set_test_pattern(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_DATA_RATE:
		ret = imx662_set_data_rate(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_SYNC_MODE:
		ret = imx662_set_sync_mode(sensor, *(u32 *)arg);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_video_ops imx662_subdev_video_ops = {
	.s_stream = imx662_s_stream,
};

static const struct v4l2_subdev_pad_ops imx662_subdev_pad_ops = {
	.enum_mbus_code = imx662_enum_mbus_code,
	.set_fmt = imx662_set_fmt,
	.get_fmt = imx662_get_fmt,
};

static const struct v4l2_subdev_core_ops imx662_subdev_core_ops = {
	.s_power = imx662_s_power,
	.ioctl = imx662_priv_ioctl,
};

static const struct v4l2_subdev_ops imx662_subdev_ops = {
	.core  = &imx662_subdev_core_ops,
	.video = &imx662_subdev_video_ops,
	.pad   = &imx662_subdev_pad_ops,
};

static int imx662_link_setup(struct media_entity *entity,
				 const struct media_pad *local,
				 const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations imx662_sd_media_ops = {
	.link_setup = imx662_link_setup,
};

static int imx662_probe(struct i2c_client *client)
{
	int retval;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct imx662 *sensor;

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

	pr_debug("enter %s function\n", __func__);

	sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	memset(sensor, 0, sizeof(*sensor));

	err = imx662_parse_dt(sensor, client);
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
							"imx662_mipi_reset");
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

		//mutex_init(&serdes_lock__);
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

		err = imx662_gmsl_serdes_setup(sensor);
		if (err) {
			dev_err(dev, "%s gmsl serdes setup failed\n", __func__);
			return err;
		}
	}

	retval = imx662_power_on(sensor);
	if (retval < 0) {
		dev_err(dev, "%s: sensor power on fail\n", __func__);
		goto probe_err_power_off;
	}

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &imx662_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.ops = &imx662_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[IMX662_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity,
				IMX662_SENS_PADS_NUM,
				sensor->pads);
	if (retval < 0)
		goto probe_err_power_off;

	memcpy(&sensor->cur_mode, &pimx662_mode_info[0],
		sizeof(struct vvcam_mode_info_s));

	/* initialize controls */
	retval = v4l2_ctrl_handler_init(&sensor->ctrls.handler, V4L2_NUM_CTRLS);
	if (retval < 0) {
		dev_err(&client->dev,
			"%s : ctrl handler init Failed\n", __func__);
		goto probe_err_power_off;
	}

	sensor->ctrls.handler.lock = &sensor->lock;

	// add new controls
	sensor->ctrls.exposure = v4l2_ctrl_new_std(&sensor->ctrls.handler, &imx662_ctrl_ops, V4L2_CID_EXPOSURE,
						3, 30000, 1, 1000);
	sensor->ctrls.gain = v4l2_ctrl_new_std(&sensor->ctrls.handler, &imx662_ctrl_ops, V4L2_CID_GAIN,
					0, 240, 3, 0);

	sensor->ctrls.black_level = v4l2_ctrl_new_std(&sensor->ctrls.handler, &imx662_ctrl_ops, V4L2_CID_BLACK_LEVEL,
					 0, 1023, 1, 50);

	sensor->ctrls.data_rate = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx662_ctrl_data_rate, NULL);
	sensor->ctrls.sync_mode = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx662_ctrl_sync_mode, NULL);
	sensor->ctrls.framerate = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx662_ctrl_framerate, NULL);

	sensor->ctrls.vs_exp = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx662_ctrl_vs_exp, NULL);
	sensor->ctrls.vs_gain = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx662_ctrl_vs_gain, NULL);
	sensor->ctrls.exp_gain = v4l2_ctrl_new_custom(&sensor->ctrls.handler, imx662_ctrl_exp_gain, NULL);

	sensor->ctrls.test_pattern = v4l2_ctrl_new_std_menu_items(&sensor->ctrls.handler, &imx662_ctrl_ops, V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(imx662_test_pattern_menu) - 1, 0, 0, imx662_test_pattern_menu);

	sd->ctrl_handler = &sensor->ctrls.handler;
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

	pr_info("%s camera mipi imx662, is found\n", __func__);

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
probe_err_free_entiny:
	media_entity_cleanup(&sd->entity);

probe_err_power_off:
	imx662_power_off(sensor);

	return retval;
}

static void imx662_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx662 *sensor = client_to_imx662(client);
	int err = 0;

	pr_debug("enter %s function\n", __func__);
	err = imx662_write_reg(sensor, XVS_DRV_XHS_DRV, 0xF);
	if (err < 0)
		pr_err("%s: failed to set XVS XHS to Hi-Z\n", __func__);


	if (!(strcmp(sensor->gmsl, "gmsl"))) {
		max96792_sdev_unregister(sensor->dser_dev, &sensor->i2c_client->dev);
		imx662_gmsl_serdes_reset(sensor);
	}

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx662_power_off(sensor);
	mutex_destroy(&sensor->lock);
}

static int __maybe_unused imx662_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx662 *sensor = client_to_imx662(client);

	sensor->resume_status = sensor->stream_status;
	if (sensor->resume_status)
		imx662_s_stream(&sensor->sd, 0);

	return 0;
}

static int __maybe_unused imx662_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx662 *sensor = client_to_imx662(client);

	if (sensor->resume_status)
		imx662_s_stream(&sensor->sd, 1);


	return 0;
}

static const struct dev_pm_ops imx662_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx662_suspend, imx662_resume)
};

static const struct i2c_device_id imx662_id[] = {
	{"imx662", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, imx662_id);


static struct i2c_driver imx662_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "imx662",
		.pm = &imx662_pm_ops,
		.of_match_table	= imx662_of_match,
	},
	.probe  = imx662_probe,
	.remove = imx662_remove,
	.id_table = imx662_id,
};

module_i2c_driver(imx662_i2c_driver);
MODULE_DESCRIPTION("IMX662 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
