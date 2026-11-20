// SPDX-License-Identifier: GPL-2.0
/*
  Copyright (c) 2026 FRAMOS GmbH
 
  vb1943_mipi.c - FRAMOS VB1943 image sensor driver
 */

//#define DEBUG 1
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/ktime.h>
#include <linux/math64.h>
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
#include "vb1943_regs.h"
#include "max96792.h"
#include "max96793.h"

#define VB1943_MAX_RETRIES               10

#define VB1943_SENS_PAD_SOURCE           0
#define VB1943_SENS_PADS_NUM             1

#define VB1943_K_FACTOR                  1000LL
#define VB1943_M_FACTOR                  1000000LL
#define VB1943_G_FACTOR                  1000000000LL

#define VB1943_MIN_AGAIN_DEC             100
#define VB1943_MAX_AGAIN_DEC             400

#define VB1943_MIN_DGAIN_DEC             0
#define VB1943_MAX_DGAIN_DEC             3199
#define VB1943_DEF_DGAIN_DEC             100
#define VB1943_DGAIN_ONE                 256

#define VB1943_MAX_ISP_GAIN              (16 * 1024)

#define VB1943_MAX_BLACK_LEVEL           4095
#define VB1943_DEFAULT_BLACK_LEVEL       32

#define VB1943_MIN_INTEGRATION_LINES     4
#define VB1943_INTEGRATION_LINES_MARGIN  28

#define VB1943_XCLK_FREQ                 25000000

#define VB1943_FOUR_LANE_MODE            4
#define VB1943_DEFAULT_DATA_RATE_MBPS    1500
#define VB1943_CFA_MONO                  1
#define VB1943_TWO_LANE_MODE             2

#define VB1943_PIXEL_CLK                 93750000LL
#define VB1943_DEFAULT_LINE_LENGTH       3372
#define VB1943_TWO_LANE_10BIT_LINE_LENGTH 6744
#define VB1943_TWO_LANE_12BIT_LINE_LENGTH 7500
#define VB1943_LINE_TIME_4LANE           8992 // in ns

#define VB1943_GS_MIN_FRAME_LENGTH       1112
#define VB1943_RS_MIN_FRAME_LENGTH       (2 * VB1943_GS_MIN_FRAME_LENGTH)
#define VB1943_MAX_FRAME_LENGTH          0xFFFE

#define VB1943_GPIO_MODE_FSYNC_IN        0x05
#define VB1943_GPIO_MODE_FSYNC_OUT       0x07
#define VB1943_GPIO0_EN                  BIT(0)
#define VB1943_GPIO1_EN                  BIT(1)
#define VB1943_VT_CTRL_MASTER            0x00
#define VB1943_VT_CTRL_SLAVE             0x01

#define VB1943_CFA_ALPHA_NORMAL          0x00
#define VB1943_CFA_ALPHA_DAMPER          0x01
#define VB1943_CFA_ALPHA_ZERO            0x02

#define VB1943_FSM_SW_STBY               0x03
#define VB1943_FSM_STREAMING             0x04

#define VB1943_DARKCAL_ON                0x01
#define VB1943_DARKCAL_OFF               0x00
#define VB1943_DUSTER_OFF                0x00

#define V4L2_CID_FRAME_RATE              (V4L2_CID_USER_IMX_BASE + 1)
#define V4L2_CID_SHUTTER_MODE            (V4L2_CID_USER_IMX_BASE + 2)
#define V4L2_CID_COLOR_MODE              (V4L2_CID_USER_IMX_BASE + 3)
#define V4L2_CID_OPERATION_MODE          (V4L2_CID_USER_IMX_BASE + 4)
#define V4L2_CID_TEMPERATURE             (V4L2_CID_USER_IMX_BASE + 5)
#define V4L2_CID_DATA_RATE               (V4L2_CID_USER_IMX_BASE + 6)
#define V4L2_NUM_CTRLS                   10

enum mode_index {
	VB1943_ALL_PIXEL_10BIT_INDEX,
	VB1943_CROP_2064x1552_10BIT_INDEX,
	VB1943_CROP_1920x1080_10BIT_INDEX,
	VB1943_ALL_PIXEL_12BIT_INDEX,
	VB1943_CROP_2064x1552_12BIT_INDEX,
	VB1943_CROP_1920x1080_12BIT_INDEX,
	VB1943_MAX_INDEX
};

static const struct of_device_id vb1943_of_match[] = {
	{ .compatible = "framos,vb1943" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vb1943_of_match);

static const char * const vb1943_test_pattern_menu[] = {
	[0] = "No pattern",
	[1] = "Diagonal Grayscale(DGREY)",
	[2] = "Pseudorandom(PN28)",
};

enum shutter_mode {
	VB1943_GS_MODE,
	VB1943_RS_MODE,
};

static const char * const vb1943_shutter_mode_menu[] = {
	[VB1943_GS_MODE] = "Global Shutter",
	[VB1943_RS_MODE] = "Rolling Shutter",
};

enum color_mode {
	VB1943_RGBNIR_MODE,
	VB1943_RGB_MODE,
	VB1943_NIR_MODE,
};

static const char * const vb1943_color_mode_menu[] = {
	[VB1943_RGBNIR_MODE] = "RGB-NIR",
	[VB1943_RGB_MODE]    = "RGB",
	[VB1943_NIR_MODE]    = "NIR",
};

/*
 * Master/slave operation mode, named like in the other Framos sensor drivers.
 * The VB1943 documentation refers to master mode as "leader" and to slave
 * mode as "follower" (VT_CTRL register).
 */
enum operation_mode {
	VB1943_MASTER_MODE,
	VB1943_SLAVE_MODE,
};

static const char * const vb1943_operation_mode_menu[] = {
	[VB1943_MASTER_MODE] = "Master",
	[VB1943_SLAVE_MODE]  = "Slave",
};

struct vb1943_again {
	u32 gain_x100;
	u8  reg_value;
};

static const struct vb1943_again vb1943_again_lut[] = {
	{ 100, 0x00 },
	{ 107, 0x01 },
	{ 114, 0x02 },
	{ 123, 0x03 },
	{ 133, 0x04 },
	{ 145, 0x05 },
	{ 160, 0x06 },
	{ 178, 0x07 },
	{ 200, 0x08 },
	{ 229, 0x09 },
	{ 267, 0x0A },
	{ 320, 0x0B },
	{ 400, 0x0C },
};

static const struct v4l2_ctrl_ops vb1943_ctrl_ops;

static struct v4l2_ctrl_config vb1943_ctrl_framerate[] = {
	{
		.ops = &vb1943_ctrl_ops,
		.id = V4L2_CID_FRAME_RATE,
		.name = "Frame rate",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 2,
		.max = 100,
		.def = 60,
		.step = 1,
	},
};

static struct v4l2_ctrl_config vb1943_ctrl_shutter_mode[] = {
	{
		.ops = &vb1943_ctrl_ops,
		.id = V4L2_CID_SHUTTER_MODE,
		.name = "Global/Rolling Shutter Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = VB1943_GS_MODE,
		.max = VB1943_RS_MODE,
		.def = VB1943_GS_MODE,
		.qmenu = vb1943_shutter_mode_menu,
	},
};

/*
 * RGB is the default, the NXP ISP needs a bayer pattern input. RGB-NIR
 * outputs the native RGB-IR CFA and NIR the upscaled IR channel.
 * On the mono variant (CFA_SELECTION = 1) RGB-NIR is the native pixel
 * pass-through and the default, NIR is upscaled (softer) and RGB is not
 * supported (the RGB conversion outputs a black image).
 */
static struct v4l2_ctrl_config vb1943_ctrl_color_mode[] = {
	{
		.ops = &vb1943_ctrl_ops,
		.id = V4L2_CID_COLOR_MODE,
		.name = "RGB-NIR/RGB/NIR Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = VB1943_RGBNIR_MODE,
		.max = VB1943_NIR_MODE,
		.def = VB1943_RGB_MODE,
		.qmenu = vb1943_color_mode_menu,
	},
};

static struct v4l2_ctrl_config vb1943_ctrl_operation_mode[] = {
	{
		.ops = &vb1943_ctrl_ops,
		.id = V4L2_CID_OPERATION_MODE,
		.name = "Operation mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = VB1943_MASTER_MODE,
		.max = VB1943_SLAVE_MODE,
		.def = VB1943_MASTER_MODE,
		.qmenu = vb1943_operation_mode_menu,
	},
};

static const u32 vb1943_data_rates_mbps[] = { 1500, 1200, 1000, 800, 600 };

static const char * const vb1943_data_rate_menu[] = {
	"1500 Mbps/lane",
	"1200 Mbps/lane",
	"1000 Mbps/lane",
	"800 Mbps/lane",
	"600 Mbps/lane",
};

static struct v4l2_ctrl_config vb1943_ctrl_data_rate[] = {
	{
		.ops = &vb1943_ctrl_ops,
		.id = V4L2_CID_DATA_RATE,
		.name = "Data rate",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(vb1943_data_rate_menu) - 1,
		.def = 0,
		.qmenu = vb1943_data_rate_menu,
	},
};

static struct v4l2_ctrl_config vb1943_ctrl_temperature[] = {
	{
		.ops = &vb1943_ctrl_ops,
		.id = V4L2_CID_TEMPERATURE,
		.name = "Sensor temperature",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
		.min = -32768,
		.max = 32767,
		.def = 0,
		.step = 1,
	},
};

struct vb1943_ctrls {
	struct v4l2_ctrl_handler handler;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	struct v4l2_ctrl *digital_gain;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *framerate;
	struct v4l2_ctrl *black_level;
	struct v4l2_ctrl *shutter_mode;
	struct v4l2_ctrl *color_mode;
	struct v4l2_ctrl *operation_mode;
	struct v4l2_ctrl *temperature;
	struct v4l2_ctrl *data_rate;
};

struct vb1943 {
	struct i2c_client *i2c_client;
	unsigned int rst_gpio;
	unsigned int pwn_gpio;
	struct clk *mclk;
	unsigned int csi_id;
	unsigned int powered_on;

	struct v4l2_subdev sd;
	struct media_pad pads[VB1943_SENS_PADS_NUM];

	struct v4l2_mbus_framefmt format;
	vvcam_mode_info_t cur_mode;
	struct mutex lock;
	u32 stream_status;
	u32 resume_status;
	struct vb1943_ctrls ctrls;
	const char *gmsl;
	struct device *ser_dev;
	struct device *dser_dev;
	struct gmsl_link_ctx g_ctx;

	u32 num_lanes;
	u32 data_rate_mbps;
	u32 dt_data_rate_mbps;
	u32 line_length;
	u32 frame_length;
	u32 min_frame_length;
	u32 target_fps;
	u32 shutter_mode;
	bool rs_forced;
	u32 color_mode;
	u32 mono;
	u32 operation_mode;
	u32 test_pattern;
	u8 duster_ctrl;
	u32 black_level;
};

#define client_to_vb1943(client)\
	container_of(i2c_get_clientdata(client), struct vb1943, sd)

static inline struct vb1943 *to_vb1943_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct vb1943, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct vb1943,
				ctrls.handler)->sd;
}

static struct vvcam_mode_info_s pvb1943_mode_info[] = {
	{
		.index          = VB1943_ALL_PIXEL_10BIT_INDEX,
		.size           = {
			.bounds_width  = VB1943_DEFAULT_WIDTH,
			.bounds_height = VB1943_DEFAULT_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = VB1943_DEFAULT_WIDTH,
			.height        = VB1943_DEFAULT_HEIGHT,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines     = VB1943_GS_MIN_FRAME_LENGTH,
			.curr_frm_len_lines    = VB1943_GS_MIN_FRAME_LENGTH,
			.one_line_exp_time_ns  = VB1943_LINE_TIME_4LANE,

			.max_integration_line  = VB1943_GS_MIN_FRAME_LENGTH - VB1943_INTEGRATION_LINES_MARGIN,
			.min_integration_line  = VB1943_MIN_INTEGRATION_LINES,

			.max_again             = 4 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = VB1943_MAX_ISP_GAIN,
			.min_dgain             = 1 * 1024,
			.gain_step             = 32,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 60 * 1024,
			.max_fps               = 100 * 1024,
			.min_fps               = 2 * 1024,
			.min_afps              = 2 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = vb1943_init_setting,
		.reg_data_count = ARRAY_SIZE(vb1943_init_setting),
	},
	{
		.index          = VB1943_CROP_2064x1552_10BIT_INDEX,
		.size           = {
			.bounds_width  = VB1943_CROP_2064x1552_WIDTH,
			.bounds_height = VB1943_CROP_2064x1552_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = VB1943_CROP_2064x1552_WIDTH,
			.height        = VB1943_CROP_2064x1552_HEIGHT,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines     = VB1943_GS_MIN_FRAME_LENGTH,
			.curr_frm_len_lines    = VB1943_GS_MIN_FRAME_LENGTH,
			.one_line_exp_time_ns  = VB1943_LINE_TIME_4LANE,

			.max_integration_line  = VB1943_GS_MIN_FRAME_LENGTH - VB1943_INTEGRATION_LINES_MARGIN,
			.min_integration_line  = VB1943_MIN_INTEGRATION_LINES,

			.max_again             = 4 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = VB1943_MAX_ISP_GAIN,
			.min_dgain             = 1 * 1024,
			.gain_step             = 32,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 100 * 1024,
			.max_fps               = 100 * 1024,
			.min_fps               = 2 * 1024,
			.min_afps              = 2 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = vb1943_init_setting,
		.reg_data_count = ARRAY_SIZE(vb1943_init_setting),
	},
	{
		.index          = VB1943_CROP_1920x1080_10BIT_INDEX,
		.size           = {
			.bounds_width  = VB1943_CROP_1920x1080_WIDTH,
			.bounds_height = VB1943_CROP_1920x1080_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = VB1943_CROP_1920x1080_WIDTH,
			.height        = VB1943_CROP_1920x1080_HEIGHT,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 10,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines     = VB1943_GS_MIN_FRAME_LENGTH,
			.curr_frm_len_lines    = VB1943_GS_MIN_FRAME_LENGTH,
			.one_line_exp_time_ns  = VB1943_LINE_TIME_4LANE,

			.max_integration_line  = VB1943_GS_MIN_FRAME_LENGTH - VB1943_INTEGRATION_LINES_MARGIN,
			.min_integration_line  = VB1943_MIN_INTEGRATION_LINES,

			.max_again             = 4 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = VB1943_MAX_ISP_GAIN,
			.min_dgain             = 1 * 1024,
			.gain_step             = 32,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 100 * 1024,
			.max_fps               = 100 * 1024,
			.min_fps               = 2 * 1024,
			.min_afps              = 2 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = vb1943_init_setting,
		.reg_data_count = ARRAY_SIZE(vb1943_init_setting),
	},
	{
		.index          = VB1943_ALL_PIXEL_12BIT_INDEX,
		.size           = {
			.bounds_width  = VB1943_DEFAULT_WIDTH,
			.bounds_height = VB1943_DEFAULT_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = VB1943_DEFAULT_WIDTH,
			.height        = VB1943_DEFAULT_HEIGHT,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines     = VB1943_RS_MIN_FRAME_LENGTH,
			.curr_frm_len_lines    = VB1943_RS_MIN_FRAME_LENGTH,
			.one_line_exp_time_ns  = VB1943_LINE_TIME_4LANE,

			.max_integration_line  = VB1943_RS_MIN_FRAME_LENGTH - VB1943_INTEGRATION_LINES_MARGIN,
			.min_integration_line  = VB1943_MIN_INTEGRATION_LINES,

			.max_again             = 4 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = VB1943_MAX_ISP_GAIN,
			.min_dgain             = 1 * 1024,
			.gain_step             = 32,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 50 * 1024,
			.max_fps               = 50 * 1024,
			.min_fps               = 1 * 1024,
			.min_afps              = 1 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = vb1943_init_setting,
		.reg_data_count = ARRAY_SIZE(vb1943_init_setting),
	},
	{
		.index          = VB1943_CROP_2064x1552_12BIT_INDEX,
		.size           = {
			.bounds_width  = VB1943_CROP_2064x1552_WIDTH,
			.bounds_height = VB1943_CROP_2064x1552_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = VB1943_CROP_2064x1552_WIDTH,
			.height        = VB1943_CROP_2064x1552_HEIGHT,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines     = VB1943_RS_MIN_FRAME_LENGTH,
			.curr_frm_len_lines    = VB1943_RS_MIN_FRAME_LENGTH,
			.one_line_exp_time_ns  = VB1943_LINE_TIME_4LANE,

			.max_integration_line  = VB1943_RS_MIN_FRAME_LENGTH - VB1943_INTEGRATION_LINES_MARGIN,
			.min_integration_line  = VB1943_MIN_INTEGRATION_LINES,

			.max_again             = 4 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = VB1943_MAX_ISP_GAIN,
			.min_dgain             = 1 * 1024,
			.gain_step             = 32,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 50 * 1024,
			.max_fps               = 50 * 1024,
			.min_fps               = 1 * 1024,
			.min_afps              = 1 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = vb1943_init_setting,
		.reg_data_count = ARRAY_SIZE(vb1943_init_setting),
	},
	{
		.index          = VB1943_CROP_1920x1080_12BIT_INDEX,
		.size           = {
			.bounds_width  = VB1943_CROP_1920x1080_WIDTH,
			.bounds_height = VB1943_CROP_1920x1080_HEIGHT,
			.top           = 0,
			.left          = 0,
			.width         = VB1943_CROP_1920x1080_WIDTH,
			.height        = VB1943_CROP_1920x1080_HEIGHT,
		},
		.hdr_mode       = SENSOR_MODE_LINEAR,
		.bit_width      = 12,
		.data_compress  = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GBRG,
		.ae_info = {
			.def_frm_len_lines     = VB1943_RS_MIN_FRAME_LENGTH,
			.curr_frm_len_lines    = VB1943_RS_MIN_FRAME_LENGTH,
			.one_line_exp_time_ns  = VB1943_LINE_TIME_4LANE,

			.max_integration_line  = VB1943_RS_MIN_FRAME_LENGTH - VB1943_INTEGRATION_LINES_MARGIN,
			.min_integration_line  = VB1943_MIN_INTEGRATION_LINES,

			.max_again             = 4 * 1024,
			.min_again             = 1 * 1024,
			.max_dgain             = VB1943_MAX_ISP_GAIN,
			.min_dgain             = 1 * 1024,
			.gain_step             = 32,

			.start_exposure        = 3000 * 1024,
			.cur_fps               = 50 * 1024,
			.max_fps               = 50 * 1024,
			.min_fps               = 1 * 1024,
			.min_afps              = 1 * 1024,
			.int_update_delay_frm  = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 4,
		},
		.preg_data      = vb1943_init_setting,
		.reg_data_count = ARRAY_SIZE(vb1943_init_setting),
	},
};

static int vb1943_write_reg(struct vb1943 *sensor, u16 reg, u8 val)
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
	 * Retry sending a message for VB1943_MAX_RETRIES and report a problem.
	 */
	for (num_retry = 0; num_retry < VB1943_MAX_RETRIES; num_retry++) {
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

static int vb1943_read_reg(struct vb1943 *sensor, u16 reg, u8 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8RegBuf[2] = { 0 };
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (i2c_master_send(sensor->i2c_client, au8RegBuf, 2) != 2) {
		dev_err(dev, "Read reg error: reg=%x\n", reg);
		return -EIO;
	}

	if (i2c_master_recv(sensor->i2c_client, &u8RdVal, 1) != 1) {
		dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, u8RdVal);
		return -EIO;
	}

	*val = u8RdVal;

	return 0;
}

/* Multi byte registers are little endian, lowest byte at the lowest address */
static int vb1943_write_reg_le(struct vb1943 *sensor, u16 reg, u8 len, u32 val)
{
	int ret = 0;
	int i;

	for (i = 0; i < len; i++) {
		ret = vb1943_write_reg(sensor, reg + i, (val >> (i * 8)) & 0xff);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int vb1943_read_reg_le(struct vb1943 *sensor, u16 reg, u8 len, u32 *val)
{
	int ret = 0;
	u8 reg_val;
	int i;

	*val = 0;
	for (i = 0; i < len; i++) {
		ret = vb1943_read_reg(sensor, reg + i, &reg_val);
		if (ret < 0)
			return ret;
		*val |= (u32)reg_val << (i * 8);
	}

	return 0;
}

static int vb1943_write_reg_arry(struct vb1943 *sensor,
				 struct vvcam_sccb_data_s *reg_arry,
				 u32 size)
{
	int ret = 0;
	int i;

	for (i = 0; i < size; i++) {
		ret = vb1943_write_reg(sensor, reg_arry[i].addr, reg_arry[i].data & 0xff);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int vb1943_group_hold(struct vb1943 *sensor, u8 val)
{
	return vb1943_write_reg(sensor, CTX0_GROUP_PARAMETER_HOLD, val);
}

static u64 vb1943_frame_time_us(struct vb1943 *sensor)
{
	return div_u64((u64)sensor->frame_length *
			sensor->cur_mode.ae_info.one_line_exp_time_ns, VB1943_K_FACTOR);
}

static int vb1943_wait_fsm_state(struct vb1943 *sensor, u8 state, u8 *fsm_state)
{
	ktime_t timeout = ktime_add_us(ktime_get(), 2 * vb1943_frame_time_us(sensor) + 50000);
	int ret;

	for (;;) {
		ret = vb1943_read_reg(sensor, SYSTEM_FSM_STATE, fsm_state);
		if (ret < 0)
			return ret;

		if (*fsm_state == state)
			return 0;

		if (ktime_after(ktime_get(), timeout))
			return -ETIMEDOUT;

		usleep_range(500, 1000);
	}
}

/*
 * Stop streaming, static registers are writable only in SW standby.
 * The stop command must be sent only while streaming, a stop command in
 * SW standby sets SYSTEM_WARNING and the sensor then stops after one
 * frame on every following stream start.
 */
static int vb1943_enter_sw_standby(struct vb1943 *sensor)
{
	u8 fsm_state = 0;
	int ret;

	ret = vb1943_read_reg(sensor, SYSTEM_FSM_STATE, &fsm_state);
	if (ret < 0)
		return ret;

	if (fsm_state == VB1943_FSM_SW_STBY)
		return 0;

	ret = vb1943_write_reg(sensor, STREAMING, 0x01);
	if (ret < 0) {
		pr_err("%s: unable to enter SW standby\n", __func__);
		return ret;
	}

	ret = vb1943_wait_fsm_state(sensor, VB1943_FSM_SW_STBY, &fsm_state);
	if (ret < 0)
		pr_err("%s: SW standby not reached (%d), FSM: 0x%02x\n",
		       __func__, ret, fsm_state);

	return ret;
}

static int vb1943_exit_sw_standby(struct vb1943 *sensor)
{
	u8 fsm_state = 0;
	int ret;

	ret = vb1943_read_reg(sensor, SYSTEM_FSM_STATE, &fsm_state);
	if (ret < 0)
		return ret;

	if (fsm_state == VB1943_FSM_STREAMING)
		return 0;

	if (fsm_state != VB1943_FSM_SW_STBY)
		pr_warn("%s: unexpected FSM state 0x%02x before stream start\n", __func__, fsm_state);

	ret = vb1943_write_reg(sensor, SW_STBY, 0x01);
	if (ret < 0) {
		pr_err("%s: unable to start streaming\n", __func__);
		return ret;
	}

	ret = vb1943_wait_fsm_state(sensor, VB1943_FSM_STREAMING, &fsm_state);
	if (ret < 0)
		pr_warn("%s: streaming not confirmed (%d), FSM: 0x%02x\n",
			__func__, ret, fsm_state);

	return 0;
}

static int vb1943_static_begin(struct vb1943 *sensor)
{
	if (!sensor->stream_status)
		return 0;

	return vb1943_enter_sw_standby(sensor);
}

static int vb1943_static_end(struct vb1943 *sensor)
{
	if (!sensor->stream_status)
		return 0;

	return vb1943_exit_sw_standby(sensor);
}

static int vb1943_boot(struct vb1943 *sensor)
{
	int ret;
	u8 fsm_state = 0;

	ret = vb1943_write_reg(sensor, SYSTEM_UP, 0x01);
	if (ret < 0) {
		pr_err("%s: error setting SYSTEM_UP register, value START_SENSOR\n", __func__);
		return ret;
	}

	usleep_range(10000, 11000);

	ret = vb1943_write_reg(sensor, BOOT, 0x10);
	if (ret < 0) {
		pr_err("%s: error setting BOOT register, value END_BOOT\n", __func__);
		return ret;
	}

	/* Additional sleep required in the case of hardware power-on sequence */
	usleep_range(40000, 41000);

	ret = vb1943_read_reg(sensor, SYSTEM_FSM_STATE, &fsm_state);
	if (ret < 0)
		return ret;

	if (fsm_state != VB1943_FSM_SW_STBY)
		pr_warn("%s: sensor not in SW standby after boot, FSM: 0x%02x\n", __func__, fsm_state);
	else
		pr_debug("%s: sensor booted, FSM: 0x%02x\n", __func__, fsm_state);

	return 0;
}

static unsigned int data_rate_mbps;
module_param(data_rate_mbps, uint, 0444);
MODULE_PARM_DESC(data_rate_mbps, "MIPI data rate per lane in Mbps (0 = from DT link-frequencies, default 1500)");

static int vb1943_set_sensor_settings(struct vb1943 *sensor)
{
	int ret;

	ret = vb1943_write_reg(sensor, LANE_NB_SEL,
			       (sensor->num_lanes == VB1943_TWO_LANE_MODE) ? 0x01 : 0x00);
	ret |= vb1943_write_reg_le(sensor, MIPI_DATA_RATE, 4, sensor->data_rate_mbps * 1000000);
	if (ret < 0) {
		pr_err("%s: unable to set sensor settings\n", __func__);
		return ret;
	}

	pr_debug("%s: %u lanes, %u Mbps\n", __func__, sensor->num_lanes, sensor->data_rate_mbps);

	return 0;
}

static int __vb1943_power_on(struct vb1943 *sensor)
{
	int ret;

	pr_debug("enter %s function\n", __func__);
	if (strcmp(sensor->gmsl, "gmsl")) {
		if (!gpio_is_valid(sensor->rst_gpio)) {
			pr_err("%s:reset pin is not valid\n", __func__);
			return -EINVAL;
		}
		/* Sensor power enable, connected to the PWN pin */
		if (gpio_is_valid(sensor->pwn_gpio)) {
			gpio_set_value_cansleep(sensor->pwn_gpio, 1);
			usleep_range(5000, 6000);
		}
		if (sensor->mclk) {
			ret = clk_prepare_enable(sensor->mclk);
			if (ret < 0) {
				pr_err("%s: failed to enable mclk\n", __func__);
				if (gpio_is_valid(sensor->pwn_gpio))
					gpio_set_value_cansleep(sensor->pwn_gpio, 0);
				return ret;
			}
			usleep_range(1000, 2000);
		}
		gpio_set_value_cansleep(sensor->rst_gpio, 1);
	} else {
		/* For now no separate power on required for serializer device */
		pr_debug("%s: max96792_power_on\n", __func__);
		max96792_power_on(sensor->dser_dev, &sensor->g_ctx);
	}

	usleep_range(100000, 110000);

	ret = vb1943_boot(sensor);
	if (!ret)
		ret = vb1943_set_sensor_settings(sensor);
	if (ret < 0) {
		pr_err("%s: sensor boot failed\n", __func__);
		if (strcmp(sensor->gmsl, "gmsl")) {
			gpio_set_value_cansleep(sensor->rst_gpio, 0);
			if (sensor->mclk)
				clk_disable_unprepare(sensor->mclk);
			if (gpio_is_valid(sensor->pwn_gpio))
				gpio_set_value_cansleep(sensor->pwn_gpio, 0);
		}
		return ret;
	}

	sensor->powered_on = 1;

	return 0;
}

static int vb1943_power_on(struct vb1943 *sensor)
{
	int ret;

	mutex_lock(&sensor->lock);
	ret = __vb1943_power_on(sensor);
	mutex_unlock(&sensor->lock);

	return ret;
}

static int __vb1943_power_off(struct vb1943 *sensor)
{
	pr_debug("enter %s function\n", __func__);


	if (strcmp(sensor->gmsl, "gmsl")) {
		if (!gpio_is_valid(sensor->rst_gpio)) {
			pr_err("%s:reset pin is not valid\n", __func__);
			return -EINVAL;
		}
		gpio_set_value_cansleep(sensor->rst_gpio, 0);
		if (sensor->mclk && sensor->powered_on)
			clk_disable_unprepare(sensor->mclk);
		if (gpio_is_valid(sensor->pwn_gpio))
			gpio_set_value_cansleep(sensor->pwn_gpio, 0);
	} else {
		pr_debug("%s: max96792_power_off\n", __func__);
		max96792_power_off(sensor->dser_dev, &sensor->g_ctx);
	}

	sensor->powered_on = 0;
	msleep(20);

	return 0;
}

static int vb1943_power_off(struct vb1943 *sensor)
{
	int ret;

	mutex_lock(&sensor->lock);
	ret = __vb1943_power_off(sensor);
	mutex_unlock(&sensor->lock);

	return ret;
}

static int vb1943_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vb1943 *sensor = client_to_vb1943(client);
	int ret = 0;

	pr_debug("enter %s function\n", __func__);
	if (on)
		ret = vb1943_power_on(sensor);
	else
		ret = vb1943_power_off(sensor);

	if (ret < 0)
		return ret;
	return 0;
}

static int vb1943_query_capability(struct vb1943 *sensor, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strscpy((char *)pcap->driver, "vb1943", sizeof(pcap->driver));
	sprintf((char *)pcap->bus_info, "csi%d", sensor->csi_id);
	if (sensor->i2c_client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}
	return 0;
}

static int vb1943_query_supports(struct vb1943 *sensor, void *parry)
{
	int ret = 0;
	struct vvcam_mode_info_array_s *psensor_mode_arry = parry;
	uint32_t support_counts = ARRAY_SIZE(pvb1943_mode_info);

	pr_debug("enter %s function\n", __func__);

	ret = copy_to_user(&psensor_mode_arry->count, &support_counts, sizeof(support_counts));
	ret |= copy_to_user(&psensor_mode_arry->modes, pvb1943_mode_info,
			   sizeof(pvb1943_mode_info));
	if (ret != 0) {
		pr_err("enter %s failed to allocate memory\n", __func__);
		ret = -ENOMEM;
	}
	return ret;
}

static int vb1943_get_sensor_id(struct vb1943 *sensor, void *pchip_id)
{
	int ret = 0;
	u16 chip_id = 1943;

	ret = copy_to_user(pchip_id, &chip_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	pr_debug("enter %s function\n", __func__);
	return ret;
}

static int vb1943_get_reserve_id(struct vb1943 *sensor, void *preserve_id)
{
	int ret = 0;
	u16 reserve_id = 1943;

	ret = copy_to_user(preserve_id, &reserve_id, sizeof(u16));
	if (ret != 0)
		ret = -ENOMEM;
	pr_debug("enter %s function\n", __func__);
	return ret;
}

static int vb1943_get_sensor_mode(struct vb1943 *sensor, void *pmode)
{
	int ret = 0;

	pr_debug("enter %s function\n", __func__);
	ret = copy_to_user(pmode, &sensor->cur_mode,
		sizeof(struct vvcam_mode_info_s));
	if (ret != 0) {
		pr_warn("error getting sensor mode %s\n", __func__);
		ret = -ENOMEM;
	}
	return ret;
}

static void vb1943_update_ctrl_val(struct v4l2_ctrl *ctrl, s32 val)
{
	if (!ctrl)
		return;

	ctrl->val = val;
	ctrl->cur.val = val;
}

/*
 * Global shutter is supported only with RAW10 and NIR only with global shutter
 */
static void vb1943_check_unsupported_mode(struct vb1943 *sensor)
{
	if (sensor->cur_mode.bit_width == 12 && sensor->shutter_mode == VB1943_GS_MODE) {
		pr_warn("%s: 12BPP & Global Shutter is not supported! Switching to Rolling Shutter\n",
			__func__);
		sensor->shutter_mode = VB1943_RS_MODE;
		sensor->rs_forced = true;
		vb1943_update_ctrl_val(sensor->ctrls.shutter_mode, VB1943_RS_MODE);
	} else if (sensor->cur_mode.bit_width == 10 && sensor->rs_forced) {
		/* back from a 12 bit mode: restore the global shutter it replaced */
		pr_info("%s: 10BPP mode, restoring Global Shutter\n", __func__);
		sensor->shutter_mode = VB1943_GS_MODE;
		sensor->rs_forced = false;
		vb1943_update_ctrl_val(sensor->ctrls.shutter_mode, VB1943_GS_MODE);
	}

	if (sensor->shutter_mode == VB1943_RS_MODE && sensor->color_mode == VB1943_NIR_MODE) {
		u32 mode = sensor->mono ? VB1943_RGBNIR_MODE : VB1943_RGB_MODE;

		pr_warn("%s: Rolling Shutter & NIR is not supported! Switching to %s\n",
			__func__, vb1943_color_mode_menu[mode]);
		sensor->color_mode = mode;
		vb1943_update_ctrl_val(sensor->ctrls.color_mode, mode);
	}
}

static u32 vb1943_calc_frame_length(struct vb1943 *sensor, u32 fps)
{
	u64 frame_length;

	frame_length = div64_u64(VB1943_G_FACTOR << 10,
			(u64)fps * sensor->cur_mode.ae_info.one_line_exp_time_ns);

	if (frame_length < sensor->min_frame_length)
		frame_length = sensor->min_frame_length;

	if (frame_length > VB1943_MAX_FRAME_LENGTH)
		frame_length = VB1943_MAX_FRAME_LENGTH;

	/* Value must be multiple of 2 */
	frame_length = (frame_length % 2) ? frame_length + 1 : frame_length;

	return (u32)frame_length;
}

static void vb1943_update_ae_info(struct vb1943 *sensor, u32 frame_length)
{
	struct vvcam_ae_info_s *ae_info = &sensor->cur_mode.ae_info;

	sensor->frame_length = frame_length;
	ae_info->curr_frm_len_lines = frame_length;
	ae_info->max_integration_line = frame_length - VB1943_INTEGRATION_LINES_MARGIN;
	ae_info->min_integration_line = VB1943_MIN_INTEGRATION_LINES;
	ae_info->cur_fps = (u32)div64_u64(VB1943_G_FACTOR << 10,
			(u64)frame_length * ae_info->one_line_exp_time_ns);
}

static void vb1943_update_timing(struct vb1943 *sensor)
{
	struct vvcam_ae_info_s *ae_info = &sensor->cur_mode.ae_info;
	u64 line_time;
	u32 fps;

	u32 line_length;

	line_length = VB1943_DEFAULT_LINE_LENGTH;
	if (sensor->num_lanes == VB1943_TWO_LANE_MODE) {
		if (sensor->cur_mode.bit_width == 12)
			line_length = VB1943_TWO_LANE_12BIT_LINE_LENGTH;
		else
			line_length = VB1943_TWO_LANE_10BIT_LINE_LENGTH;
	}

	if (sensor->data_rate_mbps < VB1943_DEFAULT_DATA_RATE_MBPS) {
		u64 needed = (u64)line_length * VB1943_DEFAULT_DATA_RATE_MBPS *
			     sensor->cur_mode.size.bounds_width;

		needed = DIV64_U64_ROUND_UP(needed,
				(u64)sensor->data_rate_mbps * VB1943_DEFAULT_WIDTH);
		line_length = max_t(u32, line_length, (u32)needed);
	}
	sensor->line_length = line_length;

	line_time = div64_u64((u64)sensor->line_length * VB1943_G_FACTOR, 4 * VB1943_PIXEL_CLK);
	ae_info->one_line_exp_time_ns = (u32)line_time;

	if (sensor->shutter_mode == VB1943_GS_MODE)
		sensor->min_frame_length = VB1943_GS_MIN_FRAME_LENGTH;
	else
		sensor->min_frame_length = VB1943_RS_MIN_FRAME_LENGTH;

	ae_info->def_frm_len_lines = sensor->min_frame_length;
	ae_info->max_fps = (u32)div64_u64(VB1943_G_FACTOR << 10,
			(u64)sensor->min_frame_length * line_time);
	/* Lowest frame rate whose frame length fits in CTX0_FRAME_LENGTH */
	ae_info->min_fps = (u32)DIV64_U64_ROUND_UP(VB1943_G_FACTOR << 10,
			(u64)VB1943_MAX_FRAME_LENGTH * line_time);
	ae_info->min_afps = max_t(u32, ae_info->min_afps, ae_info->min_fps);

	/* Use the requested frame rate, the current one may be clamped by the previous range */
	fps = sensor->target_fps ? sensor->target_fps : ae_info->cur_fps;
	fps = clamp_t(u32, fps, ae_info->min_fps, ae_info->max_fps);
	vb1943_update_ae_info(sensor, vb1943_calc_frame_length(sensor, fps));

	pr_debug("%s: lanes: %u line_length: %u line_time: %u [ns] min_frame_length: %u max_fps: %u min_fps: %u\n",
		 __func__, sensor->num_lanes, sensor->line_length, ae_info->one_line_exp_time_ns,
		 sensor->min_frame_length, ae_info->max_fps, ae_info->min_fps);
}

static int vb1943_set_sensor_mode(struct vb1943 *sensor, void *pmode)
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

	for (i = 0; i < ARRAY_SIZE(pvb1943_mode_info); i++) {
		if (pvb1943_mode_info[i].index == sensor_mode.index) {
			memcpy(&sensor->cur_mode, &pvb1943_mode_info[i],
				sizeof(struct vvcam_mode_info_s));
			sensor->cur_mode.mipi_info.mipi_lane = sensor->num_lanes;
			sensor->target_fps = 0;
			vb1943_check_unsupported_mode(sensor);
			vb1943_update_timing(sensor);
			return 0;
		}
	}
	pr_err("enter %s: Failed to set current sensor mode\n", __func__);
	return -ENXIO;
}

static int vb1943_write_frame_length(struct vb1943 *sensor, u32 frame_length)
{
	int ret;

	if (sensor->operation_mode == VB1943_SLAVE_MODE)
		frame_length = sensor->min_frame_length;

	ret = vb1943_write_reg_le(sensor, CTX0_FRAME_LENGTH, 2, frame_length);
	if (ret < 0) {
		pr_err("%s: failed to set frame length\n", __func__);
		return ret;
	}

	vb1943_update_ae_info(sensor, frame_length);
	pr_debug("%s: frame_length set: %u\n", __func__, frame_length);

	return 0;
}

static int vb1943_get_sensor_configuration(struct vb1943 *sensor, u8 *conf_reg)
{
	if (sensor->cur_mode.bit_width == 10) {
		if (sensor->shutter_mode == VB1943_GS_MODE) {
			if (sensor->color_mode == VB1943_RGBNIR_MODE)
				*conf_reg = VB1943_GS_RGB_NIR_RAW10;
			else if (sensor->color_mode == VB1943_RGB_MODE)
				*conf_reg = VB1943_GS_RGB_RAW10;
			else
				*conf_reg = VB1943_GS_NIR_RAW10;
		} else {
			if (sensor->color_mode == VB1943_RGBNIR_MODE)
				*conf_reg = VB1943_RS_RGB_NIR_RAW10;
			else if (sensor->color_mode == VB1943_RGB_MODE)
				*conf_reg = VB1943_RS_RGB_RAW10;
			else
				return -EINVAL;
		}
	} else if (sensor->cur_mode.bit_width == 12) {
		if (sensor->shutter_mode != VB1943_RS_MODE)
			return -EINVAL;

		if (sensor->color_mode == VB1943_RGBNIR_MODE)
			*conf_reg = VB1943_RS_RGB_NIR_RAW12;
		else if (sensor->color_mode == VB1943_RGB_MODE)
			*conf_reg = VB1943_RS_RGB_RAW12;
		else
			return -EINVAL;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int vb1943_set_sensor_configuration(struct vb1943 *sensor)
{
	u8 conf_reg;
	int ret;

	ret = vb1943_get_sensor_configuration(sensor, &conf_reg);
	if (ret < 0) {
		pr_err("%s: unsupported combination: %u bit, %s, %s\n", __func__,
		       sensor->cur_mode.bit_width, vb1943_shutter_mode_menu[sensor->shutter_mode],
		       vb1943_color_mode_menu[sensor->color_mode]);
		return ret;
	}

	ret = vb1943_write_reg(sensor, CTX0_SENSOR_CONFIGURATION, conf_reg);
	if (ret < 0) {
		pr_err("%s: unable to write to CTX0_SENSOR_CONFIGURATION register\n", __func__);
		return ret;
	}

	pr_debug("%s: sensor configuration: 0x%02x\n", __func__, conf_reg);

	return 0;
}

/*
 * Shutter and color mode share CTX0_SENSOR_CONFIGURATION, the frame length
 * minimum differs between global and rolling shutter
 */
static int vb1943_apply_sensor_configuration(struct vb1943 *sensor)
{
	ktime_t start = ktime_get();
	int ret, err;

	vb1943_update_timing(sensor);

	if (!sensor->powered_on)
		return 0;

	ret = vb1943_static_begin(sensor);
	if (ret < 0)
		goto restart;

	ret = vb1943_set_sensor_configuration(sensor);
	if (ret < 0)
		goto restart;

	ret = vb1943_write_frame_length(sensor, sensor->frame_length);

restart:
	err = vb1943_static_end(sensor);

	pr_info("%s: %s, %s in %lld us, err: %d\n", __func__,
		vb1943_shutter_mode_menu[sensor->shutter_mode],
		vb1943_color_mode_menu[sensor->color_mode],
		ktime_us_delta(ktime_get(), start), ret ? ret : err);

	return ret ? ret : err;
}

static int vb1943_set_shutter_mode(struct vb1943 *sensor, u32 val)
{
	u32 prev_shutter_mode = sensor->shutter_mode;
	int ret;

	pr_debug("enter %s shutter mode: %u\n", __func__, val);

	if (val > VB1943_RS_MODE)
		return -EINVAL;

	if (val == VB1943_GS_MODE && sensor->cur_mode.bit_width == 12) {
		pr_warn("%s: Global Shutter is not supported in 12BPP\n", __func__);
		return -EINVAL;
	}

	if (val == VB1943_RS_MODE && sensor->color_mode == VB1943_NIR_MODE) {
		pr_warn("%s: Rolling Shutter is not supported in NIR mode\n", __func__);
		return -EINVAL;
	}

	sensor->rs_forced = false;

	if (val == sensor->shutter_mode)
		return 0;

	sensor->shutter_mode = val;
	ret = vb1943_apply_sensor_configuration(sensor);
	if (ret < 0) {
		sensor->shutter_mode = prev_shutter_mode;
		vb1943_update_timing(sensor);
	}

	return ret;
}

static int vb1943_set_color_mode(struct vb1943 *sensor, u32 val)
{
	u32 prev_color_mode = sensor->color_mode;
	int ret;

	pr_debug("enter %s color mode: %u\n", __func__, val);

	if (val > VB1943_NIR_MODE)
		return -EINVAL;

	if (val == VB1943_NIR_MODE && sensor->shutter_mode == VB1943_RS_MODE) {
		pr_warn("%s: NIR mode is supported only with Global Shutter\n", __func__);
		return -EINVAL;
	}

	if (val == VB1943_RGB_MODE && sensor->mono) {
		pr_warn("%s: RGB mode is not supported on the mono sensor\n", __func__);
		return -EINVAL;
	}

	if (val == sensor->color_mode)
		return 0;

	sensor->color_mode = val;
	ret = vb1943_apply_sensor_configuration(sensor);
	if (ret < 0) {
		sensor->color_mode = prev_color_mode;
		vb1943_update_timing(sensor);
	}

	return ret;
}

static int vb1943_set_operation_mode(struct vb1943 *sensor, u32 val)
{
	u8 gpio_ctrl;
	int ret, err;

	pr_debug("enter %s operation mode: %u\n", __func__, val);

	if (val > VB1943_SLAVE_MODE)
		return -EINVAL;

	sensor->operation_mode = val;

	/* Control is set regardless of power state, applied in set_fmt */
	if (!sensor->powered_on)
		return 0;

	ret = vb1943_static_begin(sensor);
	if (ret < 0)
		goto restart;

	ret = vb1943_read_reg(sensor, CTX0_GPIO_CTRL, &gpio_ctrl);
	if (ret < 0) {
		pr_err("%s: error reading GPIO control\n", __func__);
		goto restart;
	}

	/*
	 * GPIO_0 is the sync pin in both modes, on the Framos adapter it is
	 * routed to XVS (TP6 on the FPA P-CON adapter), GPIO_1 is not connected.
	 * Master: GPIO_0 sends an FSYNC_OUT pulse at each start of frame.
	 * Slave: GPIO_0 is the FSYNC_IN trigger (rising edge), so the XVS of
	 * a master sensor or an external trigger source drives it.
	 */
	gpio_ctrl &= ~VB1943_GPIO1_EN;
	gpio_ctrl |= VB1943_GPIO0_EN;
	if (val == VB1943_SLAVE_MODE) {
		ret = vb1943_write_reg(sensor, GPIO_0_CTRL, VB1943_GPIO_MODE_FSYNC_IN);
		ret |= vb1943_write_reg(sensor, NB_FRAMES_TO_SEND, 0x00);
	} else {
		ret = vb1943_write_reg(sensor, GPIO_0_CTRL, VB1943_GPIO_MODE_FSYNC_OUT);
	}

	ret |= vb1943_write_reg(sensor, CTX0_GPIO_CTRL, gpio_ctrl);
	ret |= vb1943_write_reg(sensor, VT_CTRL, (val == VB1943_SLAVE_MODE) ?
				VB1943_VT_CTRL_SLAVE : VB1943_VT_CTRL_MASTER);
	/* the slave runs with the minimum frame length, see vb1943_write_frame_length */
	ret |= vb1943_write_frame_length(sensor, sensor->frame_length);
	if (ret < 0)
		pr_err("%s: error setting operation mode\n", __func__);
	else
		pr_debug("%s: Sensor is in %s mode\n", __func__,
			 vb1943_operation_mode_menu[val]);

restart:
	err = vb1943_static_end(sensor);

	return ret ? ret : err;
}

static int vb1943_set_exp(struct vb1943 *sensor, u32 exp, u8 which_control)
{
	int ret = 0;
	u64 integration_time_line;
	u32 line_time = sensor->cur_mode.ae_info.one_line_exp_time_ns;

	pr_debug("enter %s exposure received: %u control: %u\n", __func__, exp, which_control);

	if (which_control == 0) // from ISP driver, exposure in us, Q10 format
		integration_time_line = div_u64(((u64)exp * VB1943_K_FACTOR) >> 10, line_time);
	else  // from V4L2 control, exposure in us
		integration_time_line = div_u64((u64)exp * VB1943_K_FACTOR, line_time);

	if (integration_time_line > sensor->cur_mode.ae_info.max_integration_line) {
		pr_debug("%s: setting integration time to max value %u\n", __func__,
			sensor->cur_mode.ae_info.max_integration_line);
		integration_time_line = sensor->cur_mode.ae_info.max_integration_line;
	}

	if (integration_time_line < sensor->cur_mode.ae_info.min_integration_line) {
		pr_debug("%s: setting integration time to min value %u\n", __func__,
			sensor->cur_mode.ae_info.min_integration_line);
		integration_time_line = sensor->cur_mode.ae_info.min_integration_line;
	}

	pr_debug("%s: integration_time_line: %llu\n", __func__, integration_time_line);
	ret = vb1943_group_hold(sensor, 1);
	ret |= vb1943_write_reg_le(sensor, CTX0_INTEGRATION_TIME_PRIMARY, 2, (u32)integration_time_line);
	ret |= vb1943_group_hold(sensor, 0);

	if (ret < 0)
		pr_err("%s Failed to set exposure exp: %u, integration time: %llu\n",
		       __func__, exp, integration_time_line);

	return ret;
}

/* Find the largest analog gain step, which is not larger than the given gain */
static int vb1943_get_again_index(u32 gain_x100)
{
	int i;

	for (i = ARRAY_SIZE(vb1943_again_lut) - 1; i > 0; i--) {
		if (gain_x100 >= vb1943_again_lut[i].gain_x100)
			break;
	}

	return i;
}

static int vb1943_write_gain(struct vb1943 *sensor, u8 again_reg, u32 dgain_reg)
{
	int ret;

	pr_debug("%s: analog gain register: 0x%02x digital gain register: 0x%04x\n",
		 __func__, again_reg, dgain_reg);

	ret = vb1943_group_hold(sensor, 1);
	ret |= vb1943_write_reg(sensor, CTX0_ANALOG_GAIN, again_reg);
	ret |= vb1943_write_reg_le(sensor, CTX0_DIGITAL_GAIN_R, 2, dgain_reg);
	ret |= vb1943_write_reg_le(sensor, CTX0_DIGITAL_GAIN_G, 2, dgain_reg);
	ret |= vb1943_write_reg_le(sensor, CTX0_DIGITAL_GAIN_B, 2, dgain_reg);
	ret |= vb1943_write_reg_le(sensor, CTX0_DIGITAL_GAIN_IR, 2, dgain_reg);
	ret |= vb1943_group_hold(sensor, 0);

	if (ret < 0)
		pr_err("%s: error setting gain\n", __func__);

	return ret;
}

static int vb1943_set_gain(struct vb1943 *sensor, u32 gain)
{
	u32 gain_x100;
	u32 dgain_reg;
	int idx;

	pr_debug("enter %s: gain received: %u\n", __func__, gain);

	gain = clamp_t(u32, gain, 1024, VB1943_MAX_ISP_GAIN);
	gain_x100 = (gain * 100) >> 10;

	idx = vb1943_get_again_index(gain_x100);

	/* digital gain = gain / analog gain, in u8.8 format */
	dgain_reg = DIV_ROUND_CLOSEST(gain * 25, vb1943_again_lut[idx].gain_x100);
	dgain_reg = min_t(u32, dgain_reg, VB1943_MAX_DGAIN_DEC * VB1943_DGAIN_ONE / 100);

	return vb1943_write_gain(sensor, vb1943_again_lut[idx].reg_value, dgain_reg);
}

static int vb1943_set_again(struct vb1943 *sensor, u32 val)
{
	int idx;
	int ret;

	pr_debug("enter %s: analog gain received: %u\n", __func__, val);

	val = clamp_t(u32, val, VB1943_MIN_AGAIN_DEC, VB1943_MAX_AGAIN_DEC);
	idx = vb1943_get_again_index(val);

	ret = vb1943_group_hold(sensor, 1);
	ret |= vb1943_write_reg(sensor, CTX0_ANALOG_GAIN, vb1943_again_lut[idx].reg_value);
	ret |= vb1943_group_hold(sensor, 0);
	if (ret < 0) {
		pr_err("%s: error setting analog gain\n", __func__);
		return ret;
	}

	vb1943_update_ctrl_val(sensor->ctrls.gain, vb1943_again_lut[idx].gain_x100);

	return 0;
}

static int vb1943_set_dgain(struct vb1943 *sensor, u32 val)
{
	u32 dgain_reg;
	int ret;

	pr_debug("enter %s: digital gain received: %u\n", __func__, val);

	val = clamp_t(u32, val, VB1943_MIN_DGAIN_DEC, VB1943_MAX_DGAIN_DEC);
	dgain_reg = DIV_ROUND_CLOSEST(val * VB1943_DGAIN_ONE, 100);

	ret = vb1943_group_hold(sensor, 1);
	ret |= vb1943_write_reg_le(sensor, CTX0_DIGITAL_GAIN_R, 2, dgain_reg);
	ret |= vb1943_write_reg_le(sensor, CTX0_DIGITAL_GAIN_G, 2, dgain_reg);
	ret |= vb1943_write_reg_le(sensor, CTX0_DIGITAL_GAIN_B, 2, dgain_reg);
	ret |= vb1943_write_reg_le(sensor, CTX0_DIGITAL_GAIN_IR, 2, dgain_reg);
	ret |= vb1943_group_hold(sensor, 0);
	if (ret < 0)
		pr_err("%s: error setting digital gain\n", __func__);

	return ret;
}

static int vb1943_write_black_level(struct vb1943 *sensor)
{
	int ret;

	ret = vb1943_write_reg_le(sensor, CTX0_DARKCAL_PEDESTAL, 2, sensor->black_level);
	if (ret < 0)
		pr_err("%s: BLACK LEVEL control error\n", __func__);

	return ret;
}

static int vb1943_set_black_level(struct vb1943 *sensor, u32 val)
{
	int ret, err;

	pr_debug("enter %s black level: %u\n", __func__, val);

	sensor->black_level = min_t(u32, val, VB1943_MAX_BLACK_LEVEL);

	ret = vb1943_static_begin(sensor);
	if (!ret)
		ret = vb1943_write_black_level(sensor);
	err = vb1943_static_end(sensor);

	return ret ? ret : err;
}

static int vb1943_set_fps(struct vb1943 *sensor, u32 fps, u8 which_control)
{
	u32 frame_length;
	ktime_t start;
	int ret, err;

	pr_debug("enter %s fps received: %u\n", __func__, fps);
	if (which_control == 1)
		fps = fps << 10;

	sensor->target_fps = fps;

	if (fps > sensor->cur_mode.ae_info.max_fps) {
		pr_debug("fps %u too large setting to %u\n", fps, sensor->cur_mode.ae_info.max_fps);
		fps = sensor->cur_mode.ae_info.max_fps;
	} else if (fps < sensor->cur_mode.ae_info.min_fps) {
		fps = sensor->cur_mode.ae_info.min_fps;
	}

	frame_length = vb1943_calc_frame_length(sensor, fps);

	if (!sensor->stream_status)
		return vb1943_write_frame_length(sensor, frame_length);

	if (frame_length == sensor->frame_length)
		return 0;

	start = ktime_get();

	ret = vb1943_enter_sw_standby(sensor);
	if (!ret)
		ret = vb1943_write_frame_length(sensor, frame_length);
	err = vb1943_exit_sw_standby(sensor);

	pr_debug("%s: frame_length changed on the fly in %lld us, err: %d\n",
		 __func__, ktime_us_delta(ktime_get(), start), ret ? ret : err);

	return ret ? ret : err;
}

static int vb1943_get_fps(struct vb1943 *sensor, u32 *pfps)
{
	pr_debug("enter %s function\n", __func__);
	*pfps = sensor->cur_mode.ae_info.cur_fps;
	return 0;
}

static int vb1943_write_test_pattern(struct vb1943 *sensor)
{
	u32 pattern = sensor->test_pattern;
	int ret;

	if (pattern > 0 && pattern < ARRAY_SIZE(vb1943_test_pattern_menu)) {
		if (!sensor->duster_ctrl)
			vb1943_read_reg(sensor, CTX0_DUSTER_CTRL, &sensor->duster_ctrl);
		ret = vb1943_write_reg(sensor, DARKCAL_ENABLE, VB1943_DARKCAL_OFF);
		ret |= vb1943_write_reg(sensor, CTX0_DUSTER_CTRL, VB1943_DUSTER_OFF);
		ret |= vb1943_write_reg_arry(sensor,
					     mode_enable_pattern_generator,
					     ARRAY_SIZE(mode_enable_pattern_generator));
		ret |= vb1943_write_reg(sensor, PATGEN_CTRL + 1, (pattern == 1) ? 0x22 : 0x28);
	} else {
		ret = vb1943_write_reg_arry(sensor,
					    mode_disable_pattern_generator,
					    ARRAY_SIZE(mode_disable_pattern_generator));
		if (sensor->duster_ctrl) {
			ret |= vb1943_write_reg(sensor, DARKCAL_ENABLE, VB1943_DARKCAL_ON);
			ret |= vb1943_write_reg(sensor, CTX0_DUSTER_CTRL, sensor->duster_ctrl);
		}
	}

	if (ret < 0)
		pr_err("%s: error setting Test Pattern\n", __func__);

	return ret;
}

static int vb1943_set_test_pattern(struct vb1943 *sensor, u32 pattern)
{
	int ret, err;

	pr_debug("enter %s pattern: %u\n", __func__, pattern);

	if (pattern >= ARRAY_SIZE(vb1943_test_pattern_menu))
		return -EINVAL;

	sensor->test_pattern = pattern;

	ret = vb1943_static_begin(sensor);
	if (!ret)
		ret = vb1943_write_test_pattern(sensor);
	err = vb1943_static_end(sensor);

	return ret ? ret : err;
}

static int vb1943_read_temperature(struct vb1943 *sensor, s32 *temperature)
{
	u32 reg_temp;
	int ret;

	ret = vb1943_read_reg_le(sensor, TEMPERATURE_1, 2, &reg_temp);
	if (ret < 0) {
		pr_err("%s: error reading TEMPERATURE_1 register\n", __func__);
		return ret;
	}

	*temperature = (s16)reg_temp;
	pr_debug("%s: TEMPERATURE_1 value: %d\n", __func__, *temperature);

	return 0;
}

static int vb1943_apply_format(struct vb1943 *sensor);

static bool vb1943_data_rate_allowed(struct vb1943 *sensor, u32 rate)
{
	return rate <= sensor->dt_data_rate_mbps &&
	       2 * rate >= sensor->dt_data_rate_mbps;
}

static int vb1943_set_data_rate(struct vb1943 *sensor, u32 idx)
{
	u32 prev_rate = sensor->data_rate_mbps;
	u32 was_streaming;
	u32 rate;
	int ret;

	if (idx >= ARRAY_SIZE(vb1943_data_rates_mbps))
		return -EINVAL;

	rate = vb1943_data_rates_mbps[idx];
	if (!vb1943_data_rate_allowed(sensor, rate)) {
		pr_warn("%s: %u Mbps is outside %u..%u Mbps (DT rate and half of it)\n",
			__func__, rate, sensor->dt_data_rate_mbps / 2, sensor->dt_data_rate_mbps);
		return -EINVAL;
	}

	if (rate == sensor->data_rate_mbps)
		return 0;

	if (!strcmp(sensor->gmsl, "gmsl"))
		return -EINVAL;

	sensor->data_rate_mbps = rate;

	if (!sensor->powered_on) {
		vb1943_update_timing(sensor);
		return 0;
	}

	was_streaming = sensor->stream_status;
	if (was_streaming) {
		vb1943_enter_sw_standby(sensor);
		sensor->stream_status = 0;
	}

	__vb1943_power_off(sensor);
	ret = __vb1943_power_on(sensor);
	if (ret < 0) {
		pr_err("%s: sensor reboot failed\n", __func__);
		sensor->data_rate_mbps = prev_rate;
		vb1943_update_timing(sensor);
		return ret;
	}

	ret = vb1943_apply_format(sensor);
	ret |= vb1943_set_again(sensor, sensor->ctrls.gain->val);
	ret |= vb1943_set_dgain(sensor, sensor->ctrls.digital_gain->val);
	ret |= vb1943_set_exp(sensor, sensor->ctrls.exposure->val, 1);

	if (!ret && was_streaming) {
		ret = vb1943_exit_sw_standby(sensor);
		if (!ret)
			sensor->stream_status = 1;
	}

	pr_info("%s: %u Mbps/lane, line length %u, err %d\n",
		__func__, rate, sensor->line_length, ret);

	return ret;
}

static int vb1943_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vb1943 *sensor = to_vb1943_dev(sd);
	s32 temperature = 0;
	int ret = 0;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_TEMPERATURE:
		if (sensor->powered_on == 0)
			return -ENODEV;
		ret = vb1943_read_temperature(sensor, &temperature);
		ctrl->val = temperature;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int vb1943_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct vb1943 *sensor = to_vb1943_dev(sd);
	int ret;

	/* v4l2_ctrl_lock() locks our own mutex */

	switch (ctrl->id) {
	case V4L2_CID_SHUTTER_MODE:
		return vb1943_set_shutter_mode(sensor, ctrl->val);
	case V4L2_CID_COLOR_MODE:
		return vb1943_set_color_mode(sensor, ctrl->val);
	case V4L2_CID_OPERATION_MODE:
		return vb1943_set_operation_mode(sensor, ctrl->val);
	case V4L2_CID_DATA_RATE:
		return vb1943_set_data_rate(sensor, ctrl->val);
	default:
		break;
	}

	/*
	 * If the device is not powered up by the host driver do
	 * not apply any controls to H/W at this time. Instead
	 * the controls will be restored right after power-up.
	 */
	if (sensor->powered_on == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		ret = vb1943_set_again(sensor, ctrl->val);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = vb1943_set_dgain(sensor, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = vb1943_set_exp(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = vb1943_set_test_pattern(sensor, ctrl->val);
		break;
	case V4L2_CID_FRAME_RATE:
		ret = vb1943_set_fps(sensor, ctrl->val, 1);
		break;
	case V4L2_CID_BLACK_LEVEL:
		ret = vb1943_set_black_level(sensor, ctrl->val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops vb1943_ctrl_ops = {
	.g_volatile_ctrl = vb1943_g_volatile_ctrl,
	.s_ctrl = vb1943_s_ctrl,
};

static int vb1943_get_format_code(struct vb1943 *sensor, u32 *code)
{
	pr_debug("enter %s function\n", __func__);
	switch (sensor->cur_mode.bayer_pattern) {
	case BAYER_RGGB:
		if (sensor->cur_mode.bit_width == 10)
			*code = MEDIA_BUS_FMT_SRGGB10_1X10;
		else
			*code = MEDIA_BUS_FMT_SRGGB12_1X12;
		break;
	case BAYER_GBRG:
		if (sensor->cur_mode.bit_width == 10)
			*code = MEDIA_BUS_FMT_SGBRG10_1X10;
		else
			*code = MEDIA_BUS_FMT_SGBRG12_1X12;
		break;
	default:
		/*nothing need to do*/
		break;
	}
	return 0;
}

static u32 vb1943_parse_num_lanes(struct device_node *node)
{
	struct device_node *ep;
	int num_lanes;

	ep = of_graph_get_next_endpoint(node, NULL);
	if (!ep)
		return VB1943_FOUR_LANE_MODE;

	num_lanes = of_property_count_u32_elems(ep, "data-lanes");
	of_node_put(ep);

	if (num_lanes == VB1943_TWO_LANE_MODE)
		return VB1943_TWO_LANE_MODE;

	if (num_lanes != VB1943_FOUR_LANE_MODE)
		pr_warn("%s: unsupported number of data-lanes: %d, using 4 lanes\n",
			__func__, num_lanes);

	return VB1943_FOUR_LANE_MODE;
}

static u32 vb1943_parse_data_rate(struct device_node *node)
{
	struct device_node *ep;
	u64 link_freq;
	u32 rate = VB1943_DEFAULT_DATA_RATE_MBPS;

	if (data_rate_mbps)
		return data_rate_mbps;

	ep = of_graph_get_next_endpoint(node, NULL);
	if (!ep)
		return rate;

	if (!of_property_read_u64(ep, "link-frequencies", &link_freq))
		rate = div_u64(link_freq * 2, 1000000);
	of_node_put(ep);

	if (rate < 80 || rate > 1500) {
		pr_warn("%s: unsupported data rate %u Mbps, using %u Mbps\n",
			__func__, rate, VB1943_DEFAULT_DATA_RATE_MBPS);
		rate = VB1943_DEFAULT_DATA_RATE_MBPS;
	}

	pr_info("%s: MIPI data rate %u Mbps per lane\n", __func__, rate);

	return rate;
}

static int vb1943_parse_dt(struct vb1943 *sensor, struct i2c_client *client)
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

	match = of_match_device(vb1943_of_match, &client->dev);
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
	} else {
		pr_warn("unknown gmsl property %s, initializing mipi...\n", gmsl);
		sensor->gmsl = "mipi";
	}

	sensor->num_lanes = vb1943_parse_num_lanes(node);
	sensor->data_rate_mbps = vb1943_parse_data_rate(node);
	sensor->dt_data_rate_mbps = sensor->data_rate_mbps;
	pr_debug("%s: Succesfully parsed device tree, %u lanes\n", __func__, sensor->num_lanes);

	return 0;
}

static int vb1943_set_pixel_format(struct vb1943 *sensor)
{
	int err = 0;

	if (sensor->cur_mode.bit_width == 10)
		err = vb1943_write_reg_arry(sensor, vb1943_10bit_mode,
					    ARRAY_SIZE(vb1943_10bit_mode));
	else if (sensor->cur_mode.bit_width == 12)
		err = vb1943_write_reg_arry(sensor, vb1943_12bit_mode,
					    ARRAY_SIZE(vb1943_12bit_mode));
	else {
		pr_err("%s: unknown pixel format\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int vb1943_set_resolution(struct vb1943 *sensor)
{
	int ret;

	switch (sensor->cur_mode.index) {
	case VB1943_ALL_PIXEL_10BIT_INDEX:
	case VB1943_ALL_PIXEL_12BIT_INDEX:
		ret = vb1943_write_reg_arry(sensor, vb1943_setting_all_pixel,
					    ARRAY_SIZE(vb1943_setting_all_pixel));
		break;
	case VB1943_CROP_2064x1552_10BIT_INDEX:
	case VB1943_CROP_2064x1552_12BIT_INDEX:
		ret = vb1943_write_reg_arry(sensor, vb1943_setting_crop_2064x1552,
					    ARRAY_SIZE(vb1943_setting_crop_2064x1552));
		break;
	case VB1943_CROP_1920x1080_10BIT_INDEX:
	case VB1943_CROP_1920x1080_12BIT_INDEX:
		ret = vb1943_write_reg_arry(sensor, vb1943_setting_crop_1920x1080,
					    ARRAY_SIZE(vb1943_setting_crop_1920x1080));
		break;
	default:
		pr_err("%s:Invalid mode\n", __func__);
		return -EINVAL;
	}

	pr_info("%s:Setting mode %u ", __func__, sensor->cur_mode.index);

	return ret;
}

static int vb1943_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vb1943 *sensor = client_to_vb1943(client);
	int err = 0;

	pr_debug("enter function %s\n", __func__);
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
		err = vb1943_exit_sw_standby(sensor);
		if (err < 0)
			goto exit;
		sensor->stream_status = 1;
	} else  {
		pr_info("Disable stream\n");
		if (!(strcmp(sensor->gmsl, "gmsl"))) {
			/* disable serdes streaming */
			max96792_stop_streaming(sensor->dser_dev, &sensor->i2c_client->dev);
		}
		sensor->stream_status = 0;
		err = vb1943_enter_sw_standby(sensor);
		if (err < 0)
			goto exit;
	}

	return 0;
exit:
	pr_err("%s: error setting stream\n", __func__);

	return err;
}

static int vb1943_gmsl_serdes_setup(struct vb1943 *priv)
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
		pr_err("deserializer gmsl setup failed\n");
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
	if (des_err)
		pr_err("gmsl deserializer setup failed\n");

error:
	mutex_unlock(&priv->lock);
	return err;
}

static void vb1943_gmsl_serdes_reset(struct vb1943 *priv)
{
	mutex_lock(&priv->lock);

	/* reset serdes addressing and control pipeline */
	max96793_reset_control(priv->ser_dev);
	max96792_reset_control(priv->dser_dev, &priv->i2c_client->dev);

	max96792_power_off(priv->dser_dev, &priv->g_ctx);

	mutex_unlock(&priv->lock);
}

static int vb1943_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vb1943 *sensor = client_to_vb1943(client);
	u32 cur_code = MEDIA_BUS_FMT_SGBRG10_1X10;

	pr_debug("enter %s function\n", __func__);
	if (code->index > 0)
		return -EINVAL;
	vb1943_get_format_code(sensor, &cur_code);
	code->code = cur_code;

	return 0;
}

static int vb1943_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vb1943 *sensor = client_to_vb1943(client);

	mutex_lock(&sensor->lock);

	pr_debug("enter %s function\n", __func__);
	if ((fmt->format.width != sensor->cur_mode.size.bounds_width) ||
		(fmt->format.height != sensor->cur_mode.size.bounds_height)) {
		pr_err("%s:set sensor format %dx%d error\n",
			__func__, fmt->format.width, fmt->format.height);
		ret = -EINVAL;
		goto exit;
	}
	vb1943_get_format_code(sensor, &fmt->format.code);
	fmt->format.field = V4L2_FIELD_NONE;
	sensor->format = fmt->format;

	ret = vb1943_apply_format(sensor);
exit:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int vb1943_apply_format(struct vb1943 *sensor)
{
	int ret;

	vb1943_check_unsupported_mode(sensor);
	vb1943_update_timing(sensor);

	ret = vb1943_write_reg_arry(sensor,
		(struct vvcam_sccb_data_s *)sensor->cur_mode.preg_data,
		sensor->cur_mode.reg_data_count);
	if (ret < 0) {
		pr_err("%s:vb1943_write_reg_arry error, error when setting initial data\n", __func__);
		goto exit;
	}

	ret = vb1943_set_pixel_format(sensor);
	if (ret < 0) {
		pr_err("%s:vb1943_write_reg_arry error, failed to set pixel format\n", __func__);
		goto exit;
	}

	ret = vb1943_set_sensor_configuration(sensor);
	if (ret < 0) {
		pr_err("%s:unable to set sensor configuration\n", __func__);
		goto exit;
	}

	/*
	 * IR removal of the CFA blender (RGB output): "damper" fades the IR
	 * subtraction with exposure. With the default fixed alphas, dim IR-rich
	 * light at high gain gets a strong green tint (G/R 2.3 vs 1.2 measured),
	 * in normal light both give the same colours.
	 */
	ret = vb1943_write_reg(sensor, CFA_BLENDER_ALPHA_CTRL, VB1943_CFA_ALPHA_DAMPER);
	if (ret < 0) {
		pr_err("%s:unable to set CFA blender alpha control\n", __func__);
		goto exit;
	}

	ret = vb1943_set_resolution(sensor);
	if (ret < 0) {
		pr_err("%s:Failed to initialize settings for mode %u\n", __func__,
		       sensor->cur_mode.index);
		goto exit;
	}

	ret = vb1943_set_operation_mode(sensor, sensor->operation_mode);
	if (ret < 0) {
		pr_err("%s:unable to set operation mode\n", __func__);
		goto exit;
	}

	ret = vb1943_write_test_pattern(sensor);
	if (ret < 0) {
		pr_err("%s:unable to set test pattern\n", __func__);
		goto exit;
	}

	ret = vb1943_write_black_level(sensor);
	if (ret < 0) {
		pr_err("%s:unable to set black level\n", __func__);
		goto exit;
	}

	ret = vb1943_write_reg_le(sensor, LINE_LENGTH, 2, sensor->line_length);
	if (ret < 0) {
		pr_err("%s:unable to write LINE_LENGTH register\n", __func__);
		goto exit;
	}

	ret = vb1943_write_frame_length(sensor, sensor->frame_length);
	if (ret < 0) {
		pr_err("%s:unable to set frame length\n", __func__);
		goto exit;
	}

exit:
	return ret;
}

static int vb1943_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vb1943 *sensor = client_to_vb1943(client);

	pr_debug("enter %s function\n", __func__);
	mutex_lock(&sensor->lock);
	fmt->format = sensor->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

static long vb1943_priv_ioctl(struct v4l2_subdev *sd,
				unsigned int cmd,
				void *arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct vb1943 *sensor = client_to_vb1943(client);
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
		ret = vb1943_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		ret = vb1943_query_supports(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = vb1943_get_sensor_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID:
		ret = vb1943_get_reserve_id(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = vb1943_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = vb1943_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		ret = vb1943_s_stream(&sensor->sd, *(int *)arg);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= vb1943_write_reg(sensor, sensor_reg.addr,
			sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= vb1943_read_reg(sensor, sensor_reg.addr,
			(u8 *)&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg,
			sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_S_LONG_EXP:
		ret = 0;
		break;
	case VVSENSORIOC_S_EXP:
		ret = vb1943_set_exp(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_S_VSEXP:
		ret = 0;
		break;
	case VVSENSORIOC_S_LONG_GAIN:
		ret = 0;
		break;
	case VVSENSORIOC_S_GAIN:
		ret = vb1943_set_gain(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_VSGAIN:
		ret = 0;
		break;
	case VVSENSORIOC_S_FPS:
		ret = vb1943_set_fps(sensor, *(u32 *)arg, 0);
		break;
	case VVSENSORIOC_G_FPS:
		ret = vb1943_get_fps(sensor, (u32 *)arg);
		break;
	case VVSENSORIOC_S_HDR_RADIO:
		ret = 0;
		break;
	case VVSENSORIOC_S_BLC:
		ret = 0;
		break;
	case VVSENSORIOC_S_WB:
		ret = 0;
		break;
	case VVSENSORIOC_G_EXPAND_CURVE:
		ret = 0;
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		ret = vb1943_set_test_pattern(sensor, *(u32 *)arg);
		if (!ret)
			vb1943_update_ctrl_val(sensor->ctrls.test_pattern, sensor->test_pattern);
		break;
	case VVSENSORIOC_S_SHUTTER_MODE:
		ret = vb1943_set_shutter_mode(sensor, *(u32 *)arg);
		if (!ret)
			vb1943_update_ctrl_val(sensor->ctrls.shutter_mode, sensor->shutter_mode);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static const struct v4l2_subdev_video_ops vb1943_subdev_video_ops = {
	.s_stream = vb1943_s_stream,
};

static const struct v4l2_subdev_pad_ops vb1943_subdev_pad_ops = {
	.enum_mbus_code = vb1943_enum_mbus_code,
	.set_fmt = vb1943_set_fmt,
	.get_fmt = vb1943_get_fmt,
};

static const struct v4l2_subdev_core_ops vb1943_subdev_core_ops = {
	.s_power = vb1943_s_power,
	.ioctl = vb1943_priv_ioctl,
};

static const struct v4l2_subdev_ops vb1943_subdev_ops = {
	.core  = &vb1943_subdev_core_ops,
	.video = &vb1943_subdev_video_ops,
	.pad   = &vb1943_subdev_pad_ops,
};

static int vb1943_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations vb1943_sd_media_ops = {
	.link_setup = vb1943_link_setup,
};

static int vb1943_communication_verify(struct vb1943 *sensor)
{
	u32 model_id, revision;
	u8 fsm_state, cfa;
	int ret;

	ret = vb1943_read_reg_le(sensor, DEVICE_MODEL_ID, 4, &model_id);
	ret |= vb1943_read_reg_le(sensor, DEVICE_REVISION, 4, &revision);
	ret |= vb1943_read_reg(sensor, SYSTEM_FSM_STATE, &fsm_state);
	ret |= vb1943_read_reg(sensor, CFA_SELECTION, &cfa);
	if (ret < 0) {
		pr_err("%s: unable to communicate with sensor\n", __func__);
		return ret;
	}

	/* CFA_SELECTION: 0 = RGB-IR color filter array, 1 = mono */
	sensor->mono = (cfa == VB1943_CFA_MONO);

	pr_info("%s: model id: 0x%08x revision: 0x%08x FSM: 0x%02x CFA: %s\n",
		__func__, model_id, revision, fsm_state,
		sensor->mono ? "mono" : "RGB-IR");

	return 0;
}

static int vb1943_probe(struct i2c_client *client)
{
	int retval;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct vb1943 *sensor;
	struct v4l2_ctrl_config color_mode_cfg;
	struct v4l2_ctrl_config data_rate_cfg;

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

	err = vb1943_parse_dt(sensor, client);
	if (err < 0) {
		pr_err("could not parse dt\n");
		return err;
	}

	mutex_init(&sensor->lock);

	sensor->i2c_client = client;
	sensor->shutter_mode = VB1943_GS_MODE;
	sensor->color_mode = VB1943_RGB_MODE;
	sensor->operation_mode = VB1943_MASTER_MODE;
	sensor->black_level = VB1943_DEFAULT_BLACK_LEVEL;

	if (strcmp(sensor->gmsl, "gmsl")) {
		sensor->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
		if (!gpio_is_valid(sensor->rst_gpio))
			dev_warn(dev, "No sensor reset pin available");
		else {
			retval = devm_gpio_request_one(dev,
							sensor->rst_gpio,
							GPIOF_OUT_INIT_LOW,
							"vb1943_mipi_reset");
			if (retval < 0)
				dev_warn(dev, "Failed to set reset pin\n");
		}

		sensor->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
		if (gpio_is_valid(sensor->pwn_gpio)) {
			retval = devm_gpio_request_one(dev,
							sensor->pwn_gpio,
							GPIOF_OUT_INIT_LOW,
							"vb1943_mipi_pwn");
			if (retval < 0) {
				dev_err(dev, "Failed to request pwn pin\n");
				return retval;
			}
		}
	}

	if (strcmp(sensor->gmsl, "gmsl")) {
		sensor->mclk = devm_clk_get_optional(dev, "csi_mclk");
		if (IS_ERR(sensor->mclk))
			return dev_err_probe(dev, PTR_ERR(sensor->mclk), "failed to get csi_mclk\n");
		if (!sensor->mclk) {
			dev_warn(dev, "No csi_mclk clock, sensor needs external clock\n");
		} else {
			retval = clk_set_rate(sensor->mclk, VB1943_XCLK_FREQ);
			if (retval < 0)
				dev_warn(dev, "Failed to set csi_mclk to %d Hz\n", VB1943_XCLK_FREQ);
			dev_info(dev, "csi_mclk: %lu Hz\n", clk_get_rate(sensor->mclk));
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
			return -EINVAL;
		}

		err = of_property_read_u32(ser_node, "reg", &sensor->g_ctx.ser_reg);
		if (err < 0) {
			dev_err(dev, "serializer reg not found\n");
			of_node_put(ser_node);
			return err;
		}

		ser_i2c = of_find_i2c_device_by_node(ser_node);
		of_node_put(ser_node);

		if (ser_i2c == NULL) {
			dev_err(dev, "missing serializer dev handle\n");
			return -EPROBE_DEFER;
		}
		if (ser_i2c->dev.driver == NULL) {
			dev_err(dev, "missing serializer driver\n");
			return -EPROBE_DEFER;
		}

		sensor->ser_dev = &ser_i2c->dev;

		dser_node = of_parse_phandle(node, "gmsl-dser-device", 0);
		if (dser_node == NULL) {
			dev_err(dev,
				"missing %s handle\n",
					"gmsl-dser-device");
			return -EINVAL;
		}

		dser_i2c = of_find_i2c_device_by_node(dser_node);
		of_node_put(dser_node);

		if (dser_i2c == NULL) {
			dev_err(dev, "missing deserializer dev handle\n");
			return -EPROBE_DEFER;
		}
		if (dser_i2c->dev.driver == NULL) {
			dev_err(dev, "missing deserializer driver\n");
			return -EPROBE_DEFER;
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
			return -EINVAL;
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
		/* Sensor side lanes follow the serializer CSI input */
		sensor->num_lanes = (value == VB1943_TWO_LANE_MODE) ?
					VB1943_TWO_LANE_MODE : VB1943_FOUR_LANE_MODE;

		sensor->g_ctx.num_streams =
				of_property_count_strings(gmsl, "streams");
		if (sensor->g_ctx.num_streams <= 0 || sensor->g_ctx.num_streams > 2) {
			dev_err(dev, "No streams found\n");
			err = -EINVAL;
			return err;
		}

		for (i = 0; i < sensor->g_ctx.num_streams; i++) {
			of_property_read_string_index(gmsl, "streams", i,
							&str_value1[i]);
			if (!str_value1[i]) {
				dev_err(dev, "invalid stream info\n");
				return -EINVAL;
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
				return -EINVAL;
			}
		}

		sensor->g_ctx.s_dev = dev;

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
		err = vb1943_gmsl_serdes_setup(sensor);
		if (err) {
			dev_err(dev, "%s gmsl serdes setup failed\n", __func__);
			return err;
		}
	}

	retval = vb1943_power_on(sensor);
	if (retval < 0) {
		dev_err(dev, "%s: sensor power on fail\n", __func__);
		goto probe_err_power_off;
	}

	retval = vb1943_communication_verify(sensor);
	if (retval < 0) {
		dev_err(dev, "%s: unable to communicate with sensor\n", __func__);
		goto probe_err_power_off;
	}

	if (sensor->mono)
		sensor->color_mode = VB1943_RGBNIR_MODE;

	sd = &sensor->sd;
	v4l2_i2c_subdev_init(sd, client, &vb1943_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.ops = &vb1943_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[VB1943_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	retval = media_entity_pads_init(&sd->entity,
				VB1943_SENS_PADS_NUM,
				sensor->pads);
	if (retval < 0)
		goto probe_err_power_off;

	memcpy(&sensor->cur_mode, &pvb1943_mode_info[0],
		sizeof(struct vvcam_mode_info_s));
	sensor->cur_mode.mipi_info.mipi_lane = sensor->num_lanes;
	vb1943_update_timing(sensor);

	/* initialize controls */
	retval = v4l2_ctrl_handler_init(&sensor->ctrls.handler, V4L2_NUM_CTRLS);
	if (retval < 0) {
		dev_err(&client->dev,
			"%s : ctrl handler init Failed\n", __func__);
		goto probe_err_free_entiny;
	}

	sensor->ctrls.handler.lock = &sensor->lock;

	// add new controls
	sensor->ctrls.exposure = v4l2_ctrl_new_std(&sensor->ctrls.handler, &vb1943_ctrl_ops, V4L2_CID_EXPOSURE,
						36, 660000, 1, 10000);
	sensor->ctrls.gain = v4l2_ctrl_new_std(&sensor->ctrls.handler, &vb1943_ctrl_ops, V4L2_CID_GAIN,
						VB1943_MIN_AGAIN_DEC, VB1943_MAX_AGAIN_DEC, 1, VB1943_MIN_AGAIN_DEC);
	sensor->ctrls.digital_gain = v4l2_ctrl_new_std(&sensor->ctrls.handler, &vb1943_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
						VB1943_MIN_DGAIN_DEC, VB1943_MAX_DGAIN_DEC, 1, VB1943_DEF_DGAIN_DEC);
	sensor->ctrls.black_level = v4l2_ctrl_new_std(&sensor->ctrls.handler, &vb1943_ctrl_ops, V4L2_CID_BLACK_LEVEL,
						0, VB1943_MAX_BLACK_LEVEL, 1, VB1943_DEFAULT_BLACK_LEVEL);

	sensor->ctrls.framerate = v4l2_ctrl_new_custom(&sensor->ctrls.handler, vb1943_ctrl_framerate, NULL);
	sensor->ctrls.shutter_mode = v4l2_ctrl_new_custom(&sensor->ctrls.handler, vb1943_ctrl_shutter_mode, NULL);
	color_mode_cfg = vb1943_ctrl_color_mode[0];
	color_mode_cfg.def = sensor->color_mode;
	if (sensor->mono)
		color_mode_cfg.menu_skip_mask = BIT(VB1943_RGB_MODE);
	sensor->ctrls.color_mode = v4l2_ctrl_new_custom(&sensor->ctrls.handler, &color_mode_cfg, NULL);
	sensor->ctrls.operation_mode = v4l2_ctrl_new_custom(&sensor->ctrls.handler, vb1943_ctrl_operation_mode, NULL);
	sensor->ctrls.temperature = v4l2_ctrl_new_custom(&sensor->ctrls.handler, vb1943_ctrl_temperature, NULL);

	/*
	 * Data rate menu, P-CON only (over GMSL the sensor cannot be reset to
	 * apply it). Only rates from the DT rate down to half of it are offered.
	 */
	data_rate_cfg = vb1943_ctrl_data_rate[0];
	data_rate_cfg.def = -1;
	for (i = 0; i < ARRAY_SIZE(vb1943_data_rates_mbps); i++) {
		if (!vb1943_data_rate_allowed(sensor, vb1943_data_rates_mbps[i]))
			data_rate_cfg.menu_skip_mask |= BIT(i);
		if (vb1943_data_rates_mbps[i] == sensor->data_rate_mbps)
			data_rate_cfg.def = i;
	}
	if (strcmp(sensor->gmsl, "gmsl") && data_rate_cfg.def >= 0) {
		data_rate_cfg.min = __ffs(~data_rate_cfg.menu_skip_mask);
		for (i = data_rate_cfg.max; i > data_rate_cfg.min; i--)
			if (!(data_rate_cfg.menu_skip_mask & BIT(i)))
				break;
		data_rate_cfg.max = i;
		sensor->ctrls.data_rate = v4l2_ctrl_new_custom(&sensor->ctrls.handler, &data_rate_cfg, NULL);
	} else {
		dev_info(&client->dev, "data rate control not available (%s, %u Mbps)\n",
			 sensor->gmsl, sensor->data_rate_mbps);
	}

	sensor->ctrls.test_pattern = v4l2_ctrl_new_std_menu_items(&sensor->ctrls.handler, &vb1943_ctrl_ops, V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(vb1943_test_pattern_menu) - 1, 0, 0, vb1943_test_pattern_menu);

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
		goto free_ctrls;
	}

	pr_info("%s camera mipi vb1943, is found\n", __func__);

	return 0;

free_ctrls:
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
probe_err_free_entiny:
	media_entity_cleanup(&sd->entity);

probe_err_power_off:
	vb1943_power_off(sensor);

	return retval;
}

static void vb1943_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct vb1943 *sensor = client_to_vb1943(client);

	pr_debug("enter %s function\n", __func__);

	if (!(strcmp(sensor->gmsl, "gmsl"))) {
		max96792_sdev_unregister(sensor->dser_dev, &sensor->i2c_client->dev);
		vb1943_gmsl_serdes_reset(sensor);
	}

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	media_entity_cleanup(&sd->entity);
	vb1943_power_off(sensor);
	mutex_destroy(&sensor->lock);
}

static int __maybe_unused vb1943_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct vb1943 *sensor = client_to_vb1943(client);

	sensor->resume_status = sensor->stream_status;
	if (sensor->resume_status)
		vb1943_s_stream(&sensor->sd, 0);

	return 0;
}

static int __maybe_unused vb1943_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct vb1943 *sensor = client_to_vb1943(client);

	if (sensor->resume_status)
		vb1943_s_stream(&sensor->sd, 1);

	return 0;
}

static const struct dev_pm_ops vb1943_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(vb1943_suspend, vb1943_resume)
};

static const struct i2c_device_id vb1943_id[] = {
	{"vb1943", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, vb1943_id);


static struct i2c_driver vb1943_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "vb1943",
		.pm = &vb1943_pm_ops,
		.of_match_table	= vb1943_of_match,
	},
	.probe  = vb1943_probe,
	.remove = vb1943_remove,
	.id_table = vb1943_id,
};

module_i2c_driver(vb1943_i2c_driver);
MODULE_DESCRIPTION("VB1943 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");
