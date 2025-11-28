/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2024, Framos.  All rights reserved.
 *
 * imx838_regs.h - imx838 header
 */
#include "vvsensor.h"

/*
 * Image sensor registers as described in the IMX838 register map
 */

#define STANDBY             0x3000
#define REGHOLD             0x3001
#define XMSTA               0x3002
#define XMASTER             0x3003
#define INCK_SEL            0x3014
#define DATARATE_SEL        0x3015
#define WINMODE             0x3018

#define WDMODE              0x301A
#define ADDMODE             0x301B
#define THIN_V_EN           0x301C

#define HREVERSE            0x3020
#define VREVERSE            0x3021
#define ADBIT               0x3022
#define MDBIT               0x3023
#define VMAX_LOW            0x3028
#define VMAX_MID            0x3029
#define VMAX_HIGH           0x302A
#define HMAX_LOW            0x302C
#define HMAX_HIGH           0x302D
#define FDG_SEL0            0x3030
#define FDG_SEL1            0x3031
#define FDG_SEL2            0x3032
#define PIX_HST_LOW         0x303C
#define PIX_HST_HIGH        0x303D
#define PIX_HWIDTH_LOW      0x303E
#define PIX_HWIDTH_HIGH     0x303F
#define LANEMODE            0x3040
#define XSIZE_OVERLAP_LOW   0x3042
#define XSIZE_OVERLAP_HIGH  0x3043
#define PIX_VST_LOW         0x3044
#define PIX_VST_HIGH        0x3045
#define PIX_VWIDTH_LOW      0x3046
#define PIX_VWIDTH_HIGH     0x3047
#define SHR0_LOW            0x3050
#define SHR0_MID            0x3051
#define SHR0_HIGH           0x3052
#define SHR1_LOW            0x3054
#define SHR1_MID            0x3055
#define SHR1_HIGH           0x3056
#define SHR2_LOW            0x3058
#define SHR2_MID            0x3059
#define SHR2_HIGH           0x305A
#define RHS1_LOW            0x3060
#define RHS1_MID            0x3061
#define RHS1_HIGH           0x3062
#define RHS2_LOW            0x3064
#define RHS2_MID            0x3065
#define RHS2_HIGH           0x3066
#define CHDR_GAIN_EN        0x3069
#define GAIN_0_LOW          0x3070
#define GAIN_0_HIGH         0x3071
#define GAIN_1_LOW          0x3072
#define GAIN_1_HIGH         0x3073
#define GAIN_2_LOW          0x3074
#define GAIN_2_HIGH         0x3075
#define EXP_GAIN            0x3081
#define CHDR_DGAIN0_HG_LOW  0x308C
#define CHDR_DGAIN0_HG_HIGH 0x308D
#define CHDR_AGAIN0_LG_LOW  0x3094
#define CHDR_AGAIN0_LG_HIGH 0x3095
#define CHDR_AGAIN1_LOW     0x3096
#define CHDR_AGAIN1_HIGH    0x3097
#define CHDR_AGAIN0_HG_LOW  0x309C
#define CHDR_AGAIN0_HG_HIGH 0x309D
#define XHSOUTSEL_XVSOUTSEL 0x30A4
#define XVS_XHS_DRV         0x30A6
#define XVSLNG              0x30CC
#define XHSLNG              0x30CD
#define BLKLEVEL_LOW        0x30DC
#define BLKLEVEL_HIGH       0x30DD
#define GAIN_PGC_FIDMD      0x3400

#define TPG_EN_DUOUT        0x30E0
#define TPG_PATSEL_DUOUT    0x30E2
#define TPG_COLORWIDTH      0x30E4

#define EXTMODE             0x30CE

/*
 * Special values for the write table function
 */
#define IMX838_TABLE_WAIT_MS    0
#define IMX838_TABLE_END        1
#define IMX838_WAIT_MS          10

/*
 * Default resolution
 */
#define IMX838_DEFAULT_WIDTH    3856
#define IMX838_DEFAULT_HEIGHT   2180

#define IMX838_MODE_BINNING_H2V2_WIDTH 1928
#define IMX838_MODE_BINNING_H2V2_HEIGHT 1090

#define IMX838_MIN_FRAME_LENGTH_DELTA  70

#define IMX838_TO_LOW_BYTE(x) (x & 0xFF)
#define IMX838_TO_MID_BYTE(x) (x>>8)

static struct vvcam_sccb_data_s imx838_10bit_mode[] = {
	{ADBIT,                0x00},
	{MDBIT,                0x00},
};

static struct vvcam_sccb_data_s imx838_10bit_mode_clearHDR[] = {
	{ADBIT,                0x00},
	{MDBIT,                0x00},
	{0x355A,               0x50},
};

static struct vvcam_sccb_data_s imx838_12bit_mode[] = {
	{ADBIT,                0x01},
	{MDBIT,                0x01},
};

static struct vvcam_sccb_data_s imx838_12bit_mode_clearHDR[] = {
	{ADBIT,                0x01},
	{MDBIT,                0x01},
	{0x355A,               0x69},
};

static struct vvcam_sccb_data_s imx838_init_setting[] = {
	{LANEMODE,  0x03},
	{XMASTER,   0x00},
	{VREVERSE,  0x00},
	{HREVERSE,  0x00},
	{WINMODE,   0x00},
	{ADDMODE,   0x00},

	{0x3460,    0x22},

	{0x3B4C,    0x30},
	{0x3B4E,    0x30},
	{0x3BD8,    0x62},
	{0x3BDC,    0x62},

	{0x3C04,    0x06},
	{0x3C05,    0x06},
	{0x3C0C,    0x00},
	{0x3C0D,    0x00},
	{0x3C0E,    0x04},
	{0x3C0F,    0x04},
	{0x3C10,    0x04},
	{0x3C11,    0x04},
	{0x3C12,    0x04},
	{0x3C13,    0x04},
	{0x3C3C,    0x02},
	{0x3CAA,    0x02},
	{0x3CAB,    0x01},
	{0x3CC0,    0x04},
	{0x3CC1,    0x01},

	{0x3D39,    0xEE},
	{0x3D3C,    0xAA},
	{0x3D3D,    0x79},
	{0x3D48,    0xCC},
	{0x43C0,    0x1A},
	{0x43C2,    0x1A},
	{0x43C4,    0x1A},
	{0x43C6,    0x1A},
	{0x43C8,    0x1A},
	{0x43CA,    0x1A},
	{0x43CC,    0x1A},
	{0x43CE,    0x1A},
	{0x43D0,    0xE7},
	{0x43D2,    0xE7},
	{0x43D4,    0xE7},
	{0x43D6,    0xE5},
	{0x43D8,    0xBC},
	{0x43DA,    0xBC},
	{0x43DC,    0xBC},
	{0x43DE,    0xBC},
	{0x443D,    0x33},
	{0x449F,    0x0C},

	{0x44A8,    0x1A},
	{0x44AA,    0x1A},
	{0x44AC,    0x1A},
	{0x44AE,    0x1A},
	{0x44C0,    0xE7},
	{0x44C2,    0xE7},
	{0x44C4,    0xE7},
	{0x44C6,    0xE5},
	{0x44C8,    0xE3},
	{0x44CA,    0xBC},
	{0x44CC,    0xBC},
	{0x44CE,    0xBC},
	{0x44E0,    0x19},
	{0x44E1,    0x00},
	{0x44E2,    0xBB},
	{0x44E3,    0x00},

	{0x45B4,    0x1B},
	{0x45B8,    0x1B},

	// other than clear HDR
	{0x3A24,    0x05},
	{0x3A26,    0x0A},
	{0x355A,    0x64},
	{0x44A0,    0x4C},
	{0x44A2,    0x44},
	{0x44A4,    0x44},
	{0x44A6,    0x3C},

	{0x4549,    0x01},
	{0x454A,    0x01},
	{0x454B,    0x06},
	{0x454C,    0x06},
	{0x454D,    0x06},
	{0x454E,    0x06},
	{0x454F,    0x06},
	{0x4550,    0x06},

	{0x4E3C,    0x07},
	{SHR0_LOW,  0x06},

	/* INCK = 37.125Mhz */
	{INCK_SEL,  0x01},
};

static struct vvcam_sccb_data_s mode_3856x2180[] = {
	{WINMODE,              0x00},
	{ADDMODE,              0x00},
};

static struct vvcam_sccb_data_s mode_h2v2_binning[] = {
	{WINMODE,               0x00},
	{ADDMODE,               0x01},

	{ADBIT,                 0x00},
	{MDBIT,                 0x01},
};

static struct vvcam_sccb_data_s imx838_setting_dol_hdr[] = {
	{WINMODE,            0x00},
	{WDMODE,             0x01},
	{ADDMODE,            0x00},
	{THIN_V_EN,          0x01},

	{GAIN_PGC_FIDMD,     0x00},

	{SHR0_LOW,           0x40},
	{SHR0_MID,           0x0b},
	{SHR1_LOW,           0x05},

	{RHS1_LOW,           0x49},
	{RHS1_MID,           0x00},
};

static struct vvcam_sccb_data_s imx838_setting_clear_hdr[] = {
	{WINMODE,   0x00},
	{WDMODE,    0x08},
	{ADDMODE,   0x00},

	{VMAX_LOW,  0x94},
	{VMAX_MID,  0x11},

	{FDG_SEL0,  0x02},
	{SHR0_LOW,  0x06},
	{SHR0_MID,  0x00},

	{EXP_GAIN,  0x02},
	{0x3A24,    0x28},
	{0x3A26,    0x30},

	{0x44A0,    0x4E},
	{0x44A2,    0x4E},
	{0x44A4,    0x4C},
	{0x44A6,    0x4C},

	{0x4549,    0x00},
	{0x454A,    0x00},
	{0x454B,    0x04},
	{0x454C,    0x04},
	{0x454D,    0x04},
	{0x454E,    0x04},
	{0x454F,    0x04},
	{0x4550,    0x04},

};

static struct vvcam_sccb_data_s mode_enable_pattern_generator[] = {
	{BLKLEVEL_LOW,         0x00},
	{TPG_EN_DUOUT,         0x01},
	{TPG_COLORWIDTH,       0x02},
};

static struct vvcam_sccb_data_s mode_disable_pattern_generator[] = {
	{BLKLEVEL_LOW,         0x32},
	{TPG_EN_DUOUT,         0x00},
	{TPG_COLORWIDTH,       0x00},
};

enum data_rate_mode {
	IMX838_2376_MBPS,
	IMX838_2079_MBPS,
	IMX838_1782_MBPS,
	IMX838_1440_MBPS,
	IMX838_1188_MBPS,
	IMX838_891_MBPS,
	IMX838_720_MBPS,
	IMX838_594_MBPS,
};

enum sync_mode {
	NO_SYNC,
	INTERNAL_SYNC,
	EXTERNAL_SYNC,
};
