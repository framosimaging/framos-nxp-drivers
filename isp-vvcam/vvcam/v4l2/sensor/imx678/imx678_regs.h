#include "vvsensor.h"

/**
 * Image sensor registers as described in the IMX678 register map
 */

#define STANDBY             0x3000
#define REGHOLD             0x3001
#define XMSTA               0x3002
#define XMASTER             0x3003
#define INCK_SEL            0x3014
#define DATARATE_SEL        0x3015
#define WINMODE             0x3018
#define CFMODE              0x3019
#define WDMODE              0x301A
#define ADDMODE             0x301B
#define THIN_V_EN           0x301C
#define VCMODE              0x301E
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

/**
 * Special values for the write table function
 */
#define IMX678_TABLE_WAIT_MS    0
#define IMX678_TABLE_END        1
#define IMX678_WAIT_MS          10

/**
 * Default resolution
 */
#define IMX678_DEFAULT_WIDTH    3856
#define IMX678_DEFAULT_HEIGHT   2180

#define IMX678_MODE_BINNING_H2V2_WIDTH 1928
#define IMX678_MODE_BINNING_H2V2_HEIGHT 1090

#define IMX678_MIN_FRAME_LENGTH_DELTA  70

#define IMX678_TO_LOW_BYTE(x) (x & 0xFF)
#define IMX678_TO_MID_BYTE(x) (x>>8)

static struct vvcam_sccb_data_s imx678_10bit_mode[] = {
    {ADBIT,     0x00},
    {MDBIT,     0x00},
};

static struct vvcam_sccb_data_s imx678_12bit_mode[] = {
    {ADBIT,     0x01},
    {MDBIT,     0x01},
};

static struct vvcam_sccb_data_s imx678_init_setting[] = {
	{LANEMODE,             0x03},
    {XMASTER,              0x00},
    {VREVERSE,             0x00},
    {HREVERSE,             0x00},
    {WINMODE,              0x00},
    {ADDMODE,              0x00},

    {0x3460,               0x22},
    {0x355A,               0x64},
    {0x3A02,               0x7A},
    {0x3A10,               0xEC},
    {0x3A12,               0x71},
    {0x3A14,               0xDE},
    {0x3A20,               0x2B},
    {0x3A24,               0x22},
    {0x3A25,               0x25},
    {0x3A26,               0x2A},
    {0x3A27,               0x2C},
    {0x3A28,               0x39},
    {0x3A29,               0x38},
    {0x3A30,               0x04},
    {0x3A31,               0x04},
    {0x3A32,               0x03},
    {0x3A33,               0x03},
    {0x3A34,               0x09},
    {0x3A35,               0x06},
    {0x3A38,               0xCD},
    {0x3A3A,               0x4C},
    {0x3A3C,               0xB9},
    {0x3A3E,               0x30},
    {0x3A40,               0x2C},
    {0x3A42,               0x39},
    {0x3A4E,               0x00},
    {0x3A52,               0x00},
    {0x3A56,               0x00},
    {0x3A5A,               0x00},
    {0x3A5E,               0x00},
    {0x3A62,               0x00},
    {0x3A64,               0x00},
    {0x3A6E,               0xA0},
    {0x3A70,               0x50},
    {0x3A8C,               0x04},
    {0x3A8D,               0x03},
    {0x3A8E,               0x09},
    {0x3A90,               0x38},
    {0x3A91,               0x42},
    {0x3A92,               0x3C},
    {0x3B0E,               0xF3},
    {0x3B12,               0xE5},
    {0x3B27,               0xC0},
    {0x3B2E,               0xEF},
    {0x3B30,               0x6A},
    {0x3B32,               0xF6},
    {0x3B36,               0xE1},
    {0x3B3A,               0xE8},
    {0x3B5A,               0x17},
    {0x3B5E,               0xEF},
    {0x3B60,               0x6A},
    {0x3B62,               0xF6},
    {0x3B66,               0xE1},
    {0x3B6A,               0xE8},
    {0x3B88,               0xEC},
    {0x3B8A,               0xED},
    {0x3B94,               0x71},
    {0x3B96,               0x72},
    {0x3B98,               0xDE},
    {0x3B9A,               0xDF},
    {0x3C0F,               0x06},
    {0x3C10,               0x06},
    {0x3C11,               0x06},
    {0x3C12,               0x06},
    {0x3C13,               0x06},
    {0x3C18,               0x20},
    {0x3C37,               0x10},
    {0x3C3A,               0x7A},
    {0x3C40,               0xF4},
    {0x3C48,               0xE6},
    {0x3C54,               0xCE},
    {0x3C56,               0xD0},
    {0x3C6C,               0x53},
    {0x3C6E,               0x55},
    {0x3C70,               0xC0},
    {0x3C72,               0xC2},
    {0x3C7E,               0xCE},
    {0x3C8C,               0xCF},
    {0x3C8E,               0xEB},
    {0x3C98,               0x54},
    {0x3C9A,               0x70},
    {0x3C9C,               0xC1},
    {0x3C9E,               0xDD},
    {0x3CB0,               0x7A},
    {0x3CB2,               0xBA},
    {0x3CC8,               0xBC},
    {0x3CCA,               0x7C},
    {0x3CD4,               0xEA},
    {0x3CD5,               0x01},
    {0x3CD6,               0x4A},
    {0x3CD8,               0x00},
    {0x3CD9,               0x00},
    {0x3CDA,               0xFF},
    {0x3CDB,               0x03},
    {0x3CDC,               0x00},
    {0x3CDD,               0x00},
    {0x3CDE,               0xFF},
    {0x3CDF,               0x03},
    {0x3CE4,               0x4C},
    {0x3CE6,               0xEC},
    {0x3CE7,               0x01},
    {0x3CE8,               0xFF},
    {0x3CE9,               0x03},
    {0x3CEA,               0x00},
    {0x3CEB,               0x00},
    {0x3CEC,               0xFF},
    {0x3CED,               0x03},
    {0x3CEE,               0x00},
    {0x3CEF,               0x00},
    {0x3CF2,               0xFF},
    {0x3CF3,               0x03},
    {0x3CF4,               0x00},
    {0x3E28,               0x82},
    {0x3E2A,               0x80},
    {0x3E30,               0x85},
    {0x3E32,               0x7D},
    {0x3E5C,               0xCE},
    {0x3E5E,               0xD3},
    {0x3E70,               0x53},
    {0x3E72,               0x58},
    {0x3E74,               0xC0},
    {0x3E76,               0xC5},
    {0x3E78,               0xC0},
    {0x3E79,               0x01},
    {0x3E7A,               0xD4},
    {0x3E7B,               0x01},
    {0x3EB4,               0x0B},
    {0x3EB5,               0x02},
    {0x3EB6,               0x4D},
    {0x3EB7,               0x42},
    {0x3EEC,               0xF3},
    {0x3EEE,               0xE7},
    {0x3F01,               0x01},
    {0x3F24,               0x10},
    {0x3F28,               0x2D},
    {0x3F2A,               0x2D},
    {0x3F2C,               0x2D},
    {0x3F2E,               0x2D},
    {0x3F30,               0x23},
    {0x3F38,               0x2D},
    {0x3F3A,               0x2D},
    {0x3F3C,               0x2D},
    {0x3F3E,               0x28},
    {0x3F40,               0x1E},
    {0x3F48,               0x2D},
    {0x3F4A,               0x2D},
    {0x3F4C,               0x00},
    {0x4004,               0xE4},
    {0x4006,               0xFF},
    {0x4018,               0x69},
    {0x401A,               0x84},
    {0x401C,               0xD6},
    {0x401E,               0xF1},
    {0x4038,               0xDE},
    {0x403A,               0x00},
    {0x403B,               0x01},
    {0x404C,               0x63},
    {0x404E,               0x85},
    {0x4050,               0xD0},
    {0x4052,               0xF2},
    {0x4108,               0xDD},
    {0x410A,               0xF7},
    {0x411C,               0x62},
    {0x411E,               0x7C},
    {0x4120,               0xCF},
    {0x4122,               0xE9},
    {0x4138,               0xE6},
    {0x413A,               0xF1},
    {0x414C,               0x6B},
    {0x414E,               0x76},
    {0x4150,               0xD8},
    {0x4152,               0xE3},
    {0x417E,               0x03},
    {0x417F,               0x01},
    {0x4186,               0xE0},
    {0x4190,               0xF3},
    {0x4192,               0xF7},
    {0x419C,               0x78},
    {0x419E,               0x7C},
    {0x41A0,               0xE5},
    {0x41A2,               0xE9},
    {0x41C8,               0xE2},
    {0x41CA,               0xFD},
    {0x41DC,               0x67},
    {0x41DE,               0x82},
    {0x41E0,               0xD4},
    {0x41E2,               0xEF},
    {0x4200,               0xDE},
    {0x4202,               0xDA},
    {0x4218,               0x63},
    {0x421A,               0x5F},
    {0x421C,               0xD0},
    {0x421E,               0xCC},
    {0x425A,               0x82},
    {0x425C,               0xEF},
    {0x4348,               0xFE},
    {0x4349,               0x06},
    {0x4352,               0xCE},
    {0x4420,               0x0B},
    {0x4421,               0x02},
    {0x4422,               0x4D},
    {0x4423,               0x0A},
    {0x4426,               0xF5},
    {0x442A,               0xE7},
    {0x4432,               0xF5},
    {0x4436,               0xE7},
    {0x4466,               0xB4},
    {0x446E,               0x32},
    {0x449F,               0x1C},
    {0x44A4,               0x2C},
    {0x44A6,               0x2C},
    {0x44A8,               0x2C},
    {0x44AA,               0x2C},
    {0x44B4,               0x2C},
    {0x44B6,               0x2C},
    {0x44B8,               0x2C},
    {0x44BA,               0x2C},
    {0x44C4,               0x2C},
    {0x44C6,               0x2C},
    {0x44C8,               0x2C},
    {0x4506,               0xF3},
    {0x450E,               0xE5},
    {0x4516,               0xF3},
	{0x4522,               0xE5},
    {0x4524,               0xF3},
    {0x452C,               0xE5},
    {0x453C,               0x22},
    {0x453D,               0x1B},
    {0x453E,               0x1B},
    {0x453F,               0x15},
    {0x4540,               0x15},
    {0x4541,               0x15},
    {0x4542,               0x15},
    {0x4543,               0x15},
    {0x4544,               0x15},
    {0x4548,               0x00},
    {0x4549,               0x01},
    {0x454A,               0x01},
    {0x454B,               0x06},
    {0x454C,               0x06},
    {0x454D,               0x06},
    {0x454E,               0x06},
    {0x454F,               0x06},
    {0x4550,               0x06},
    {0x4554,               0x55},
    {0x4555,               0x02},
    {0x4556,               0x42},
    {0x4557,               0x05},
    {0x4558,               0xFD},
    {0x4559,               0x05},
    {0x455A,               0x94},
    {0x455B,               0x06},
    {0x455D,               0x06},
    {0x455E,               0x49},
    {0x455F,               0x07},
    {0x4560,               0x7F},
    {0x4561,               0x07},
    {0x4562,               0xA5},
    {0x4564,               0x55},
	{0x4565,               0x02},
    {0x4566,               0x42},
    {0x4567,               0x05},
    {0x4568,               0xFD},
    {0x4569,               0x05},
    {0x456A,               0x94},
    {0x456B,               0x06},
    {0x456D,               0x06},
    {0x456E,               0x49},
    {0x456F,               0x07},
    {0x4572,               0xA5},
    {0x460C,               0x7D},
    {0x460E,               0xB1},
    {0x4614,               0xA8},
    {0x4616,               0xB2},
    {0x461C,               0x7E},
    {0x461E,               0xA7},
    {0x4624,               0xA8},
    {0x4626,               0xB2},
    {0x462C,               0x7E},
    {0x462E,               0x8A},
    {0x4630,               0x94},
    {0x4632,               0xA7},
    {0x4634,               0xFB},
    {0x4636,               0x2F},
    {0x4638,               0x81},
    {0x4639,               0x01},
    {0x463A,               0xB5},
	{0x463B,               0x01},
    {0x463C,               0x26},
    {0x463E,               0x30},
    {0x4640,               0xAC},
    {0x4641,               0x01},
    {0x4642,               0xB6},
    {0x4643,               0x01},
    {0x4644,               0xFC},
    {0x4646,               0x25},
    {0x4648,               0x82},
    {0x4649,               0x01},
    {0x464A,               0xAB},
    {0x464B,               0x01},
    {0x464C,               0x26},
    {0x464E,               0x30},
    {0x4654,               0xFC},
    {0x4656,               0x08},
    {0x4658,               0x12},
    {0x465A,               0x25},
    {0x4662,               0xFC},
    {0x46A2,               0xFB},
    {0x46D6,               0xF3},
    {0x46E6,               0x00},
    {0x46E8,               0xFF},
    {0x46E9,               0x03},
	{0x46EC,               0x7A},
    {0x46EE,               0xE5},
    {0x46F4,               0xEE},
    {0x46F6,               0xF2},
    {0x470C,               0xFF},
    {0x470D,               0x03},
    {0x470E,               0x00},
    {0x4714,               0xE0},
    {0x4716,               0xE4},
    {0x471E,               0xED},
    {0x472E,               0x00},
    {0x4730,               0xFF},
    {0x4731,               0x03},
    {0x4734,               0x7B},
    {0x4736,               0xDF},
    {0x4754,               0x7D},
    {0x4756,               0x8B},
    {0x4758,               0x93},
    {0x475A,               0xB1},
    {0x475C,               0xFB},
    {0x475E,               0x09},
    {0x4760,               0x11},
    {0x4762,               0x2F},
    {0x4766,               0xCC},
    {0x4776,               0xCB},
    {0x477E,               0x4A},
	{0x478E,               0x49},
    {0x4794,               0x7C},
    {0x4796,               0x8F},
    {0x4798,               0xB3},
    {0x4799,               0x00},
    {0x479A,               0xCC},
    {0x479C,               0xC1},
    {0x479E,               0xCB},
    {0x47A4,               0x7D},
    {0x47A6,               0x8E},
    {0x47A8,               0xB4},
    {0x47A9,               0x00},
    {0x47AA,               0xC0},
    {0x47AC,               0xFA},
    {0x47AE,               0x0D},
    {0x47B0,               0x31},
    {0x47B1,               0x01},
    {0x47B2,               0x4A},
    {0x47B3,               0x01},
    {0x47B4,               0x3F},
    {0x47B6,               0x49},
    {0x47BC,               0xFB},
    {0x47BE,               0x0C},
    {0x47C0,               0x32},
    {0x47C1,               0x01},
    {0x47C2,               0x3E},
    {0x47C3,               0x01},

    {HMAX_LOW,             0x4C},
    {HMAX_HIGH,            0x04},

    {SHR0_LOW,             0x06},

    // 891 data rate
    {DATARATE_SEL,         0x05},

    // INCK = 37.125Mhz
    {INCK_SEL,             0x01},
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

typedef enum {
    IMX678_2376_MBPS,
    IMX678_2079_MBPS,
    IMX678_1782_MBPS,
    IMX678_1440_MBPS,
    IMX678_1188_MBPS,
    IMX678_891_MBPS,
    IMX678_720_MBPS,
    IMX678_594_MBPS,
} data_rate_mode;

typedef enum {
    NO_SYNC,
    INTERNAL_SYNC,
    EXTERNAL_SYNC,
} sync_mode;