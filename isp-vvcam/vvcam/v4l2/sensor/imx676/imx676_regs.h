#include "vvsensor.h"

/**
 * Image sensor registers as described in the IMX676 register map
 */

#define STANDBY              0x3000
#define REGHOLD              0x3001
#define XMSTA                0x3002

#define INCK_SEL             0x3014
#define DATARATE_SEL         0x3015
#define WINMODE              0x3018
#define WDMODE               0x301A
#define ADDMODE              0x301B

#define THIN_V_EN            0x301C
#define VCMODE               0x301E

#define HVREVERSE            0x3020
#define VREVERSE             0x3021
#define ADBIT                0x3022
#define MDBIT                0x3023

#define VMAX_LOW             0x3028
#define VMAX_MID             0x3029
#define VMAX_HIGH            0x302A
#define HMAX_LOW             0x302C
#define HMAX_HIGH            0x302D

#define FDG_SEL0             0x3030
#define FDG_SEL1             0x3031
#define FDG_SEL2             0x3032

#define PIX_HST_LOW          0x303C
#define PIX_HST_HIGH         0x303D
#define PIX_HWIDTH_LOW       0x303E
#define PIX_HWIDTH_HIGH      0x303F

#define LANEMODE             0x3040
#define XSIZE_OVRLAP_LOW     0x3042

#define PIX_VST_LOW          0x3044
#define PIX_VST_HIGH         0x3045
#define PIX_VWIDTH_LOW       0x3046
#define PIX_VWIDTH_HIGH      0x3047

#define GAIN_HG0             0x304C
#define SHR0_LOW             0x3050
#define SHR0_MID             0x3051
#define SHR0_HIGH            0x3052
#define SHR1_LOW             0x3054
#define SHR1_MID             0x3055
#define SHR1_HIGH            0x3056

#define SHR2                 0x3058
#define RHS1                 0x3060
#define RHS2                 0x3064

#define GAIN_0               0x3070
#define GAIN_1               0x3072
#define GAIN_2               0x3074 // used in DOL mode

#define XHSOUTSEL_XVSOUTSEL  0x30A4
#define XVS_XHS_DRV          0x30A6
#define XVSLNG               0x30CC
#define XHSLNG               0x30CD

#define BLKLEVEL_LOW         0x30DC
#define BLKLEVEL_HIGH        0x30DD

#define TPG_EN_DUOUT        0x30E0
#define TPG_PATSEL_DUOUT    0x30E2
#define TPG_COLORWIDTH      0x30E4
#define TESTCLKEN           0x5300

#define EXTMODE             0x30CE
#define SECOND_SLAVE_ADD    0x300C

/**
 * Special values for the write table function
 */
#define IMX676_TABLE_WAIT_MS    0
#define IMX676_TABLE_END        1
#define IMX676_WAIT_MS          10

#define IMX676_DEFAULT_WIDTH            3552
#define IMX676_DEFAULT_HEIGHT           3092

#define IMX676_CROP_3552x2160_WIDTH     3552
#define IMX676_CROP_3552x2160_HEIGHT    2160

#define IMX676_MODE_BINNING_H2V2_WIDTH  1776
#define IMX676_MODE_BINNING_H2V2_HEIGHT 1778

#define IMX676_CROP_BINNING_1768x1080_WIDTH     1768
#define IMX676_CROP_BINNING_1768x1080_HEIGHT    1080

#define IMX676_MIN_FRAME_LENGTH_DELTA  72

#define IMX676_TO_LOW_BYTE(x) (x & 0xFF)
#define IMX676_TO_MID_BYTE(x) (x>>8)

static struct vvcam_sccb_data_s imx676_10bit_mode[] = {
    {ADBIT,     0x00},
    {MDBIT,     0x00},

    {0x355A,    0x1C},

    {0x3C0A,    0x03},
    {0x3C0B,    0x03},
    {0x3C0C,    0x03},
    {0x3C0D,    0x03},
    {0x3C0E,    0x03},
    {0x3C0F,    0x03},
};

static struct vvcam_sccb_data_s imx676_12bit_mode[] = {
    {ADBIT,     0x01},
    {MDBIT,     0x01},

    {0x355A,    0x10},

    {0x3C0A,    0x1F},
    {0x3C0B,    0x1F},
    {0x3C0C,    0x1F},
    {0x3C0D,    0x1F},
    {0x3C0E,    0x1F},
    {0x3C0F,    0x1F},
};

static struct vvcam_sccb_data_s imx676_init_setting[] = {
	{LANEMODE,             0x03},
    // INCK = 37.125Mhz
    {INCK_SEL,             0x01},

    {0x304E,               0x04},
    {0x3148,               0x00},
    {0x3460,               0x22},
    {0x347B,               0x02},
    {0x3A3C,               0x0F},
    {0x3A44,               0x0B},
    {0x3A76,               0xB5},
    {0x3A77,               0x00},
    {0x3A78,               0x03},
    {0x3B22,               0x04},
    {0x3B23,               0x44},
    {0x3C03,               0x04},
    {0x3C04,               0x04},
    {0x3C30,               0x73},
    {0x3C34,               0x6C},
    {0x3C3C,               0x20},
    {0x3CB8,               0x00},
    {0x3CBA,               0xFF},
    {0x3CBB,               0x03},
    {0x3CBC,               0xFF},
    {0x3CBD,               0x03},
    {0x3CC2,               0xFF},
    {0x3CC3,               0x03},
    {0x3CC8,               0xFF},
    {0x3CC9,               0x03},
    {0x3CCA,               0x00},
    {0x3CCE,               0xFF},
    {0x3CCF,               0x03},
    {0x3CD0,               0xFF},
    {0x3CD1,               0x03},
    {0x3E00,               0x1E},
    {0x3E02,               0x04},
    {0x3E03,               0x00},
    {0x3E20,               0x04},
    {0x3E21,               0x00},
    {0x3E22,               0x1E},
    {0x3E24,               0xB6},
    {0x4490,               0x07},
    {0x4494,               0x10},
    {0x4495,               0x00},
    {0x4496,               0xB2},
    {0x4497,               0x00},
    {0x44A0,               0x33},
    {0x44A2,               0x10},
    {0x44A4,               0x10},
    {0x44A6,               0x10},
    {0x44A8,               0x4B},  // no clear HDR
    {0x44AA,               0x4B},  // no clear HDR
    {0x44AC,               0x4B},  // no clear HDR
    {0x44AE,               0x46},  // no clear HDR
    {0x44B0,               0x33},  // no clear HDR
    {0x44B2,               0x10},
    {0x44B4,               0x10},
    {0x44B6,               0x10},
    {0x44B8,               0x42}, // no clear HDR
    {0x44BA,               0x42}, // no clear HDR
    {0x44BC,               0x42}, // no clear HDR
    {0x44BE,               0x42}, // no clear HDR
    {0x44C0,               0x33}, // no clear HDR
    {0x44C2,               0x10},
    {0x44C4,               0x10},
    {0x44C6,               0x10},
    {0x44C8,               0xE7},
    {0x44CA,               0xE2},
    {0x44CC,               0xE2},
    {0x44CE,               0xDD},
    {0x44D0,               0xDD},
    {0x44D2,               0xB2},
    {0x44D4,               0xB2},
    {0x44D6,               0xB2},
    {0x44D8,               0xE1},
    {0x44DA,               0xE1},
    {0x44DC,               0xE1},
    {0x44DE,               0xDD},
    {0x44E0,               0xDD},
    {0x44E2,               0xB2},
    {0x44E4,               0xB2},
    {0x44E6,               0xB2},
    {0x44E8,               0xDD},
    {0x44EA,               0xDD},
    {0x44EC,               0xDD},
    {0x44EE,               0xDD},
    {0x44F0,               0xDD},
    {0x44F2,               0xB2},
    {0x44F4,               0xB2},
    {0x44F6,               0xB2},
    {0x4538,               0x15},
    {0x4539,               0x15},
    {0x453A,               0x15},
    {0x4544,               0x15},
    {0x4545,               0x15},
    {0x4546,               0x15},
    {0x4550,               0x10},
    {0x4551,               0x10},
    {0x4552,               0x10},
    {0x4553,               0x10},
    {0x4554,               0x10},
    {0x4555,               0x10},
    {0x4556,               0x10},
    {0x4557,               0x10},
    {0x4558,               0x10},
    {0x455C,               0x10},
    {0x455D,               0x10},
    {0x455E,               0x10},
    {0x455F,               0x10},
    {0x4560,               0x10},
    {0x4561,               0x10},
    {0x4562,               0x10},
    {0x4563,               0x10},
    {0x4564,               0x10},
    {0x4604,               0x04},
    {0x4608,               0x22},
    {0x479C,               0x04},
    {0x47A0,               0x22},
    {0x4E3C,               0x07}, // output interface for 2 and 4 lane

    {HMAX_LOW,             0x74},
    {HMAX_HIGH,            0x02},
    {XVS_XHS_DRV,          0x00},
    {SHR0_LOW,             0x08},
    {VMAX_LOW,             0x64},
    {VMAX_MID,             0x0F},

    // this one is tricky, implement a special logic for this when selecting data rate
    {0x355A,               0x1C},

    // AD conversion bit -check this for different data rates
    {0x3C0A,               0x03},
    {0x3C0B,               0x03},
    {0x3C0C,               0x03},
    {0x3C0D,               0x03},
    {0x3C0E,               0x03},
    {0x3C0F,               0x03},

    // 1188 data rate
    {DATARATE_SEL,         0x04},
};

static struct vvcam_sccb_data_s mode_3552x3092[] = {
    {WINMODE,              0x04},
    {ADDMODE,              0x00},
 
    // options for non clear HDR no binning
    {0x4498,               0x4C},
    {0x449A,               0x4B},
    {0x449C,               0x4B},
    {0x449E,               0x49},
 
    {PIX_HST_HIGH,      0x00},    
    {PIX_HST_LOW,       0x00},
    {PIX_HWIDTH_HIGH,   0x0D},
    {PIX_HWIDTH_LOW,    0xE0},
    {PIX_VST_HIGH,      0x00},
    {PIX_VST_LOW,       0xE8},
    {PIX_VWIDTH_HIGH,   0x0C},
    {PIX_VWIDTH_LOW,    0x14},
};

static struct vvcam_sccb_data_s mode_crop_3552x2160[] = {
    {WINMODE,           0x04},
    {ADDMODE,           0x00},
    
    // options for non clear HDR no binning
    {0x4498,               0x4C},
    {0x449A,               0x4B},
    {0x449C,               0x4B},
    {0x449E,               0x49},

    {PIX_HST_HIGH,      0x00},    
    {PIX_HST_LOW,       0x00},
    {PIX_HWIDTH_HIGH,   0x0D},
    {PIX_HWIDTH_LOW,    0xE0},
    {PIX_VST_HIGH,      0x02},
    {PIX_VST_LOW,       0xBA},
    {PIX_VWIDTH_HIGH,   0x08},
    {PIX_VWIDTH_LOW,    0x70},
};

static struct vvcam_sccb_data_s mode_h2v2_binning[] = {
    {WINMODE,               0x00},
    {ADDMODE,               0x01},

    {ADBIT,                 0x00},
    {MDBIT,                 0x01},

    {0x355A,                0x00},

    {0x3C0A,    0x03},
    {0x3C0B,    0x03},
    {0x3C0C,    0x03},
    {0x3C0D,    0x03},
    {0x3C0E,    0x03},
    {0x3C0F,    0x03},

    // options for non clear HDR with binning
    {0x4498,               0x50},
    {0x449A,               0x4B},
    {0x449C,               0x4B},
    {0x449E,               0x47},
};

static struct vvcam_sccb_data_s mode_crop_binning_1768x1080[] = {
    {WINMODE,               0x04},
    {ADDMODE,               0x01},

    {ADBIT,                 0x00},
    {MDBIT,                 0x01},

    {0x355A,                0x00},

    {0x3C0A,    0x03},
    {0x3C0B,    0x03},
    {0x3C0C,    0x03},
    {0x3C0D,    0x03},
    {0x3C0E,    0x03},
    {0x3C0F,    0x03},

    // options for non clear HDR with binning
    {0x4498,               0x50},
    {0x449A,               0x4B},
    {0x449C,               0x4B},
    {0x449E,               0x47},

    {PIX_HST_HIGH,      0x00},    
    {PIX_HST_LOW,       0x00},
    {PIX_HWIDTH_HIGH,   0x0D},
    {PIX_HWIDTH_LOW,    0xD0},
    {PIX_VST_HIGH,      0x02},
    {PIX_VST_LOW,       0xBA},
    {PIX_VWIDTH_HIGH,   0x08},
    {PIX_VWIDTH_LOW,    0x70},
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
    IMX676_2376_MBPS,
    IMX676_2079_MBPS,
    IMX676_1782_MBPS,
    IMX676_1440_MBPS,
    IMX676_1188_MBPS,
    IMX676_891_MBPS,
    IMX676_720_MBPS,
    IMX676_594_MBPS,
} data_rate_mode;

typedef enum {
    NO_SYNC,
    INTERNAL_SYNC,
    EXTERNAL_SYNC,
} sync_mode;
