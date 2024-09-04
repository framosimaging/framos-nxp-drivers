/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2024, Framos. All rights reserved.
 *
 * max96793.c - max96793 GMSL Serializer header
 */

/**
 * @file
 * <b>max96793 API: For Maxim Integrated max96793 serializer.</b>
 *
 * @b Description: Defines elements used to set up and use a
 *  Maxim Integrated max96793 serializer.
 */

#ifndef __MAX96793_H__
#define __MAX96793_H__

#include "gmsl-link.h"

/**
 * \defgroup max96793 max96793 serializer driver
 *
 * Controls the max96793 serializer module.
 *
 * @ingroup serdes_group
 * @{
 */

#define GMSL_CSI_1X4_MODE 0x1
#define GMSL_CSI_2X4_MODE 0x2
#define GMSL_CSI_2X2_MODE 0x3
#define GMSL_CSI_4X2_MODE 0x4

#define GMSL_CSI_PORT_A 0x0
#define GMSL_CSI_PORT_B 0x1
#define GMSL_CSI_PORT_C 0x2
#define GMSL_CSI_PORT_D 0x3
#define GMSL_CSI_PORT_E 0x4
#define GMSL_CSI_PORT_F 0x5

#define GMSL_SERDES_CSI_LINK_A 0x1
#define GMSL_SERDES_CSI_LINK_B 0x2

/* Didn't find kernel defintions, for now adding here */
#define GMSL_CSI_DT_RAW_12 0x2C
#define GMSL_CSI_DT_UED_U1 0x30
#define GMSL_CSI_DT_EMBED 0x12

#define GMSL_ST_ID_UNUSED 0xFF

/**
 * Maximum number of data streams (\ref gmsl_stream elements) in a GMSL link
 * (\ref gmsl_link_ctx).
 */
#define GMSL_DEV_MAX_NUM_DATA_STREAMS 4

/**
 * @brief  Powers on a serializer device and performs the I2C overrides
 * for sensor and serializer devices.
 *
 * The I2C overrides include setting proxy I2C slave addresses for the devices.
 *
 * Before the client calls this function it must ensure that
 * the deserializer device is in link_ex exclusive link mode
 * by calling the deserializer driver's max9296_setup_link() function.
 *
 * @param  [in]  dev	The serializer device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_setup_control(struct device *dev);

/**
 * Reverts I2C overrides and resets a serializer device.
 *
 * @param  [in]  dev	The serializer device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_reset_control(struct device *dev);

/**
 * @brief  Pairs a sensor device with a serializer device.
 *
 * To be called by sensor client driver.
 *
 * @param  [in]  dev	The deserializer device handle.
 * @param  [in]  g_ctx	The @ref gmsl_link_ctx structure handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_sdev_pair(struct device *dev, struct gmsl_link_ctx *g_ctx);

/**
 * @brief Unpairs a sensor device from a serializer device.
 *
 * To be called by sensor client driver.
 *
 * @param  [in]  dev	The serializer device handle.
 * @param  [in]  s_dev	The sensor device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_sdev_unpair(struct device *dev, struct device *s_dev);

/**
 * Sets up the serializer device's internal pipeline for a specified
 * sensor/serializer pair.
 *
 * @param  [in]  dev	The serializer device handle.
 * @param  [in]  code	Code format of the sensor (RGGB, GBRG, ...).
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96793_setup_streaming(struct device *dev, u32 code);

int max96793_gmsl3_setup(struct device *dev);

int max96793_gpio10_xtrig1_setup(struct device *dev, char *image_sensor_type);

int max96793_xvs_setup(struct device *dev, bool direction);

enum {
	max96793_OUT,
	max96793_IN,
};

/** @} */

#endif /* __max96793_H__ */
