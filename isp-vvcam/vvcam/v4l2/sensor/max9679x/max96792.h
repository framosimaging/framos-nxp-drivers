/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (c) 2024, Framos.  All rights reserved.
 *
 * max96792.h - max96792 GMSL Deserializer header
 */

/**
 * @file
 * <b>max96792 API: For Maxim Integrated MAX96792 deserializer.</b>
 *
 * @b Description: Defines elements used to set up and use a
 *  Maxim Integrated MAX96792 deserializer.
 */

#ifndef __MAX96792_H__
#define __MAX96792_H__

#include "gmsl-link.h"

/**
 * \defgroup max96792 MAX96792 deserializer driver
 *
 * Controls the MAX96792 deserializer module.
 *
 * @ingroup serdes_group
 * @{
 */

/**
 * Puts a deserializer device in single exclusive link mode, so link-specific
 * I2C overrides can be performed for sensor and serializer devices.
 *
 * @param [in]  dev	The deserializer device handle.
 * @param [in]  s_dev	The sensor device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96792_setup_link(struct device *dev, struct device *s_dev);

/**
 * @brief  Sets up a deserializer link's control pipeline.
 *
 * Puts the deserializer in dual splitter mode. You must call this function
 * during device boot, after max96792_setup_link().
 *
 * @param [in]  dev	The deserializer device handle.
 * @param [in]  s_dev	The sensor device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96792_setup_control(struct device *dev, struct device *s_dev);

/**
 * @brief  Resets a deserializer device's link control pipeline.
 *
 * The deserializer driver internally decrements the reference count and
 * resets the deserializer device if all the source sensor devices are
 * powered off, resetting all control and streaming configuration.
 *
 * @param [in]  dev	The deserializer device handle.
 * @param [in]  s_dev	The sensor device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96792_reset_control(struct device *dev, struct device *s_dev);

/**
 * @brief  Registers a source sensor device with a deserializer device.
 *
 * The deserializer driver internally checks all perquisites and compatibility
 * factors. If it finds that the registration request is valid,
 * it stores the source's @ref gmsl_link_ctx context handle in the source list
 * maintained by the deserializer driver instance.
 *
 * @param [in]  dev	The deserializer device handle.
 * @param [in]  g_ctx	A @c gmsl_link_ctx structure handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96792_sdev_register(struct device *dev, struct gmsl_link_ctx *g_ctx);

/**
 * Unregisters a source sensor device from its deserializer device.
 *
 * @param [in]  dev	The deserializer device handle.
 * @param [in]  s_dev	The sensor device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96792_sdev_unregister(struct device *dev, struct device *s_dev);

/**
 * Performs internal pipeline configuration for a link in context to set up
 * streaming, and puts the deserializer link in ready-to-stream state.
 *
 * @param [in]  dev	The deserializer device handle.
 * @param [in]  s_dev	The sensor device handle.
 *
 * @return  0 or success, or -1 otherwise.
 */
int max96792_setup_streaming(struct device *dev, struct device *s_dev);

/**
 * @brief Enables streaming.
 *
 * This function is to be called by the sensor client driver.
 *
 * @param [in]  dev	The deserializer device handle.
 * @param [in]  s_dev	The sensor device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96792_start_streaming(struct device *dev, struct device *s_dev);

/**
 * @brief Disables streaming.
 *
 * This function is to be called by the sensor client driver.
 *
 * @note  Both @c max96792_start_streaming and @c max96792_stop_streaming
 * are mainly added to enable and disable sensor streaming on the fly
 * while other sensors are active.
 *
 * @param [in]  dev	The deserializer device handle.
 * @param [in]  s_dev	The sensor device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96792_stop_streaming(struct device *dev, struct device *s_dev);

/**
 * @brief  Powers on the max96792 deserializer module.
 *
 * Asserts shared reset GPIO and powers on the regulator;
 * maintains the reference count internally for source devices.
 *
 * @param [in]  dev	The deserializer device handle.
 *
 * @return  0 for success, or -1 otherwise.
 */
int max96792_power_on(struct device *dev, struct gmsl_link_ctx *g_ctx);

/**
 * @brief  Powers off the max96792 deserializer module.
 *
 * Deasserts the shared reset GPIO and powers off the regulator based on
 * the reference count.
 *
 * @param [in]  dev	The deserializer device handle.
 */
void max96792_power_off(struct device *dev, struct gmsl_link_ctx *g_ctx);

int max96792_gmsl3_setup(struct device *dev);

int max96792_xvs_setup(struct device *dev, bool direction);

/**
 * @brief  Sets deserializer clock for different datarates.
 *
 *
 * @param [in]  dev		The deserializer device handle.
 * @param [in]  data_rate	Sensor data rate
 *
 * @return  0 for success, or -1 otherwise.
 */

int max96792_set_deser_clock(struct device *dev, int data_rate);

enum {
	max96792_OUT,
	max96792_IN,
};

/** @} */

#endif /* __MAX96792_H__ */
