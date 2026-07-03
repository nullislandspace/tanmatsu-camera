/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_cam_sensor_types.h"

#define TC358743_SCCB_ADDR   0x0F  /* 7-bit I2C address (ADDR pin low) */
#define TC358743_PID         0x0000
#define TC358743_SENSOR_NAME "TC358743"

/**
 * @brief Power on the TC358743 and verify it is present on the SCCB bus.
 *
 * Full chip initialisation (HDMI PHY, PLL, CSI-2 TX, EDID) is deferred to
 * the first set_format() call, matching p4kvm's tc358743_init_streaming()
 * — HPD stays low until set_stream(1) (p4kvm's tc358743_enable_hdmi_output()).
 *
 * @param[in] config  Power-on and SCCB configuration.
 * @return Camera device handle on success, NULL on failure.
 */
esp_cam_sensor_device_t *tc358743_detect(esp_cam_sensor_config_t *config);

#ifdef __cplusplus
}
#endif
