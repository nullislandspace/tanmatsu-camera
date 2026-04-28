/*
 * SPDX-FileCopyrightText: 2026 Tanmatsu Camera contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_cam_sensor_types.h"

// Standard OmniVision SCCB address. OV9281 modules do not expose a
// SID strap, so this is universal across boards (Linux DTs, Arducam
// modules, RPi camera HATs).
#define OV9281_SCCB_ADDR   0x60

// Combined chip-ID returned by reading 0x300A:0x300B. OV9281 and
// OV9282 are register-compatible and both report this same value —
// the suffix is a packaging revision, not a die difference.
#define OV9281_PID         0x9281
#define OV9281_SENSOR_NAME "OV9281"

/**
 * @brief Power on the camera sensor and probe it on the configured SCCB bus.
 *
 * @param[in] config Configuration for power-on, GPIO, and SCCB binding.
 * @return Camera device handle on success, NULL on failure.
 */
esp_cam_sensor_device_t *ov9281_detect(esp_cam_sensor_config_t *config);

#ifdef __cplusplus
}
#endif
