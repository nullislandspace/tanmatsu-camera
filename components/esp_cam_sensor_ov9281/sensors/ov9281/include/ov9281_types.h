/*
 * SPDX-FileCopyrightText: 2026 Tanmatsu Camera contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * OV9281 SCCB register entry. Mirrors the (uint16_t reg, uint8_t val)
 * shape used by the OV5640/OV5645/OV5647 drivers in esp_cam_sensor so
 * tooling and patterns transfer over directly.
 */
typedef struct {
    uint16_t reg;
    uint8_t  val;
} ov9281_reginfo_t;

#ifdef __cplusplus
}
#endif
