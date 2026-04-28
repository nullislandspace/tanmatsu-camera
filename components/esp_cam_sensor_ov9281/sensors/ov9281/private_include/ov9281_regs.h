/*
 * SPDX-FileCopyrightText: 2026 Tanmatsu Camera contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Magic register values used by ov9281_write_array() to step through a
// register table — matches the OV5647/OV5640 driver convention.
#define OV9281_REG_DELAY            0xeeee
#define OV9281_REG_END              0xffff

// Standard OmniVision register bank — same addresses as OV5640/45/47.
#define OV9281_REG_SENSOR_ID_H      0x300a
#define OV9281_REG_SENSOR_ID_L      0x300b
#define OV9281_REG_SOFTWARE_RESET   0x0103
#define OV9281_REG_STREAM           0x0100  // bit 0 = stream on/off
#define OV9281_REG_MIPI_CTRL00      0x4800  // bit 5 = clock-lane gate (0=continuous, 1=non-continuous)

// Frame-timing registers — VTS pair is the FPS-cap knob used by the
// app's camera_sensor_set_preview_fps().
#define OV9281_REG_TIMING_HTS_H     0x380c
#define OV9281_REG_TIMING_HTS_L     0x380d
#define OV9281_REG_TIMING_VTS_H     0x380e
#define OV9281_REG_TIMING_VTS_L     0x380f
#define OV9281_REG_TIMING_FORMAT_1  0x3820  // bit 2 = vflip
#define OV9281_REG_TIMING_FORMAT_2  0x3821  // bit 2 = hmirror

#ifdef __cplusplus
}
#endif
