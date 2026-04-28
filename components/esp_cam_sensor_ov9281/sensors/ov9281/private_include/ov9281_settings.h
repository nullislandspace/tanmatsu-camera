/*
 * SPDX-FileCopyrightText: 2026 Tanmatsu Camera contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * OV9281 register init blobs.
 *
 * Init sequence ported verbatim from the Linux mainline driver
 * `drivers/media/i2c/ov9282.c` (v6.10+), which is the only widely
 * deployed and vendor-validated init sequence for this sensor. Both
 * OV9281 and OV9282 dies use this driver in Linux — they are
 * register-compatible.
 *
 * Annotations on each block describe its purpose; exact register
 * meanings are inferred from the Linux driver's source comments and
 * the OmniVision OV9281 product brief. Where a register's purpose is
 * not publicly documented, it is left as a "from datasheet / reverse
 * engineered" magic value preserved as-is.
 *
 * The blob is split into:
 *   - ov9281_mipi_reset_regs       : minimal pre-init sequence (stream
 *                                    off, software reset, brief delay)
 *                                    written once before any format
 *                                    register table.
 *   - ov9281_init_RAW10_1280x800_30fps : full init for monochrome
 *                                    RAW10 at 1280x800 with 2-lane
 *                                    MIPI CSI-2 and a 24 MHz xclk.
 *                                    Line rate 800 Mbps/lane, native
 *                                    fps capped at 30 via VTS=3644.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sdkconfig.h"
#include "ov9281_regs.h"
#include "ov9281_types.h"

// MIPI line rate in Hz (800 Mbps/lane = 800,000,000). Linux's
// OV9282_LINK_FREQ is the DDR clock (400 MHz); the per-lane bit rate
// is 2x that. The esp_cam_sensor framework's mipi_info.mipi_clk field
// expects the line rate in Hz, so we report the doubled value.
#define OV9281_MIPI_LINE_RATE_1280x800_30FPS  (800ULL * 1000 * 1000)

// Native frame rate of ov9281_init_RAW10_1280x800_30fps. With
// pixel_rate = 160 MP/s, HTS = 1456, VTS = 3644 →
// 160e6 / (1456 * 3644) = 30.06 fps. The host-side VTS-cap mechanism
// scales relative to this rate.
#define OV9281_NATIVE_FPS_1280x800            30

// Pre-init: park stream off, software-reset, brief settle. Used at
// the very start of every set_format sequence so the chip starts from
// a known state. Software reset clears every register, so the format
// register table writes the working configuration from scratch.
static const ov9281_reginfo_t ov9281_mipi_reset_regs[] = {
    {OV9281_REG_STREAM,         0x00},
    {OV9281_REG_SOFTWARE_RESET, 0x01},
    {OV9281_REG_DELAY,          0x05},   // wait 5 ms for reset
    {OV9281_REG_END,            0x00},
};

// Full init for monochrome RAW10 1280x800 @ 30 fps, 2-lane MIPI,
// 24 MHz xclk. Streaming is left OFF at the end — host calls
// ov9281_set_stream() to start.
static const ov9281_reginfo_t ov9281_init_RAW10_1280x800_30fps[] = {
    /* PLL: 24 MHz xclk → 400 MHz DDR / 800 Mbps per lane */
    {0x0302, 0x32},
    {0x030d, 0x50},   /* PLL_CTRL_0D — RAW10 path divider */
    {0x030e, 0x02},

    /* GPIO + pad-output disables (parallel data lines off, ILPWM only) */
    {0x3001, 0x00},
    {0x3004, 0x00},
    {0x3005, 0x00},
    {0x3006, 0x04},
    {0x3011, 0x0a},
    {0x3013, 0x18},
    {0x301c, 0xf0},

    /* MIPI PHY + clock tree */
    {0x3022, 0x01},
    {0x3030, 0x10},
    {0x3039, 0x32},
    {0x303a, 0x00},

    /* AEC manual-mode boilerplate (we use real-gain format) */
    {0x3503, 0x08},
    {0x3505, 0x8c},
    {0x3507, 0x03},
    {0x3508, 0x00},   /* analog gain high byte = 0 */

    /* Analog cores — voltage regulators / column buffers */
    {0x3610, 0x80},
    {0x3611, 0xa0},
    {0x3620, 0x6e},
    {0x3632, 0x56},
    {0x3633, 0x78},
    {0x3662, 0x05},   /* ANA_CORE_2 — RAW10 selection */
    {0x3666, 0x00},
    {0x366f, 0x5a},
    {0x3680, 0x84},

    /* Sensor timing tunables */
    {0x3712, 0x80},
    {0x372d, 0x22},
    {0x3731, 0x80},
    {0x3732, 0x30},
    {0x377d, 0x22},
    {0x3788, 0x02},
    {0x3789, 0xa4},
    {0x378a, 0x00},
    {0x378b, 0x4a},
    {0x3799, 0x20},
    {0x3881, 0x42},
    {0x38a8, 0x02},
    {0x38a9, 0x80},
    {0x38b1, 0x00},
    {0x38c4, 0x00},
    {0x38c5, 0xc0},
    {0x38c6, 0x04},
    {0x38c7, 0x80},
    {0x3920, 0xff},

    /* MIPI / ISP packet config */
    {0x4010, 0x40},
    {0x4043, 0x40},
    {0x4307, 0x30},
    {0x4317, 0x00},
    {0x4501, 0x00},
    {0x450a, 0x08},
    {0x4601, 0x04},
    {0x470f, 0x00},
    {0x4f07, 0x00},

    /* ISP enable: bad-pixel correction, white-pixel correction, BLC */
    {0x5000, 0x9f},
    {0x5001, 0x00},
    {0x5e00, 0x00},
    {0x5d00, 0x07},
    {0x5d01, 0x00},

    /* Mirror/flip default + ISP misc */
    {0x0101, 0x01},
    {0x1000, 0x03},
    {0x5a08, 0x84},

    /* Bit-depth select: RAW10 (use 0x60 / 0x07 for RAW8) */
    {0x030d, 0x50},
    {0x3662, 0x05},

    /* Cropping window: 1280 × 800 active, with 8-pixel ISP border on each side */
    {0x3778, 0x00},
    {0x3800, 0x00}, {0x3801, 0x00},   /* X start = 0    */
    {0x3802, 0x00}, {0x3803, 0x00},   /* Y start = 0    */
    {0x3804, 0x05}, {0x3805, 0x0f},   /* X end   = 1295 */
    {0x3806, 0x03}, {0x3807, 0x2f},   /* Y end   = 815  */
    {0x3808, 0x05}, {0x3809, 0x00},   /* output width  = 1280 */
    {0x380a, 0x03}, {0x380b, 0x20},   /* output height = 800  */
    {0x3810, 0x00}, {0x3811, 0x08},   /* ISP X offset = 8 */
    {0x3812, 0x00}, {0x3813, 0x08},   /* ISP Y offset = 8 */
    {0x3814, 0x11}, {0x3815, 0x11},   /* X/Y subsample 1:1 */
    {OV9281_REG_TIMING_FORMAT_1, 0x40},   /* vflip off (bit 6 = sensor orient) */
    {OV9281_REG_TIMING_FORMAT_2, 0x00},   /* hmirror off */
    {0x4003, 0x40},
    {0x4008, 0x04}, {0x4009, 0x0b},   /* BLC dark-row span */
    {0x400c, 0x00}, {0x400d, 0x07},
    {0x4507, 0x00}, {0x4509, 0x00},

    /* Frame timing: HTS = 1456, VTS = 3644 → 30.06 fps native.
     * The app reads VTS back via SCCB after set_format and uses it as
     * the base for frame-rate scaling, so this value is also the
     * reference for any later set_preview_fps() call.
     */
    {OV9281_REG_TIMING_HTS_H,   0x05}, {OV9281_REG_TIMING_HTS_L,   0xb0},
    {OV9281_REG_TIMING_VTS_H,   0x0e}, {OV9281_REG_TIMING_VTS_L,   0x3c},

    /* Default exposure & gain (24-bit exposure in 1/16-line units;
     * default ≈ 0x282 = 642 lines; analog gain 1.0× = 0x10).
     */
    {0x3500, 0x00}, {0x3501, 0x28}, {0x3502, 0x20},
    {0x3509, 0x10},

    {OV9281_REG_END, 0x00},
};

#ifdef __cplusplus
}
#endif
