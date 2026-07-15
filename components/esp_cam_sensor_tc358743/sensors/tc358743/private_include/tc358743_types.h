/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Per-format PLL / D-PHY / HDMI-RX config. Field set and meaning are
 * identical to p4kvm's tc358743_cfg_t (p4kvm/main/tc358743.h) — the regs
 * field of esp_cam_sensor_format_t points to one of these.
 */
typedef struct {
    uint32_t refclk_hz;
    uint16_t pll_prd;
    uint16_t pll_fbd;
    uint16_t fifo_level;
    uint32_t lineinitcnt;
    uint32_t lptxtimecnt;
    uint32_t tclk_headercnt;
    uint32_t tclk_trailcnt;
    uint32_t ths_headercnt;
    uint32_t twakeup;
    uint32_t tclk_postcnt;
    uint32_t ths_trailcnt;
    uint32_t hstxvregcnt;
    uint8_t ddc5v_mode;
    unsigned lanes;
    bool enable_hdcp;
    uint8_t hdmi_detection_delay;
} tc358743_format_params_t;

#ifdef __cplusplus
}
#endif
