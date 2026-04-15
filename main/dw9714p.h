#pragma once

#include <stdint.h>
#include "esp_err.h"

// Dongwoon DW9714 / DW9714P VCM driver. The trailing "P" is a package
// variant only; programming is identical to the plain DW9714 as far as
// all public driver sources (mainline Linux, Intel/ASUS BSPs) are
// concerned.
//
// The chip sits on the shared primary I2C bus (same GPIO9/10 pair as
// the OV5647 SCCB). All functions here claim/release the BSP I2C mutex
// internally so they can be called from the main task without racing
// camera sensor register access.

#define DW9714P_I2C_ADDR 0x0C  // 7-bit, fixed (no strap)

#define DW9714P_POS_MIN  0u
#define DW9714P_POS_MAX  1023u
#define DW9714P_POS_MID  512u

// Probe for the chip by attempting a harmless I2C transaction. Must be
// called before dw9714p_init() / dw9714p_set_position(). Returns
// ESP_OK on ACK, ESP_ERR_NOT_FOUND on NACK, other esp_err_t on bus
// setup failure.
esp_err_t dw9714p_probe(void);

// Send the unlock / DLC / T_SRC / re-lock sequence used by Linux
// DW9714 BSP drivers. Includes a 15 ms delay up front to cover the
// chip's internal offset calibration after power-on.
//
// The mainline Linux driver skips this sequence entirely and works on
// power-on defaults — if direct position writes already move the lens
// on this hardware we may drop this function later.
esp_err_t dw9714p_init(void);

// Set the VCM target position, 0..1023. Values outside the range are
// clamped. Uses direct mode (s-bits = 0).
esp_err_t dw9714p_set_position(uint16_t pos);
