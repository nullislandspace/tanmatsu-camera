/*
 * SPDX-FileCopyrightText: 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * TC358743 — Toshiba HDMI to MIPI CSI-2 bridge driver.
 *
 * Register sequence, values and timing are ported directly from p4kvm's
 * tc358743.c (p4kvm/main/tc358743.c in this repo), confirmed working on
 * the identical Waveshare HDMI-to-CSI module. This file wraps that same
 * logic in the esp_cam_sensor_device_t interface so it plugs into this
 * project's shared sensor framework (camera_sensor.c / camera_pipeline.c)
 * the same way the OV5647/OV5640/OV5645/OV9281 drivers do.
 *
 * p4kvm's two-phase bring-up maps onto esp_cam_sensor_ops_t as:
 *   set_format()  == p4kvm's tc358743_init_streaming()       (HPD stays low)
 *   set_stream(1) == p4kvm's tc358743_enable_hdmi_output()   (raises HPD)
 *   set_stream(0) == p4kvm's tc358743_set_streaming(false)
 *
 * The low-level I2C access differs out of necessity: p4kvm talks to the
 * chip directly over i2c_master_dev_handle_t, while esp_cam_sensor drivers
 * go through the SCCB abstraction (esp_sccb_io_handle_t). SCCB's 16/32-bit
 * register helpers are big-endian on the wire, so multi-byte accesses are
 * pre/post byte-swapped to match the chip's little-endian registers —
 * everything else (register addresses, bitmasks, write values, ordering,
 * delays) is unchanged from p4kvm.
 */

#include "driver/gpio.h"
#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"
#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <inttypes.h>
#include <string.h>

#include "tc358743.h"
#include "tc358743_regs.h"
#include "tc358743_settings.h"
#include "tc358743_types.h"

static const char *TAG = "tc358743";

#ifndef CONFIG_CAMERA_TC358743_MIPI_YUV422_800X480_60FPS
#error "TC358743 requires CAMERA_TC358743_MIPI_YUV422_800X480_60FPS in menuconfig"
#endif

#define delay_ms(ms) vTaskDelay(pdMS_TO_TICKS(ms))

static const esp_cam_sensor_format_t tc358743_format_info[] = {
    {
        .name = "MIPI_2lane_27Minput_YUV422_800x480_60fps",
        .format = ESP_CAM_SENSOR_PIXFORMAT_YUV422,
        .port = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk = 27000000,
        .width = 800,
        .height = 480,
        .regs = &tc358743_params_2lane_800x480_60,
        .regs_size = sizeof(tc358743_format_params_t),
        .fps = 60,
        .isp_info = NULL,
        .mipi_info =
            {
                .mipi_clk = 972000000, /* p4kvm's proven lane rate, far more than 800x480@60 needs */
                .lane_num = 2,
                .line_sync_en = false,
            },
        .reserved = NULL,
    },
};

/* ---- Low-level register I/O ----
 *
 * esp_sccb_transmit_reg_a16v16/32 are big-endian on the wire; the TC358743
 * is little-endian, so pre/post byte-swap every multi-byte access. 8-bit
 * accesses need no correction. */

static esp_err_t tc358743_read8(esp_sccb_io_handle_t h, uint16_t reg, uint8_t *val) {
  return esp_sccb_transmit_receive_reg_a16v8(h, reg, val);
}

static esp_err_t tc358743_read16(esp_sccb_io_handle_t h, uint16_t reg, uint16_t *val) {
  esp_err_t ret = esp_sccb_transmit_receive_reg_a16v16(h, reg, val);
  if (ret == ESP_OK) {
    *val = __builtin_bswap16(*val);
  }
  return ret;
}

static esp_err_t tc358743_write8(esp_sccb_io_handle_t h, uint16_t reg, uint8_t val) {
  return esp_sccb_transmit_reg_a16v8(h, reg, val);
}

static esp_err_t tc358743_write16(esp_sccb_io_handle_t h, uint16_t reg, uint16_t val) {
  return esp_sccb_transmit_reg_a16v16(h, reg, __builtin_bswap16(val));
}

static esp_err_t tc358743_write32(esp_sccb_io_handle_t h, uint16_t reg, uint32_t val) {
  return esp_sccb_transmit_reg_a16v32(h, reg, __builtin_bswap32(val));
}

/* Matches p4kvm's wr8_and_or()/wr16_and_or(): (current & and_mask) | or_val. */
static esp_err_t tc358743_write8_and_or(esp_sccb_io_handle_t h, uint16_t reg,
                                        uint8_t and_mask, uint8_t or_val) {
  uint8_t v = 0;
  esp_err_t ret = tc358743_read8(h, reg, &v);
  if (ret != ESP_OK) {
    return ret;
  }
  return tc358743_write8(h, reg, (uint8_t)((v & and_mask) | or_val));
}

static esp_err_t tc358743_write16_and_or(esp_sccb_io_handle_t h, uint16_t reg,
                                         uint16_t and_mask, uint16_t or_val) {
  uint16_t v = 0;
  esp_err_t ret = tc358743_read16(h, reg, &v);
  if (ret != ESP_OK) {
    return ret;
  }
  return tc358743_write16(h, reg, (uint16_t)((v & and_mask) | or_val));
}

/* ---- Ported from p4kvm/main/tc358743.c ---- */

static void reset_blocks(esp_sccb_io_handle_t h, uint16_t mask) {
  uint16_t sysctl = 0;
  tc358743_read16(h, SYSCTL, &sysctl);
  tc358743_write16(h, SYSCTL, sysctl | mask);
  tc358743_write16(h, SYSCTL, sysctl & ~mask);
}

static void sleep_mode(esp_sccb_io_handle_t h, bool enable) {
  tc358743_write16_and_or(h, SYSCTL, (uint16_t)~MASK_SLEEP, enable ? MASK_SLEEP : 0);
}

static void enable_stream(esp_sccb_io_handle_t h, bool enable) {
  if (enable) {
    /* Non-continuous MIPI clock: TXOPTIONCNTRL is left as set_csi_lanes()
     * left it (0); matches p4kvm / Linux. */
    tc358743_write8(h, VI_MUTE, MASK_AUTO_MUTE);
  } else {
    tc358743_write8(h, VI_MUTE, MASK_AUTO_MUTE | MASK_VI_MUTE);
  }
  tc358743_write16_and_or(h, CONFCTL, (uint16_t) ~(MASK_VBUFEN | MASK_ABUFEN),
                          enable ? (MASK_VBUFEN | MASK_ABUFEN) : 0);
}

static void set_ref_clk(esp_sccb_io_handle_t h, const tc358743_format_params_t *pdata) {
  uint32_t sys_freq = pdata->refclk_hz / 10000;
  tc358743_write8(h, SYS_FREQ0, sys_freq & 0x00ff);
  tc358743_write8(h, SYS_FREQ1, (sys_freq & 0xff00) >> 8);

  tc358743_write8_and_or(h, PHY_CTL0, (uint8_t)~MASK_PHY_SYSCLK_IND,
                         (pdata->refclk_hz == 42000000) ? MASK_PHY_SYSCLK_IND : 0);

  uint16_t fh_min = pdata->refclk_hz / 100000;
  tc358743_write8(h, FH_MIN0, fh_min & 0x00ff);
  tc358743_write8(h, FH_MIN1, (fh_min & 0xff00) >> 8);

  uint16_t fh_max = (uint16_t)((fh_min * 66) / 10);
  tc358743_write8(h, FH_MAX0, fh_max & 0x00ff);
  tc358743_write8(h, FH_MAX1, (fh_max & 0xff00) >> 8);

  uint32_t lockdet_ref = pdata->refclk_hz / 100;
  tc358743_write8(h, LOCKDET_REF0, lockdet_ref & 0x0000ff);
  tc358743_write8(h, LOCKDET_REF1, (lockdet_ref & 0x00ff00) >> 8);
  tc358743_write8(h, LOCKDET_REF2, (lockdet_ref & 0x0f0000) >> 16);

  tc358743_write8_and_or(h, NCO_F0_MOD, (uint8_t)~MASK_NCO_F0_MOD,
                         (pdata->refclk_hz == 27000000) ? MASK_NCO_F0_MOD_27MHZ : 0);

  uint32_t cec_freq = (656u * sys_freq) / 4200u;
  tc358743_write16(h, 0x0028, (uint16_t)cec_freq);
  tc358743_write16(h, 0x002a, (uint16_t)cec_freq);
}

static void set_pll(esp_sccb_io_handle_t h, const tc358743_format_params_t *pdata) {
  uint16_t pllctl0 = 0, pllctl1 = 0;
  tc358743_read16(h, PLLCTL0, &pllctl0);
  tc358743_read16(h, PLLCTL1, &pllctl1);
  uint16_t pllctl0_new = SET_PLL_PRD(pdata->pll_prd) | SET_PLL_FBD(pdata->pll_fbd);
  uint32_t hsck = (pdata->refclk_hz / pdata->pll_prd) * pdata->pll_fbd;

  if ((pllctl0 != pllctl0_new) || ((pllctl1 & MASK_PLL_EN) == 0)) {
    uint16_t pll_frs;
    if (hsck > 500000000) {
      pll_frs = 0x0;
    } else if (hsck > 250000000) {
      pll_frs = 0x1;
    } else if (hsck > 125000000) {
      pll_frs = 0x2;
    } else {
      pll_frs = 0x3;
    }

    sleep_mode(h, true);
    tc358743_write16(h, PLLCTL0, pllctl0_new);
    tc358743_write16_and_or(h, PLLCTL1, (uint16_t) ~(MASK_PLL_FRS | MASK_RESETB | MASK_PLL_EN),
                            SET_PLL_FRS(pll_frs) | MASK_RESETB | MASK_PLL_EN);
    delay_ms(1);
    tc358743_write16_and_or(h, PLLCTL1, (uint16_t)~MASK_CKEN, MASK_CKEN);
    sleep_mode(h, false);
  }
}

static void set_hdmi_hdcp(esp_sccb_io_handle_t h, bool enable) {
  if (enable) {
    return;
  }
  tc358743_write8_and_or(h, HDCP_MODE, (uint8_t)~MASK_MANUAL_AUTHENTICATION,
                         MASK_MANUAL_AUTHENTICATION);
}

static void set_hdmi_phy(esp_sccb_io_handle_t h, const tc358743_format_params_t *pdata) {
  tc358743_write8_and_or(h, PHY_EN, (uint8_t)~MASK_ENABLE_PHY, 0);
  tc358743_write8(h, PHY_CTL1, SET_PHY_AUTO_RST1_US(1600) | SET_FREQ_RANGE_MODE_CYCLES(1));
  /* All PHY auto-resets disabled — matches p4kvm's waveshare_pi defaults
   * (every hdmi_phy_auto_reset_* flag false). */
  tc358743_write8_and_or(h, PHY_CTL2, (uint8_t)~MASK_PHY_AUTO_RSTn, 0);
  tc358743_write8(h, PHY_BIAS, 0x40);
  tc358743_write8(h, PHY_CSQ, SET_CSQ_CNT_LEVEL(0x0a));
  tc358743_write8(h, AVM_CTL, 45);
  tc358743_write8_and_or(h, HDMI_DET, (uint8_t)~MASK_HDMI_DET_V,
                         (uint8_t)(pdata->hdmi_detection_delay << 4));
  tc358743_write8_and_or(h, HV_RST, (uint8_t) ~(MASK_H_PI_RST | MASK_V_PI_RST), 0);
  tc358743_write8_and_or(h, PHY_EN, (uint8_t)~MASK_ENABLE_PHY, MASK_ENABLE_PHY);
}

static void set_hdmi_audio(esp_sccb_io_handle_t h) {
  tc358743_write8(h, FORCE_MUTE, 0x00);
  tc358743_write8(h, AUTO_CMD0,
                  MASK_AUTO_MUTE7 | MASK_AUTO_MUTE6 | MASK_AUTO_MUTE5 | MASK_AUTO_MUTE4 |
                      MASK_AUTO_MUTE1 | MASK_AUTO_MUTE0);
  tc358743_write8(h, AUTO_CMD1, MASK_AUTO_MUTE9);
  tc358743_write8(h, AUTO_CMD2, MASK_AUTO_PLAY3 | MASK_AUTO_PLAY2);
  tc358743_write8(h, BUFINIT_START, SET_BUFINIT_START_MS(500));
  tc358743_write8(h, FS_MUTE, 0x00);
  tc358743_write8(h, FS_IMODE, MASK_NLPCM_SMODE | MASK_FS_SMODE);
  tc358743_write8(h, ACR_MODE, MASK_CTS_MODE);
  tc358743_write8(h, ACR_MDF0, MASK_ACR_L2MDF_1976_PPM | MASK_ACR_L1MDF_976_PPM);
  tc358743_write8(h, ACR_MDF1, MASK_ACR_L3MDF_3906_PPM);
  tc358743_write8(h, SDO_MODE1, MASK_SDO_FMT_I2S);
  tc358743_write8(h, DIV_MODE, SET_DIV_DLY_MS(100));
  tc358743_write16_and_or(h, CONFCTL, 0xffff,
                          MASK_AUDCHNUM_2 | MASK_AUDOUTSEL_I2S | MASK_AUTOINDEX);
}

static void set_hdmi_info_frame(esp_sccb_io_handle_t h) {
  tc358743_write8(h, PK_INT_MODE,
                  MASK_ISRC2_INT_MODE | MASK_ISRC_INT_MODE | MASK_ACP_INT_MODE |
                      MASK_VS_INT_MODE | MASK_SPD_INT_MODE | MASK_MS_INT_MODE |
                      MASK_AUD_INT_MODE | MASK_AVI_INT_MODE);
  tc358743_write8(h, NO_PKT_LIMIT, 0x2c);
  tc358743_write8(h, NO_PKT_CLR, 0x53);
  tc358743_write8(h, ERR_PK_LIMIT, 0x01);
  tc358743_write8(h, NO_PKT_LIMIT2, 0x30);
  tc358743_write8(h, NO_GDB_LIMIT, 0x10);
}

static void initial_setup(esp_sccb_io_handle_t h, const tc358743_format_params_t *pdata) {
  tc358743_write16_and_or(h, SYSCTL, (uint16_t) ~(MASK_IRRST | MASK_CECRST),
                          MASK_IRRST | MASK_CECRST);
  reset_blocks(h, MASK_CTXRST | MASK_HDMIRST);
  sleep_mode(h, false);

  tc358743_write16(h, FIFOCTL, pdata->fifo_level);
  set_ref_clk(h, pdata);
  tc358743_write8_and_or(h, DDC_CTL, (uint8_t)~MASK_DDC5V_MODE,
                         pdata->ddc5v_mode & MASK_DDC5V_MODE);
  tc358743_write8_and_or(h, EDID_MODE, (uint8_t)~MASK_EDID_MODE, MASK_EDID_MODE_E_DDC);

  set_hdmi_phy(h, pdata);
  set_hdmi_hdcp(h, pdata->enable_hdcp);
  set_hdmi_audio(h);
  set_hdmi_info_frame(h);

  tc358743_write8_and_or(h, VI_MODE, (uint8_t)~MASK_RGB_DVI, 0);
  tc358743_write8_and_or(h, VOUT_SET2, (uint8_t)~MASK_VOUTCOLORMODE, MASK_VOUTCOLORMODE_AUTO);
  tc358743_write8(h, VOUT_SET3, MASK_VOUT_EXTCNT);
}

/** RGB888: Linux tc358743_set_csi_color_space(RGB888_1X24). Unused by this
 * driver (Tanmatsu's pipeline wants YUV422) but kept for parity with p4kvm. */
static void set_csi_color_space_rgb888_regs(esp_sccb_io_handle_t h) {
  tc358743_write8_and_or(h, VOUT_SET2, (uint8_t) ~(MASK_SEL422 | MASK_VOUT_422FIL_100), 0);
  tc358743_write8_and_or(h, VI_REP, (uint8_t)~MASK_VOUT_COLOR_SEL, MASK_VOUT_COLOR_RGB_FULL);
  tc358743_write16_and_or(h, CONFCTL, (uint16_t)~MASK_YCBCRFMT, 0);
}

/** UYVY 16-bit: Linux tc358743_set_csi_color_space(MEDIA_BUS_FMT_UYVY8_1X16). */
static void set_csi_color_space_uyvy422_regs(esp_sccb_io_handle_t h) {
  tc358743_write8_and_or(h, VOUT_SET2, (uint8_t) ~(MASK_SEL422 | MASK_VOUT_422FIL_100),
                         (uint8_t)(MASK_SEL422 | MASK_VOUT_422FIL_100));
  tc358743_write8_and_or(h, VI_REP, (uint8_t)~MASK_VOUT_COLOR_SEL,
                         MASK_VOUT_COLOR_601_YCBCR_LIMITED);
  tc358743_write16_and_or(h, CONFCTL, (uint16_t)~MASK_YCBCRFMT, MASK_YCBCRFMT_422_8_BIT);
}

static void set_csi_lanes(esp_sccb_io_handle_t h, const tc358743_format_params_t *pdata) {
  unsigned lanes = pdata->lanes;

  reset_blocks(h, MASK_CTXRST);

  if (lanes < 1) {
    tc358743_write32(h, CLW_CNTRL, MASK_CLW_LANEDISABLE);
  }
  if (lanes < 1) {
    tc358743_write32(h, D0W_CNTRL, MASK_D0W_LANEDISABLE);
  }
  if (lanes < 2) {
    tc358743_write32(h, D1W_CNTRL, MASK_D1W_LANEDISABLE);
  }
  if (lanes < 3) {
    tc358743_write32(h, D2W_CNTRL, MASK_D2W_LANEDISABLE);
  }
  if (lanes < 4) {
    tc358743_write32(h, D3W_CNTRL, MASK_D3W_LANEDISABLE);
  }

  tc358743_write32(h, LINEINITCNT, pdata->lineinitcnt);
  tc358743_write32(h, LPTXTIMECNT, pdata->lptxtimecnt);
  tc358743_write32(h, TCLK_HEADERCNT, pdata->tclk_headercnt);
  tc358743_write32(h, TCLK_TRAILCNT, pdata->tclk_trailcnt);
  tc358743_write32(h, THS_HEADERCNT, pdata->ths_headercnt);
  tc358743_write32(h, TWAKEUP, pdata->twakeup);
  tc358743_write32(h, TCLK_POSTCNT, pdata->tclk_postcnt);
  tc358743_write32(h, THS_TRAILCNT, pdata->ths_trailcnt);
  tc358743_write32(h, HSTXVREGCNT, pdata->hstxvregcnt);

  tc358743_write32(h, HSTXVREGEN,
                   ((lanes > 0) ? MASK_CLM_HSTXVREGEN : 0) |
                       ((lanes > 0) ? MASK_D0M_HSTXVREGEN : 0) |
                       ((lanes > 1) ? MASK_D1M_HSTXVREGEN : 0) |
                       ((lanes > 2) ? MASK_D2M_HSTXVREGEN : 0) |
                       ((lanes > 3) ? MASK_D3M_HSTXVREGEN : 0));

  /* Non-continuous MIPI clock — matches p4kvm / Linux tc358743 set_csi(). */
  tc358743_write32(h, TXOPTIONCNTRL, 0);
  tc358743_write32(h, STARTCNTRL, MASK_START);
  tc358743_write32(h, CSI_START, MASK_STRT);

  uint32_t nol = (lanes == 4)   ? MASK_NOL_4
                 : (lanes == 3) ? MASK_NOL_3
                 : (lanes == 2) ? MASK_NOL_2
                                : MASK_NOL_1;

  tc358743_write32(h, CSI_CONFW,
                   MASK_MODE_SET | MASK_ADDRESS_CSI_CONTROL | MASK_CSI_MODE | MASK_TXHSMD | nol);
  tc358743_write32(h, CSI_CONFW,
                   MASK_MODE_SET | MASK_ADDRESS_CSI_ERR_INTENA | MASK_TXBRK | MASK_QUNK |
                       MASK_WCER | MASK_INER);
  tc358743_write32(h, CSI_CONFW,
                   MASK_MODE_CLEAR | MASK_ADDRESS_CSI_ERR_HALT | MASK_TXBRK | MASK_QUNK);
  tc358743_write32(h, CSI_CONFW, MASK_MODE_SET | MASK_ADDRESS_CSI_INT_ENA | MASK_INTER);
}

static void hpd_set(esp_sccb_io_handle_t h, bool on) {
  tc358743_write8_and_or(h, HPD_CTL, (uint8_t)~MASK_HPD_OUT0, on ? MASK_HPD_OUT0 : 0);
}

/*
 * Load EDID into internal RAM (HPD must stay low, source must not DDC during
 * this). Caller raises HPD after PLL/CSI and any other sink setup.
 *
 * p4kvm writes this in 128-byte block transfers over raw i2c_master; the
 * SCCB abstraction used here only exposes single-register writes, so this
 * is byte-at-a-time instead. Slower (once, at init, so inconsequential) but
 * produces the identical EDID RAM contents.
 */
static void edid_write_builtin(esp_sccb_io_handle_t h) {
  const uint16_t edid_len = sizeof(tc358743_default_edid);
  tc358743_write8(h, EDID_LEN1, edid_len & 0xff);
  tc358743_write8(h, EDID_LEN2, edid_len >> 8);
  for (uint16_t i = 0; i < edid_len; i++) {
    tc358743_write8(h, (uint16_t)(EDID_RAM + i), tc358743_default_edid[i]);
  }
  delay_ms(10);
}

static void log_sys_status(esp_sccb_io_handle_t h) {
  uint8_t st = 0;
  if (tc358743_read8(h, SYS_STATUS, &st) != ESP_OK) {
    return;
  }
  ESP_LOGI(TAG, "SYS_STATUS=0x%02x (TMDS=%d HDMI=%d SYNC=%d DDC5V=%d)", st,
           (int)(st >> 1) & 1, (int)(st >> 4) & 1, (int)(st >> 7) & 1, (int)st & 1);
}

/* ---- GPIO power-on/reset ----
 *
 * Delays match p4kvm's tc358743_resetn_pulse() (p4kvm/main/capture_hw.c):
 * with a dedicated reset line, 200 ms after release; without one (this
 * exact HDMI-to-CSI module, un-reset, on Tanmatsu), wait out the chip's
 * internal power-on-reset before any I2C traffic. */

static esp_err_t tc358743_hw_reset(esp_cam_sensor_device_t *dev) {
  if (dev->reset_pin >= 0) {
    gpio_set_level(dev->reset_pin, 0);
    delay_ms(10);
    gpio_set_level(dev->reset_pin, 1);
    delay_ms(10);
  }
  return ESP_OK;
}

static esp_err_t tc358743_soft_reset(esp_cam_sensor_device_t *dev) {
  uint16_t sysctl = MASK_IRRST | MASK_CECRST;
  esp_err_t ret =
      tc358743_write16(dev->sccb_handle, SYSCTL, sysctl | MASK_CTXRST | MASK_HDMIRST);
  if (ret == ESP_OK) {
    delay_ms(1);
    ret = tc358743_write16(dev->sccb_handle, SYSCTL, sysctl);
  }
  return ret;
}

static esp_err_t tc358743_power_on(esp_cam_sensor_device_t *dev) {
  if (dev->pwdn_pin >= 0) {
    gpio_config_t conf = {0};
    conf.pin_bit_mask = 1ULL << dev->pwdn_pin;
    conf.mode = GPIO_MODE_OUTPUT;
    esp_err_t ret = gpio_config(&conf);
    if (ret != ESP_OK) {
      return ret;
    }
    /* power-down is active high → drive low to power on */
    gpio_set_level(dev->pwdn_pin, 1);
    delay_ms(10);
    gpio_set_level(dev->pwdn_pin, 0);
    delay_ms(10);
  }

  if (dev->reset_pin >= 0) {
    gpio_config_t conf = {0};
    conf.pin_bit_mask = 1ULL << dev->reset_pin;
    conf.mode = GPIO_MODE_OUTPUT;
    esp_err_t ret = gpio_config(&conf);
    if (ret != ESP_OK) {
      return ret;
    }
    gpio_set_level(dev->reset_pin, 0);
    delay_ms(10);
    gpio_set_level(dev->reset_pin, 1);
    delay_ms(200);
  } else {
    ESP_LOGW(TAG, "No reset pin wired — waiting 500 ms for internal POR");
    delay_ms(500);
  }

  return ESP_OK;
}

static esp_err_t tc358743_power_off(esp_cam_sensor_device_t *dev) {
  if (dev->pwdn_pin >= 0) {
    gpio_set_level(dev->pwdn_pin, 0);
    delay_ms(10);
    gpio_set_level(dev->pwdn_pin, 1);
    delay_ms(10);
  }
  if (dev->reset_pin >= 0) {
    gpio_set_level(dev->reset_pin, 1);
    delay_ms(10);
    gpio_set_level(dev->reset_pin, 0);
    delay_ms(10);
  }
  return ESP_OK;
}

/* ---- esp_cam_sensor_ops_t implementation ---- */

static esp_err_t tc358743_get_sensor_id(esp_cam_sensor_device_t *dev,
                                        esp_cam_sensor_id_t *id) {
  /* p4kvm: "0x0000 is common and not used for probe" — the real chip and
   * the Linux mainline driver both expect CHIPID's upper byte to read 0.
   * Detection is based on the I2C transaction succeeding, not the value. */
  uint16_t chipid = 0;
  esp_err_t ret = tc358743_read16(dev->sccb_handle, CHIPID, &chipid);
  ESP_LOGI(TAG, "CHIPID=0x%04x ret=%d", chipid, ret);
  if (ret != ESP_OK) {
    return ret;
  }
  id->pid = chipid;
  return ESP_OK;
}

static esp_err_t tc358743_query_para_desc(esp_cam_sensor_device_t *dev,
                                          esp_cam_sensor_param_desc_t *qdesc) {
  ESP_LOGD(TAG, "id=%" PRIx32 " not supported", qdesc->id);
  return ESP_ERR_INVALID_ARG;
}

static esp_err_t tc358743_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id,
                                         void *arg, size_t size) {
  return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t tc358743_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id,
                                         const void *arg, size_t size) {
  return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t tc358743_query_support_formats(esp_cam_sensor_device_t *dev,
                                                esp_cam_sensor_format_array_t *formats) {
  ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
  ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
  formats->count = ARRAY_SIZE(tc358743_format_info);
  formats->format_array = &tc358743_format_info[0];
  return ESP_OK;
}

static esp_err_t
tc358743_query_support_capability(esp_cam_sensor_device_t *dev,
                                  esp_cam_sensor_capability_t *sensor_cap) {
  ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
  ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, sensor_cap);
  sensor_cap->fmt_yuv = 1;
  return ESP_OK;
}

/* p4kvm's tc358743_init_streaming(): HPD stays low throughout — the source
 * must not see a hotplug edge (and start reading EDID / sending TMDS) until
 * PLL, CSI-2 lanes and the color-space are all configured. HPD itself is
 * raised later, from set_stream(1) (p4kvm's tc358743_enable_hdmi_output()). */
static esp_err_t tc358743_set_format(esp_cam_sensor_device_t *dev,
                                     const esp_cam_sensor_format_t *format) {
  ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);

  if (format == NULL) {
    format = &tc358743_format_info[0];
  }
  const tc358743_format_params_t *pdata = (const tc358743_format_params_t *)format->regs;
  if (pdata == NULL) {
    ESP_LOGE(TAG, "No format params in format->regs");
    return ESP_ERR_INVALID_ARG;
  }

  esp_sccb_io_handle_t h = dev->sccb_handle;

  hpd_set(h, false);
  delay_ms(20);

  initial_setup(h, pdata);
  edid_write_builtin(h);

  enable_stream(h, false);
  set_pll(h, pdata);
  set_csi_lanes(h, pdata);
  set_csi_color_space_uyvy422_regs(h);

  tc358743_write16(h, INTSTATUS, 0xffff);
  tc358743_write16(h, INTMASK, (uint16_t)(~(MASK_HDMI_MSK | MASK_CSI_MSK) & 0xffff));

  dev->cur_format = format;
  ESP_LOGI(TAG, "Format set: %s", format->name);
  return ESP_OK;
}

static esp_err_t tc358743_get_format(esp_cam_sensor_device_t *dev,
                                     esp_cam_sensor_format_t *format) {
  ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
  ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);
  if (dev->cur_format == NULL) {
    return ESP_FAIL;
  }
  memcpy(format, dev->cur_format, sizeof(esp_cam_sensor_format_t));
  return ESP_OK;
}

static esp_err_t tc358743_set_stream(esp_cam_sensor_device_t *dev, int enable) {
  ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
  esp_sccb_io_handle_t h = dev->sccb_handle;

  if (enable) {
    /* p4kvm's tc358743_enable_hdmi_output(): video FIFO on, then a
     * delayed hotplug edge, then re-kick CSI_START — some boards leave
     * CSI TX idle until this is rewritten once CONFCTL enables video. */
    enable_stream(h, true);
    delay_ms(150);
    /* Deviation from p4kvm: INIT_END must be written before HPD_CTL on
     * this hardware, or the HPD write silently doesn't take effect. See
     * the comment on INIT_END in tc358743_regs.h. */
    tc358743_write8(h, INIT_END, MASK_INIT_END);
    hpd_set(h, true);
    delay_ms(50);
    tc358743_write32(h, CSI_START, MASK_STRT);
    log_sys_status(h);
  } else {
    /* p4kvm's tc358743_set_streaming(false): mute + disable the video
     * buffer, then reset/reconfigure the CSI-2 TX block. */
    enable_stream(h, false);
    const tc358743_format_params_t *pdata =
        dev->cur_format ? (const tc358743_format_params_t *)dev->cur_format->regs : NULL;
    if (pdata != NULL) {
      set_csi_lanes(h, pdata);
    }
  }

  dev->stream_status = enable;
  ESP_LOGI(TAG, "Stream %s", enable ? "enabled" : "disabled");
  return ESP_OK;
}

static esp_err_t tc358743_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd,
                                     void *arg) {
  esp_err_t ret = ESP_OK;
  esp_cam_sensor_reg_val_t *sensor_reg;

  switch (cmd) {
  case ESP_CAM_SENSOR_IOC_HW_RESET:
    ret = tc358743_hw_reset(dev);
    break;
  case ESP_CAM_SENSOR_IOC_SW_RESET:
    ret = tc358743_soft_reset(dev);
    break;
  case ESP_CAM_SENSOR_IOC_S_STREAM:
    ret = tc358743_set_stream(dev, *(int *)arg);
    break;
  case ESP_CAM_SENSOR_IOC_S_REG:
    sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
    ret = tc358743_write16(dev->sccb_handle, (uint16_t)sensor_reg->regaddr,
                           (uint16_t)sensor_reg->value);
    break;
  case ESP_CAM_SENSOR_IOC_G_REG: {
    sensor_reg = (esp_cam_sensor_reg_val_t *)arg;
    uint16_t val = 0;
    ret = tc358743_read16(dev->sccb_handle, (uint16_t)sensor_reg->regaddr, &val);
    if (ret == ESP_OK) {
      sensor_reg->value = val;
    }
    break;
  }
  case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
    ret = tc358743_get_sensor_id(dev, arg);
    break;
  default:
    ret = ESP_ERR_NOT_SUPPORTED;
    break;
  }
  return ret;
}

static esp_err_t tc358743_delete(esp_cam_sensor_device_t *dev) {
  ESP_LOGD(TAG, "del tc358743 (%p)", dev);
  if (dev) {
    tc358743_power_off(dev);
    free(dev);
  }
  return ESP_OK;
}

static const esp_cam_sensor_ops_t tc358743_ops = {
    .query_para_desc = tc358743_query_para_desc,
    .get_para_value = tc358743_get_para_value,
    .set_para_value = tc358743_set_para_value,
    .query_support_formats = tc358743_query_support_formats,
    .query_support_capability = tc358743_query_support_capability,
    .set_format = tc358743_set_format,
    .get_format = tc358743_get_format,
    .priv_ioctl = tc358743_priv_ioctl,
    .del = tc358743_delete,
};

/* ---- Public detect function ---- */

esp_cam_sensor_device_t *tc358743_detect(esp_cam_sensor_config_t *config) {
  if (config == NULL) {
    return NULL;
  }

  esp_cam_sensor_device_t *dev = calloc(1, sizeof(esp_cam_sensor_device_t));
  if (dev == NULL) {
    ESP_LOGE(TAG, "No memory for device");
    return NULL;
  }

  dev->name = (char *)TC358743_SENSOR_NAME;
  dev->sccb_handle = config->sccb_handle;
  dev->xclk_pin = config->xclk_pin;
  dev->reset_pin = config->reset_pin;
  dev->pwdn_pin = config->pwdn_pin;
  dev->sensor_port = config->sensor_port;
  dev->ops = &tc358743_ops;
  dev->cur_format = &tc358743_format_info[0];

  if (tc358743_power_on(dev) != ESP_OK) {
    ESP_LOGE(TAG, "Power on failed");
    goto err_free;
  }

  if (tc358743_get_sensor_id(dev, &dev->id) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read chip ID");
    goto err_power_off;
  }

  ESP_LOGI(TAG, "Detected TC358743, CHIPID=0x%04x", dev->id.pid);
  return dev;

err_power_off:
  tc358743_power_off(dev);
err_free:
  free(dev);
  return NULL;
}

#if CONFIG_CAMERA_TC358743_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(tc358743_detect, ESP_CAM_SENSOR_MIPI_CSI, TC358743_SCCB_ADDR) {
  ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
  return tc358743_detect(config);
}
#endif
