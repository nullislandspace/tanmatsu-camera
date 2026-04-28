/*
 * SPDX-FileCopyrightText: 2026 Tanmatsu Camera contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * OmniVision OV9281 / OV9282 monochrome global-shutter MIPI CSI-2
 * camera sensor driver, written to plug into the Espressif
 * esp_cam_sensor framework via ESP_CAM_SENSOR_DETECT_FN auto-detect.
 *
 * The driver follows the same shape as the in-tree OV5647 driver in
 * managed_components/espressif__esp_cam_sensor: opaque device handle
 * carrying the SCCB I/O, an ops struct exposing query/set_format /
 * S_STREAM / mirror / flip / chip-id, and a single detect entry point
 * that powers the chip on, validates the chip-id, and returns the
 * device handle.
 *
 * Init register tables live in ov9281_settings.h and are ported from
 * the Linux mainline ov9282.c driver. See that file for source notes.
 *
 * NOTE: implemented without OV9281 hardware on hand. The init blob,
 * power-on sequence, and chip-id readback come from authoritative
 * sources (Linux kernel + libcamera) and follow the established OV
 * register conventions, but this code path has not been bench-verified.
 */

#include <inttypes.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_detect.h"

#include "ov9281.h"
#include "ov9281_settings.h"

#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif
#define delay_ms(ms)  vTaskDelay((ms > portTICK_PERIOD_MS ? (ms) / portTICK_PERIOD_MS : 1))

#define ARRAY_SIZE(a)  (sizeof(a) / sizeof((a)[0]))

static const char *TAG = "ov9281";

// One MIPI CSI format for now: 1280x800 RAW10 @ 30 fps native, 2-lane.
// Extra resolutions (binned 640x400, RAW8 high-fps modes) can be added
// later by appending entries here — the framework iterates the array
// for query_support_formats() so additions are zero-touch on the host.
static const esp_cam_sensor_format_t ov9281_format_info[] = {
    {
        .name       = "MIPI_2lane_24Minput_RAW10_1280x800_30fps",
        .format     = ESP_CAM_SENSOR_PIXFORMAT_RAW10,
        .port       = ESP_CAM_SENSOR_MIPI_CSI,
        .xclk       = 24000000,
        .width      = 1280,
        .height     = 800,
        .regs       = ov9281_init_RAW10_1280x800_30fps,
        .regs_size  = ARRAY_SIZE(ov9281_init_RAW10_1280x800_30fps),
        .fps        = OV9281_NATIVE_FPS_1280x800,
        // OV9281 is monochrome — no Bayer pattern. Leave isp_info NULL
        // so the host's ISP layer treats this as "no demosaic info
        // available". On the ESP32-P4 the ISP demosaicer will still
        // run (the ESP-IDF requires it for the CSI bridge to route
        // pixels to memory), but downstream output will be roughly
        // grayscale because every input pixel carries the same
        // luminance signal regardless of which Bayer position the
        // demosaicer thinks it occupies.
        .isp_info   = NULL,
        .mipi_info  = {
            .mipi_clk     = OV9281_MIPI_LINE_RATE_1280x800_30FPS,
            .lane_num     = 2,
            .line_sync_en = CONFIG_CAMERA_OV9281_CSI_LINESYNC_ENABLE ? true : false,
        },
        .reserved   = NULL,
    },
};

static esp_err_t ov9281_read(esp_sccb_io_handle_t sccb, uint16_t reg, uint8_t *out)
{
    return esp_sccb_transmit_receive_reg_a16v8(sccb, reg, out);
}

static esp_err_t ov9281_write(esp_sccb_io_handle_t sccb, uint16_t reg, uint8_t val)
{
    return esp_sccb_transmit_reg_a16v8(sccb, reg, val);
}

// Walk a register table terminated by OV9281_REG_END. Entries with
// reg == OV9281_REG_DELAY are interpreted as "sleep val ms" instead
// of an SCCB write.
static esp_err_t ov9281_write_array(esp_sccb_io_handle_t sccb, const ov9281_reginfo_t *table)
{
    esp_err_t ret = ESP_OK;
    int       i   = 0;
    while (ret == ESP_OK && table[i].reg != OV9281_REG_END) {
        if (table[i].reg == OV9281_REG_DELAY) {
            delay_ms(table[i].val);
        } else {
            ret = ov9281_write(sccb, table[i].reg, table[i].val);
        }
        i++;
    }
    ESP_LOGD(TAG, "write_array: %d entries, ret=%d", i, ret);
    return ret;
}

// Read-modify-write a register-bitfield. `offset` is the LSB of the
// field; `length` is its bit-width.
static esp_err_t ov9281_set_reg_bits(esp_sccb_io_handle_t sccb, uint16_t reg,
                                     uint8_t offset, uint8_t length, uint8_t value)
{
    uint8_t cur = 0;
    esp_err_t ret = ov9281_read(sccb, reg, &cur);
    if (ret != ESP_OK) return ret;
    uint8_t mask = ((1u << length) - 1u) << offset;
    uint8_t neu  = (cur & ~mask) | ((value << offset) & mask);
    return ov9281_write(sccb, reg, neu);
}

static esp_err_t ov9281_hw_reset(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }
    return ESP_OK;
}

static esp_err_t ov9281_soft_reset(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ov9281_set_reg_bits(dev->sccb_handle, OV9281_REG_SOFTWARE_RESET, 0, 1, 0x01);
    delay_ms(5);
    return ret;
}

static esp_err_t ov9281_get_sensor_id(esp_cam_sensor_device_t *dev, esp_cam_sensor_id_t *id)
{
    uint8_t pid_h = 0, pid_l = 0;
    esp_err_t ret = ov9281_read(dev->sccb_handle, OV9281_REG_SENSOR_ID_H, &pid_h);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read pid_h failed");
    ret = ov9281_read(dev->sccb_handle, OV9281_REG_SENSOR_ID_L, &pid_l);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "read pid_l failed");

    uint16_t pid = ((uint16_t)pid_h << 8) | pid_l;
    if (pid) {
        id->pid = pid;
    }
    return ret;
}

static esp_err_t ov9281_set_stream(esp_cam_sensor_device_t *dev, int enable)
{
    // Configure MIPI continuous-clock mode before unmuting the data
    // path. Linux's driver writes 0x00 here for continuous clock and
    // 0x20 for non-continuous (gated) clock; we use continuous because
    // the ESP32-P4 CSI receiver's PHY is happier with a free-running
    // clock lane.
    esp_err_t ret = ov9281_write(dev->sccb_handle, OV9281_REG_MIPI_CTRL00, 0x00);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "MIPI_CTRL00 write failed");

    ret = ov9281_write(dev->sccb_handle, OV9281_REG_STREAM, enable ? 0x01 : 0x00);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "stream %d write failed", enable);

    dev->stream_status = enable;
    ESP_LOGD(TAG, "stream=%d", enable);
    return ESP_OK;
}

static esp_err_t ov9281_set_mirror(esp_cam_sensor_device_t *dev, int enable)
{
    // Bit 2 of TIMING_FORMAT_2 (0x3821) flips the readout horizontally.
    return ov9281_set_reg_bits(dev->sccb_handle, OV9281_REG_TIMING_FORMAT_2, 2, 1, enable ? 0x01 : 0x00);
}

static esp_err_t ov9281_set_vflip(esp_cam_sensor_device_t *dev, int enable)
{
    // Bit 2 of TIMING_FORMAT_1 (0x3820) flips the readout vertically.
    return ov9281_set_reg_bits(dev->sccb_handle, OV9281_REG_TIMING_FORMAT_1, 2, 1, enable ? 0x01 : 0x00);
}

static esp_err_t ov9281_query_support_formats(esp_cam_sensor_device_t *dev,
                                              esp_cam_sensor_format_array_t *formats)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, formats);
    formats->count        = ARRAY_SIZE(ov9281_format_info);
    formats->format_array = &ov9281_format_info[0];
    return ESP_OK;
}

static esp_err_t ov9281_query_support_capability(esp_cam_sensor_device_t *dev,
                                                 esp_cam_sensor_capability_t *cap)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, cap);
    // OV9281 emits raw 10-bit (or 8-bit) monochrome only — no on-chip
    // demosaic and no JPEG. We expose this as fmt_raw to keep the
    // existing host pipeline path; downstream code is responsible for
    // knowing the data is monochrome rather than Bayer.
    cap->fmt_raw = 1;
    return ESP_OK;
}

static esp_err_t ov9281_set_format(esp_cam_sensor_device_t *dev, const esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);

    // Default to the Kconfig-selected entry if the caller didn't pass
    // one explicitly. Matches the OV5647/OV5640 driver convention so
    // app code can call set_format(NULL) immediately after detect.
    if (format == NULL) {
        if (dev->sensor_port == ESP_CAM_SENSOR_MIPI_CSI) {
            format = &ov9281_format_info[CONFIG_CAMERA_OV9281_MIPI_IF_FORMAT_INDEX_DEFAULT];
        } else {
            ESP_LOGE(TAG, "non-MIPI port not supported");
            return ESP_ERR_NOT_SUPPORTED;
        }
    }

    esp_err_t ret = ov9281_write_array(dev->sccb_handle, ov9281_mipi_reset_regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "reset regs write failed");

    ret = ov9281_write_array(dev->sccb_handle, (const ov9281_reginfo_t *)format->regs);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "format regs write failed");

    // Leave streaming OFF — the host calls S_STREAM when the CSI
    // pipeline is primed and ready to receive frames.
    ret = ov9281_set_stream(dev, 0);
    ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "stream-off after format failed");

    dev->cur_format = format;
    ESP_LOGI(TAG, "format set: %s", format->name);
    return ESP_OK;
}

static esp_err_t ov9281_get_format(esp_cam_sensor_device_t *dev, esp_cam_sensor_format_t *format)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, format);
    if (dev->cur_format == NULL) return ESP_ERR_INVALID_STATE;
    memcpy(format, dev->cur_format, sizeof(*format));
    return ESP_OK;
}

static esp_err_t ov9281_query_para_desc(esp_cam_sensor_device_t *dev,
                                        esp_cam_sensor_param_desc_t *qdesc)
{
    switch (qdesc->id) {
    case ESP_CAM_SENSOR_VFLIP:
    case ESP_CAM_SENSOR_HMIRROR:
        qdesc->type            = ESP_CAM_SENSOR_PARAM_TYPE_NUMBER;
        qdesc->number.minimum  = 0;
        qdesc->number.maximum  = 1;
        qdesc->number.step     = 1;
        qdesc->default_value   = 0;
        return ESP_OK;
    default:
        ESP_LOGD(TAG, "param id 0x%" PRIx32 " not supported", qdesc->id);
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t ov9281_get_para_value(esp_cam_sensor_device_t *dev, uint32_t id,
                                       void *arg, size_t size)
{
    (void)dev; (void)id; (void)arg; (void)size;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t ov9281_set_para_value(esp_cam_sensor_device_t *dev, uint32_t id,
                                       const void *arg, size_t size)
{
    (void)size;
    switch (id) {
    case ESP_CAM_SENSOR_VFLIP:
        return ov9281_set_vflip(dev, *(const int *)arg);
    case ESP_CAM_SENSOR_HMIRROR:
        return ov9281_set_mirror(dev, *(const int *)arg);
    default:
        ESP_LOGE(TAG, "set param 0x%" PRIx32 " not supported", id);
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t ov9281_priv_ioctl(esp_cam_sensor_device_t *dev, uint32_t cmd, void *arg)
{
    ESP_CAM_SENSOR_NULL_POINTER_CHECK(TAG, dev);
    esp_cam_sensor_reg_val_t *rv;
    uint8_t reg_v = 0;
    esp_err_t ret;

    switch (cmd) {
    case ESP_CAM_SENSOR_IOC_HW_RESET:
        return ov9281_hw_reset(dev);
    case ESP_CAM_SENSOR_IOC_SW_RESET:
        return ov9281_soft_reset(dev);
    case ESP_CAM_SENSOR_IOC_S_STREAM:
        return ov9281_set_stream(dev, *(int *)arg);
    case ESP_CAM_SENSOR_IOC_G_REG:
        rv  = (esp_cam_sensor_reg_val_t *)arg;
        ret = ov9281_read(dev->sccb_handle, rv->regaddr, &reg_v);
        if (ret == ESP_OK) rv->value = reg_v;
        return ret;
    case ESP_CAM_SENSOR_IOC_S_REG:
        rv = (esp_cam_sensor_reg_val_t *)arg;
        return ov9281_write(dev->sccb_handle, rv->regaddr, rv->value);
    case ESP_CAM_SENSOR_IOC_G_CHIP_ID:
        return ov9281_get_sensor_id(dev, arg);
    default:
        return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t ov9281_power_on(esp_cam_sensor_device_t *dev)
{
    esp_err_t ret = ESP_OK;

    if (dev->pwdn_pin >= 0) {
        gpio_config_t conf = {
            .pin_bit_mask = 1ULL << dev->pwdn_pin,
            .mode         = GPIO_MODE_OUTPUT,
        };
        ret = gpio_config(&conf);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "pwdn pin config failed");
        // OV9281 has no separate PWDN pin; if a board wires one it's
        // typically active-high standby. Drive low to keep the chip
        // out of standby.
        gpio_set_level(dev->pwdn_pin, 0);
        delay_ms(1);
    }

    if (dev->reset_pin >= 0) {
        gpio_config_t conf = {
            .pin_bit_mask = 1ULL << dev->reset_pin,
            .mode         = GPIO_MODE_OUTPUT,
        };
        ret = gpio_config(&conf);
        ESP_RETURN_ON_FALSE(ret == ESP_OK, ret, TAG, "reset pin config failed");

        // Pulse RESETB low for 10 ms, then release. Datasheet says the
        // PHY needs ≥400 µs after RESETB de-assert before the first
        // SCCB transaction; 10 ms gives generous margin.
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(10);
        gpio_set_level(dev->reset_pin, 1);
        delay_ms(10);
    }

    return ret;
}

static esp_err_t ov9281_power_off(esp_cam_sensor_device_t *dev)
{
    if (dev->reset_pin >= 0) {
        gpio_set_level(dev->reset_pin, 0);
        delay_ms(2);
    }
    if (dev->pwdn_pin >= 0) {
        gpio_set_level(dev->pwdn_pin, 1);
        delay_ms(2);
    }
    return ESP_OK;
}

static esp_err_t ov9281_delete(esp_cam_sensor_device_t *dev)
{
    ESP_LOGD(TAG, "del ov9281 (%p)", dev);
    if (dev) free(dev);
    return ESP_OK;
}

static const esp_cam_sensor_ops_t ov9281_ops = {
    .query_para_desc          = ov9281_query_para_desc,
    .get_para_value           = ov9281_get_para_value,
    .set_para_value           = ov9281_set_para_value,
    .query_support_formats    = ov9281_query_support_formats,
    .query_support_capability = ov9281_query_support_capability,
    .set_format               = ov9281_set_format,
    .get_format               = ov9281_get_format,
    .priv_ioctl               = ov9281_priv_ioctl,
    .del                      = ov9281_delete,
};

esp_cam_sensor_device_t *ov9281_detect(esp_cam_sensor_config_t *config)
{
    if (config == NULL) return NULL;

    esp_cam_sensor_device_t *dev = calloc(1, sizeof(*dev));
    if (dev == NULL) {
        ESP_LOGE(TAG, "no memory for device handle");
        return NULL;
    }

    dev->name        = (char *)OV9281_SENSOR_NAME;
    dev->sccb_handle = config->sccb_handle;
    dev->xclk_pin    = config->xclk_pin;
    dev->reset_pin   = config->reset_pin;
    dev->pwdn_pin    = config->pwdn_pin;
    dev->sensor_port = config->sensor_port;
    dev->ops         = &ov9281_ops;

    if (config->sensor_port != ESP_CAM_SENSOR_MIPI_CSI) {
        ESP_LOGE(TAG, "only MIPI CSI port is supported");
        goto fail;
    }
    dev->cur_format = &ov9281_format_info[CONFIG_CAMERA_OV9281_MIPI_IF_FORMAT_INDEX_DEFAULT];

    if (ov9281_power_on(dev) != ESP_OK) {
        ESP_LOGE(TAG, "power-on failed");
        goto fail;
    }

    if (ov9281_get_sensor_id(dev, &dev->id) != ESP_OK) {
        ESP_LOGE(TAG, "chip-id read failed");
        goto fail_off;
    }
    if (dev->id.pid != OV9281_PID) {
        // The detect loop probes each sensor's SCCB address in turn,
        // and an unrelated sensor at 0x60 (or stale bus state) will
        // return an unexpected PID. Returning NULL lets the loop move
        // on to the next driver — this is the normal "wrong sensor at
        // this address" path, not a real error.
        ESP_LOGD(TAG, "chip-id mismatch: got 0x%04x, want 0x%04x",
                 dev->id.pid, OV9281_PID);
        goto fail_off;
    }
    ESP_LOGI(TAG, "detected OV9281/OV9282, PID=0x%04x", dev->id.pid);
    return dev;

fail_off:
    ov9281_power_off(dev);
fail:
    free(dev);
    return NULL;
}

#if CONFIG_CAMERA_OV9281_AUTO_DETECT_MIPI_INTERFACE_SENSOR
ESP_CAM_SENSOR_DETECT_FN(ov9281_detect, ESP_CAM_SENSOR_MIPI_CSI, OV9281_SCCB_ADDR)
{
    ((esp_cam_sensor_config_t *)config)->sensor_port = ESP_CAM_SENSOR_MIPI_CSI;
    return ov9281_detect(config);
}
#endif
