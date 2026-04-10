#include "camera_sensor.h"

#include <inttypes.h>
#include <stddef.h>
#include <string.h>

#include "bsp/i2c.h"
#include "driver/i2c_master.h"
#include "esp_cam_sensor_detect.h"
#include "esp_log.h"
#include "esp_sccb_i2c.h"

static const char *TAG = "camera_sensor";

#define SCCB_FREQ_HZ            100000
// Unified format: preview and photo capture both run on the sensor's
// highest-resolution MIPI mode. The preview pipeline PPA-scales this
// into a letterboxed 16:9 display image, and photo capture is a
// "stop sensor, grab current frame, start sensor" snapshot of the same
// buffer — so the preview is always a true WYSIWYG preview of what
// the shutter will save, with zero format-switch latency and no need
// for AE/AWB snapshot/restore dance.
#define PREVIEW_FORMAT_NAME     "MIPI_2lane_24Minput_RAW10_1920x1080_30fps"

// OmniVision Timing Group VTS (vertical total size / frame length in
// lines) register pair. Identical on OV5640, OV5645 and OV5647 — it is
// part of the standard 5MP OV timing register bank. Writing a larger
// value extends the vertical blanking interval and therefore slows the
// frame rate down without touching the PCLK. Max exposure line count
// tracks VTS automatically on these sensors so the built-in AE loop
// gets more headroom rather than less.
#define OV_REG_TIMING_VTS_H     0x380E
#define OV_REG_TIMING_VTS_L     0x380F

// Base VTS and nominal frame rate for the 1920x1080 RAW10 preset:
// ov5647_settings.h:492-493 writes VTS=1199 with a nominal 30 fps.
// The 800x640 preset was empirically ~28% slow vs its nominal, so
// 30 fps here might actually be closer to 21-22 fps in practice —
// recalibrate after measuring the fin counter with the target VTS
// applied.
#define PREVIEW_BASE_VTS_LINES  1199u
#define PREVIEW_BASE_FPS        30u

esp_err_t camera_sensor_detect(camera_sensor_t *out) {
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(*out));

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t               err = bsp_i2c_primary_bus_get_handle(&bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bsp_i2c_primary_bus_get_handle: %d", err);
        return err;
    }

    esp_cam_sensor_config_t cam_config = {
        .sccb_handle = NULL,
        .reset_pin   = -1,
        .pwdn_pin    = -1,
        .xclk_pin    = -1,
        .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
    };

    esp_cam_sensor_device_t *device = NULL;
    esp_sccb_io_handle_t     sccb   = NULL;

    bsp_i2c_primary_bus_claim();
    for (esp_cam_sensor_detect_fn_t *p = &__esp_cam_sensor_detect_fn_array_start;
         p < &__esp_cam_sensor_detect_fn_array_end; ++p) {
        sccb_i2c_config_t i2c_cfg = {
            .scl_speed_hz    = SCCB_FREQ_HZ,
            .device_address  = p->sccb_addr,
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        };
        if (sccb_new_i2c_io(bus, &i2c_cfg, &sccb) != ESP_OK) {
            continue;
        }
        cam_config.sccb_handle = sccb;
        cam_config.sensor_port = p->port;

        device = (*(p->detect))(&cam_config);
        if (device) {
            break;
        }

        esp_sccb_del_i2c_io(sccb);
        sccb                   = NULL;
        cam_config.sccb_handle = NULL;
    }
    bsp_i2c_primary_bus_release();

    if (!device) {
        ESP_LOGE(TAG, "No camera sensor detected on the primary I2C bus");
        return ESP_ERR_NOT_FOUND;
    }

    out->device = device;
    out->sccb   = sccb;
    ESP_LOGI(TAG, "Camera sensor detected");
    return ESP_OK;
}

void camera_sensor_release(camera_sensor_t *sensor) {
    if (sensor == NULL) return;
    if (sensor->sccb) {
        bsp_i2c_primary_bus_claim();
        esp_sccb_del_i2c_io(sensor->sccb);
        bsp_i2c_primary_bus_release();
    }
    sensor->sccb   = NULL;
    sensor->device = NULL;
}

esp_err_t camera_sensor_set_format_by_name(camera_sensor_t *sensor, const char *exact_name,
                                           esp_cam_sensor_format_t *out_fmt) {
    if (sensor == NULL || sensor->device == NULL || exact_name == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_cam_sensor_format_array_t fmt_array = {0};
    bsp_i2c_primary_bus_claim();
    esp_err_t err = esp_cam_sensor_query_format(sensor->device, &fmt_array);
    bsp_i2c_primary_bus_release();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "query_format: %d", err);
        return err;
    }

    const esp_cam_sensor_format_t *match = NULL;
    for (uint32_t i = 0; i < fmt_array.count; ++i) {
        if (fmt_array.format_array[i].port == ESP_CAM_SENSOR_MIPI_CSI &&
            strcmp(fmt_array.format_array[i].name, exact_name) == 0) {
            match = &fmt_array.format_array[i];
            break;
        }
    }

    if (!match) {
        ESP_LOGE(TAG, "Format '%s' not reported by sensor", exact_name);
        return ESP_ERR_NOT_FOUND;
    }

    bsp_i2c_primary_bus_claim();
    err = esp_cam_sensor_set_format(sensor->device, match);
    bsp_i2c_primary_bus_release();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_format('%s'): %d", exact_name, err);
        return err;
    }

    ESP_LOGI(TAG, "Format set: %s (%" PRIu32 "x%" PRIu32 ")", match->name, match->width, match->height);
    if (out_fmt) {
        *out_fmt = *match;
    }
    return ESP_OK;
}

esp_err_t camera_sensor_set_format_preview(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt) {
    return camera_sensor_set_format_by_name(sensor, PREVIEW_FORMAT_NAME, out_fmt);
}

esp_err_t camera_sensor_set_format_photo(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt) {
    if (sensor == NULL || sensor->device == NULL) return ESP_ERR_INVALID_ARG;

    // Walk the supported format list and pick the highest-resolution MIPI CSI
    // entry. This is version-tolerant: the OV5647 driver has historically
    // reported the maximum format as 1920x1080 RAW10.
    esp_cam_sensor_format_array_t fmt_array = {0};
    bsp_i2c_primary_bus_claim();
    esp_err_t err = esp_cam_sensor_query_format(sensor->device, &fmt_array);
    bsp_i2c_primary_bus_release();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "query_format: %d", err);
        return err;
    }

    const esp_cam_sensor_format_t *best = NULL;
    for (uint32_t i = 0; i < fmt_array.count; ++i) {
        const esp_cam_sensor_format_t *f = &fmt_array.format_array[i];
        if (f->port != ESP_CAM_SENSOR_MIPI_CSI) continue;
        if (best == NULL || (uint64_t)f->width * f->height > (uint64_t)best->width * best->height) {
            best = f;
        }
    }

    if (!best) {
        ESP_LOGE(TAG, "No MIPI CSI formats reported by sensor");
        return ESP_ERR_NOT_FOUND;
    }

    bsp_i2c_primary_bus_claim();
    err = esp_cam_sensor_set_format(sensor->device, best);
    bsp_i2c_primary_bus_release();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_format(photo '%s'): %d", best->name, err);
        return err;
    }

    ESP_LOGI(TAG, "Photo format: %s (%" PRIu32 "x%" PRIu32 ")", best->name, best->width, best->height);
    if (out_fmt) {
        *out_fmt = *best;
    }
    return ESP_OK;
}

esp_err_t camera_sensor_stream(camera_sensor_t *sensor, bool enable) {
    if (sensor == NULL || sensor->device == NULL) return ESP_ERR_INVALID_ARG;
    int flag = enable ? 1 : 0;

    bsp_i2c_primary_bus_claim();
    esp_err_t err = esp_cam_sensor_ioctl(sensor->device, ESP_CAM_SENSOR_IOC_S_STREAM, &flag);
    bsp_i2c_primary_bus_release();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "S_STREAM=%d: %d", flag, err);
    }
    return err;
}

esp_err_t camera_sensor_read_reg(camera_sensor_t *sensor, uint16_t regaddr, uint8_t *out_val) {
    if (sensor == NULL || sensor->device == NULL || out_val == NULL) return ESP_ERR_INVALID_ARG;
    esp_cam_sensor_reg_val_t rv = { .regaddr = regaddr, .value = 0 };
    bsp_i2c_primary_bus_claim();
    esp_err_t err = esp_cam_sensor_ioctl(sensor->device, ESP_CAM_SENSOR_IOC_G_REG, &rv);
    bsp_i2c_primary_bus_release();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "G_REG 0x%04x: %d", regaddr, err);
        return err;
    }
    *out_val = (uint8_t)(rv.value & 0xFF);
    return ESP_OK;
}

esp_err_t camera_sensor_write_reg(camera_sensor_t *sensor, uint16_t regaddr, uint8_t value) {
    if (sensor == NULL || sensor->device == NULL) return ESP_ERR_INVALID_ARG;
    esp_cam_sensor_reg_val_t rv = { .regaddr = regaddr, .value = value };
    bsp_i2c_primary_bus_claim();
    esp_err_t err = esp_cam_sensor_ioctl(sensor->device, ESP_CAM_SENSOR_IOC_S_REG, &rv);
    bsp_i2c_primary_bus_release();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "S_REG 0x%04x=0x%02x: %d", regaddr, value, err);
    }
    return err;
}

esp_err_t camera_sensor_set_preview_fps(camera_sensor_t *sensor, uint32_t target_fps) {
    if (sensor == NULL || target_fps == 0 || target_fps > PREVIEW_BASE_FPS) {
        return ESP_ERR_INVALID_ARG;
    }

    // new_vts = base_vts * (base_fps / target_fps), rounded.
    uint32_t vts = (PREVIEW_BASE_VTS_LINES * PREVIEW_BASE_FPS + target_fps / 2u) / target_fps;
    if (vts > 0xFFFFu) vts = 0xFFFFu;

    esp_err_t err = camera_sensor_write_reg(sensor, OV_REG_TIMING_VTS_H, (uint8_t)((vts >> 8) & 0xFFu));
    if (err != ESP_OK) return err;
    err = camera_sensor_write_reg(sensor, OV_REG_TIMING_VTS_L, (uint8_t)(vts & 0xFFu));
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "preview fps override: %" PRIu32 " fps (VTS=%" PRIu32 ")", target_fps, vts);
    return ESP_OK;
}
