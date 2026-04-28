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

// Per-sensor preview / video format names. preview is what mode PHOTO
// (and VIEW, which reuses the preview as its backdrop) streams at;
// video is what mode VIDEO streams at. On the OV5647 the two are
// different — preview is the high-res RAW10 1920x1080 (every shutter
// press is a snapshot of the live frame, full WYSIWYG) and video
// drops to 800x640 RAW8 to stay inside the PPA throughput budget for
// realtime encoding. On OV5640/OV5645 we use one shared RGB565 format
// for all modes: those sensors deliver finished pixels directly, the
// resolutions are already modest, and the PPA budget is fine.
#define PREVIEW_FORMAT_OV5647   "MIPI_2lane_24Minput_RAW10_1920x1080_30fps"
#define VIDEO_FORMAT_OV5647     "MIPI_2lane_24Minput_RAW8_800x640_50fps"
#define SHARED_FORMAT_OV5640    "MIPI_2lane_24Minput_RGB565_1280x720_14fps"
#define SHARED_FORMAT_OV5645    "MIPI_2lane_24Minput_RGB565_1280x960_30fps"

// OmniVision Timing Group VTS (vertical total size / frame length in
// lines) register pair. Identical on OV5640, OV5645 and OV5647 — it is
// part of the standard 5MP OV timing register bank. Writing a larger
// value extends the vertical blanking interval and therefore slows the
// frame rate down without touching the PCLK. Max exposure line count
// tracks VTS automatically on these sensors so the built-in AE loop
// gets more headroom rather than less.
//
// We treat the VTS-register write as the only working frame-rate cap:
// previous attempts to limit fps via driver ioctls or alternate format
// presets did not actually slow the OV5647. Read the active format's
// nominal fps and the driver-just-written VTS register pair via SCCB
// after every set_format_*() call so the same scaling math works for
// any sensor + format pair without hardcoded base constants.
#define OV_REG_TIMING_VTS_H     0x380E
#define OV_REG_TIMING_VTS_L     0x380F

// Map a driver-supplied sensor name string ("OV5647", "OV5640", ...)
// onto our enum. Unknown names leave kind=UNKNOWN; the caller treats
// that as "best-effort OV5647-style RAW pipeline" since the detect
// loop already proved the sensor responds to SCCB and the standard OV
// VTS bank.
static camera_sensor_kind_t name_to_kind(const char *name) {
    if (name == NULL) return CAMERA_SENSOR_UNKNOWN;
    if (strcmp(name, "OV5647") == 0) return CAMERA_SENSOR_OV5647;
    if (strcmp(name, "OV5640") == 0) return CAMERA_SENSOR_OV5640;
    if (strcmp(name, "OV5645") == 0) return CAMERA_SENSOR_OV5645;
    return CAMERA_SENSOR_UNKNOWN;
}

// Read VTS H/L back from the sensor and stash both that and the
// active format's reported fps onto the sensor handle, so a later
// camera_sensor_set_preview_fps() can scale relative to whatever the
// driver just wrote rather than carrying a hardcoded OV5647-specific
// base value. Failures are non-fatal — they just leave fps override
// unavailable for the current format.
static void capture_fps_base(camera_sensor_t *sensor, const esp_cam_sensor_format_t *fmt) {
    if (sensor == NULL || fmt == NULL) return;
    sensor->base_fps        = fmt->fps;
    sensor->base_vts_lines  = 0;

    uint8_t vts_h = 0, vts_l = 0;
    if (camera_sensor_read_reg(sensor, OV_REG_TIMING_VTS_H, &vts_h) != ESP_OK) return;
    if (camera_sensor_read_reg(sensor, OV_REG_TIMING_VTS_L, &vts_l) != ESP_OK) return;
    sensor->base_vts_lines = ((uint32_t)vts_h << 8) | vts_l;
}

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

    out->device         = device;
    out->sccb           = sccb;
    out->kind           = name_to_kind(device->name);
    out->base_vts_lines = 0;
    out->base_fps       = 0;
    ESP_LOGI(TAG, "Camera sensor detected: %s (kind=%d)",
             device->name ? device->name : "?", (int)out->kind);
    return ESP_OK;
}

const char *camera_sensor_name(const camera_sensor_t *sensor) {
    if (sensor == NULL || sensor->device == NULL || sensor->device->name == NULL) {
        return "?";
    }
    return sensor->device->name;
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
    capture_fps_base(sensor, match);
    if (out_fmt) {
        *out_fmt = *match;
    }
    return ESP_OK;
}

// Pick the format-name string for the requested mode based on which
// sensor the detect loop bound to. Returns NULL if the mode is not
// supported on this sensor (caller should surface an error).
static const char *format_name_for(const camera_sensor_t *sensor, bool video) {
    if (sensor == NULL) return NULL;
    switch (sensor->kind) {
        case CAMERA_SENSOR_OV5647:
            return video ? VIDEO_FORMAT_OV5647 : PREVIEW_FORMAT_OV5647;
        case CAMERA_SENSOR_OV5640:
            // Single shared RGB565 format — OV5640 only ships one MIPI
            // mode. video / preview return the same string so the
            // PHOTO↔VIDEO transition is effectively a no-op.
            return SHARED_FORMAT_OV5640;
        case CAMERA_SENSOR_OV5645:
            return SHARED_FORMAT_OV5645;
        case CAMERA_SENSOR_UNKNOWN:
        default:
            // Unknown sensor — fall back to the OV5647 names. If the
            // detected sensor doesn't actually advertise these formats
            // the set_format query below will return ESP_ERR_NOT_FOUND
            // and the caller can surface a "format not supported"
            // error, which is honest about what's happening.
            return video ? VIDEO_FORMAT_OV5647 : PREVIEW_FORMAT_OV5647;
    }
}

esp_err_t camera_sensor_set_format_preview(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt) {
    const char *name = format_name_for(sensor, false);
    if (name == NULL) return ESP_ERR_NOT_SUPPORTED;
    return camera_sensor_set_format_by_name(sensor, name, out_fmt);
}

esp_err_t camera_sensor_set_format_video(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt) {
    const char *name = format_name_for(sensor, true);
    if (name == NULL) return ESP_ERR_NOT_SUPPORTED;
    return camera_sensor_set_format_by_name(sensor, name, out_fmt);
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
    capture_fps_base(sensor, best);
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
    if (sensor == NULL || target_fps == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sensor->base_vts_lines == 0 || sensor->base_fps == 0) {
        // No active format captured yet — caller forgot to call
        // camera_sensor_set_format_*() first, or the VTS read-back
        // failed silently inside capture_fps_base().
        return ESP_ERR_INVALID_STATE;
    }
    if (target_fps > sensor->base_fps) {
        // We can only slow the sensor down by lengthening VTS.
        // Speeding it up would need a HTS cut or a new format preset.
        return ESP_ERR_INVALID_ARG;
    }
    if (target_fps == sensor->base_fps) {
        // No-op: target matches native fps. Don't write the register
        // — that way "cap to 15" on a 14fps sensor (OV5640 RGB565) is
        // a clean no-op rather than a redundant SCCB write.
        return ESP_OK;
    }

    // new_vts = base_vts * (base_fps / target_fps), rounded.
    uint32_t vts = (sensor->base_vts_lines * sensor->base_fps + target_fps / 2u) / target_fps;
    if (vts > 0xFFFFu) vts = 0xFFFFu;

    esp_err_t err = camera_sensor_write_reg(sensor, OV_REG_TIMING_VTS_H, (uint8_t)((vts >> 8) & 0xFFu));
    if (err != ESP_OK) return err;
    err = camera_sensor_write_reg(sensor, OV_REG_TIMING_VTS_L, (uint8_t)(vts & 0xFFu));
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "preview fps override: %" PRIu32 " fps (VTS=%" PRIu32 ", base=%" PRIu32 "@%" PRIu32 "fps)",
             target_fps, vts, sensor->base_vts_lines, sensor->base_fps);
    return ESP_OK;
}
