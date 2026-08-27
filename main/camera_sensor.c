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
// OV9281 ships a single MIPI format. Same string is used for preview
// and video, identical handling to the OV5640/45 RGB565 sensors —
// PHOTO↔VIDEO is a no-op format-wise.
#define SHARED_FORMAT_OV9281    "MIPI_2lane_24Minput_RAW10_1280x800_30fps"
// The TC358743 ships exactly one format, and its EDID advertises the
// matching mode as the only one it supports, so the HDMI source is
// told to send precisely this. Preview and video share it.
#define SHARED_FORMAT_TC358743  "MIPI_2lane_27Minput_YUV422_800x480_60fps"

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

// Standard OmniVision AEC/AGC bank. Exposure is a value in 1/16-line
// units spread over 0x3500..0x3502 (so the register holds lines << 4,
// the low nibble being a fractional row); analog gain is "real gain"
// fixed-point with 4 fractional bits, 0x10 = 1.0x.
//
// The gain register WIDTH is the one thing that is not common across
// these parts, so it is table-driven below rather than assumed: the
// 5MP sensors carry a 10-bit gain in 0x350A[1:0]:0x350B, while the
// OV9281 has a single 8-bit gain at 0x3509 and nothing at 0x350A/B.
// Writing the 5MP layout to an OV9281 would scribble over an unrelated
// register, which is exactly the kind of bug that presents as "the
// image is fine but the sensor locks up after a minute".
#define OV_REG_AEC_MODE         0x3503  // bit0 = AEC manual, bit1 = AGC manual
#define OV_REG_EXP_H            0x3500
#define OV_REG_EXP_M            0x3501
#define OV_REG_EXP_L            0x3502
#define OV_AEC_MANUAL_BITS      0x03    // AEC + AGC both under host control

// Per-sensor description of that bank.
typedef struct {
    // Does this part carry the OmniVision timing/AEC/AGC bank at all?
    // False means every 0x35xx / 0x380x access below is not merely
    // meaningless but unsafe: camera_sensor_read_reg/write_reg go
    // through ESP_CAM_SENSOR_IOC_G_REG/S_REG, which each driver
    // implements against its own address space. On the TC358743 that
    // is a live 16-bit write, so "exposure register 0x3503" lands on
    // whatever the Toshiba bridge happens to keep at 0x3503.
    bool     has_ov_regs;
    bool     has_auto_ae;  // on-chip AE loop that must be switched off first
    uint16_t gain_reg_h;   // 0 = gain is a single byte at gain_reg_l
    uint16_t gain_reg_l;
    uint16_t gain_max_q4;  // largest analog gain the register can express
    uint16_t exp_margin;   // exposure ceiling is (VTS - this) lines
} sensor_ae_caps_t;

static sensor_ae_caps_t ae_caps(camera_sensor_kind_t kind) {
    switch (kind) {
        case CAMERA_SENSOR_OV9281:
            // Machine-vision part: no AE loop at all, 8-bit real gain
            // at 0x3509 (0x10 = 1.0x .. 0xFF = 15.94x). The exposure
            // ceiling of VTS-12 matches the mainline Linux ov9282
            // driver's OV9282_EXPOSURE_OFFSET.
            return (sensor_ae_caps_t){ true, false, 0x0000, 0x3509, 0x00FF, 12 };
        case CAMERA_SENSOR_TC358743:
            // An HDMI receiver, not an imager. There is no exposure,
            // no gain and no frame-length bank to speak of, and its
            // register space is a different map entirely.
            return (sensor_ae_caps_t){ false, false, 0x0000, 0x0000, 0x0000, 0 };
        case CAMERA_SENSOR_OV5647:
        case CAMERA_SENSOR_OV5640:
        case CAMERA_SENSOR_OV5645:
        case CAMERA_SENSOR_UNKNOWN:
        default:
            // 10-bit gain (max 63.9x). Unknown sensors land here too:
            // the detect loop already proved this chip answers to the
            // standard OV register bank, and the 5MP layout is the one
            // every OV part except the OV9281 uses.
            return (sensor_ae_caps_t){ true, true, 0x350A, 0x350B, 0x03FF, 4 };
    }
}

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
    if (strcmp(name, "OV9281") == 0) return CAMERA_SENSOR_OV9281;
    if (strcmp(name, "TC358743") == 0) return CAMERA_SENSOR_TC358743;
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

    sensor->cur_vts_lines   = 0;
    sensor->row_time_ns     = 0;

    // Nothing below applies to a part without the OV bank. Leaving
    // base_vts_lines and row_time_ns at 0 is also what makes the
    // exposure and fps entry points refuse cleanly further down.
    if (!ae_caps(sensor->kind).has_ov_regs) return;

    uint8_t vts_h = 0, vts_l = 0;
    if (camera_sensor_read_reg(sensor, OV_REG_TIMING_VTS_H, &vts_h) != ESP_OK) return;
    if (camera_sensor_read_reg(sensor, OV_REG_TIMING_VTS_L, &vts_l) != ESP_OK) return;
    sensor->base_vts_lines = ((uint32_t)vts_h << 8) | vts_l;
    sensor->cur_vts_lines  = sensor->base_vts_lines;

    // A frame is base_vts_lines rows long and lasts 1/base_fps seconds,
    // so one row takes 1e9 / (fps * VTS) ns. Deriving it this way means
    // we never have to know the PLL tree, the PCLK or HTS for any given
    // sensor — and it stays correct when a later VTS override stretches
    // the frame, because that adds blank rows without changing how long
    // a row takes.
    if (sensor->base_fps > 0 && sensor->base_vts_lines > 0) {
        sensor->row_time_ns = (uint32_t)(1000000000ULL /
            ((uint64_t)sensor->base_fps * (uint64_t)sensor->base_vts_lines));
    }
}

// The bridge is identified by its SCCB address rather than by its
// detect function pointer, because ESP_CAM_SENSOR_DETECT_FN registers a
// static wrapper -- the address in the table is not the public
// tc358743_detect at all, so comparing against it would silently never
// match. 0x0F is unique among the registered camera drivers (OV5647
// 0x36, OV5640/45 0x3C, OV9281 0x60) and among the other devices on
// this bus (coprocessor 0x5F, BMI270 0x68, ES8156 0x08).
#define TC358743_DETECT_ADDR 0x0F

esp_err_t camera_sensor_detect(camera_sensor_t *out) {
    return camera_sensor_detect_scoped(out, CAMERA_DETECT_ORDINARY);
}

esp_err_t camera_sensor_detect_scoped(camera_sensor_t *out,
                                      camera_detect_scope_t scope) {
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
        const bool is_bridge = (p->sccb_addr == TC358743_DETECT_ADDR);
        if (is_bridge != (scope == CAMERA_DETECT_BRIDGE)) {
            continue;
        }

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
        if (scope == CAMERA_DETECT_BRIDGE) {
            ESP_LOGW(TAG, "No HDMI bridge on the primary I2C bus either");
        } else {
            ESP_LOGE(TAG, "No camera sensor detected on the primary I2C bus");
        }
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
        case CAMERA_SENSOR_OV9281:
            // Monochrome RAW10 1280x800. Same format string for both
            // modes — OV9281 has no dedicated low-res video preset.
            return SHARED_FORMAT_OV9281;
        case CAMERA_SENSOR_TC358743:
            return SHARED_FORMAT_TC358743;
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
    if (!ae_caps(sensor->kind).has_ov_regs) {
        // No VTS bank to stretch. The frame rate of a bridge chip is
        // whatever its upstream source sends.
        return ESP_ERR_NOT_SUPPORTED;
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
        sensor->cur_vts_lines = sensor->base_vts_lines;
        return ESP_OK;
    }

    // new_vts = base_vts * (base_fps / target_fps), rounded.
    uint32_t vts = (sensor->base_vts_lines * sensor->base_fps + target_fps / 2u) / target_fps;
    if (vts > 0xFFFFu) vts = 0xFFFFu;

    esp_err_t err = camera_sensor_write_reg(sensor, OV_REG_TIMING_VTS_H, (uint8_t)((vts >> 8) & 0xFFu));
    if (err != ESP_OK) return err;
    err = camera_sensor_write_reg(sensor, OV_REG_TIMING_VTS_L, (uint8_t)(vts & 0xFFu));
    if (err != ESP_OK) return err;
    sensor->cur_vts_lines = vts;

    ESP_LOGI(TAG, "preview fps override: %" PRIu32 " fps (VTS=%" PRIu32 ", base=%" PRIu32 "@%" PRIu32 "fps)",
             target_fps, vts, sensor->base_vts_lines, sensor->base_fps);
    return ESP_OK;
}

// ---------------------------------------------------------------------
// Manual exposure / analog gain
// ---------------------------------------------------------------------

// Darkest rung of the ladder, expressed as a light budget in
// "microseconds at unity gain". Each step doubles it, so step N costs
// BRIGHT_BASE_EG_US << (N-1) and the whole ladder spans 2^15.
#define BRIGHT_BASE_EG_US       30u

static uint32_t bright_eg_us(int step) {
    if (step < CAMERA_BRIGHT_MIN) step = CAMERA_BRIGHT_MIN;
    if (step > CAMERA_BRIGHT_MAX) step = CAMERA_BRIGHT_MAX;
    return BRIGHT_BASE_EG_US << (step - CAMERA_BRIGHT_MIN);
}

// Longest integration the current frame period allows. Note this reads
// cur_vts_lines, not base_vts_lines: capping the preview to 15 fps
// doubles VTS and therefore doubles the exposure headroom, which is the
// entire reason a slow frame rate is worth having on a sensor with no
// AE loop of its own.
static uint32_t exposure_max_lines(const camera_sensor_t *sensor,
                                   const sensor_ae_caps_t *caps) {
    uint32_t vts = sensor->cur_vts_lines ? sensor->cur_vts_lines
                                         : sensor->base_vts_lines;
    if (vts <= caps->exp_margin) return 1;
    return vts - caps->exp_margin;
}

bool camera_sensor_has_auto_exposure(const camera_sensor_t *sensor) {
    if (sensor == NULL) return false;
    return ae_caps(sensor->kind).has_auto_ae;
}

bool camera_sensor_has_manual_exposure(const camera_sensor_t *sensor) {
    if (sensor == NULL) return false;
    return ae_caps(sensor->kind).has_ov_regs;
}

uint32_t camera_sensor_eg_min_us(const camera_sensor_t *sensor) {
    if (sensor == NULL || sensor->row_time_ns == 0) return 1;
    // One row at unity gain is the shortest thing we can express.
    uint32_t us = sensor->row_time_ns / 1000u;
    return us ? us : 1u;
}

uint32_t camera_sensor_eg_max_us(const camera_sensor_t *sensor) {
    if (sensor == NULL || sensor->row_time_ns == 0) return 1;
    const sensor_ae_caps_t caps = ae_caps(sensor->kind);
    const uint32_t max_lines = exposure_max_lines(sensor, &caps);
    const uint64_t max_us    = ((uint64_t)max_lines * sensor->row_time_ns) / 1000ULL;
    const uint64_t eg        = (max_us * caps.gain_max_q4) / 16ULL;
    return (eg > UINT32_MAX) ? UINT32_MAX : (uint32_t)eg;
}

esp_err_t camera_sensor_set_exposure_eg(camera_sensor_t *sensor, uint32_t eg_us,
                                        camera_exposure_t *out) {
    if (sensor == NULL || sensor->device == NULL) return ESP_ERR_INVALID_ARG;
    if (!ae_caps(sensor->kind).has_ov_regs) return ESP_ERR_NOT_SUPPORTED;
    if (sensor->row_time_ns == 0) {
        // No format bound yet, or the VTS read-back in capture_fps_base
        // failed. Without a row time we cannot turn microseconds into
        // register values, and guessing would be worse than refusing.
        return ESP_ERR_INVALID_STATE;
    }
    if (eg_us == 0) eg_us = 1;

    const sensor_ae_caps_t caps = ae_caps(sensor->kind);

    // Spend the light budget on integration time first, because time is
    // free and gain is not: analog gain amplifies read noise along with
    // the signal. Only what the frame period cannot absorb rolls over
    // into gain.
    const uint32_t max_lines = exposure_max_lines(sensor, &caps);
    const uint32_t max_us    = (uint32_t)(((uint64_t)max_lines *
                                           sensor->row_time_ns) / 1000ULL);

    uint32_t want_us = (eg_us < max_us) ? eg_us : max_us;
    uint32_t lines   = (uint32_t)(((uint64_t)want_us * 1000ULL +
                                   sensor->row_time_ns / 2u) / sensor->row_time_ns);
    if (lines < 1)         lines = 1;
    if (lines > max_lines) lines = max_lines;

    uint32_t got_us = (uint32_t)(((uint64_t)lines * sensor->row_time_ns) / 1000ULL);
    if (got_us == 0) got_us = 1;  // sub-microsecond row, avoid /0 below

    uint32_t gain_q4 = (uint32_t)(((uint64_t)eg_us * 16ULL + got_us / 2u) / got_us);
    if (gain_q4 < 16)                gain_q4 = 16;              // 1.0x floor
    if (gain_q4 > caps.gain_max_q4)  gain_q4 = caps.gain_max_q4;

    // Take the sensor's AE loop out of the way, remembering exactly what
    // the driver's init table had left in the mode register so we can
    // hand control back later without inventing a value. The OV5647 for
    // example boots at 0x62 (AGC manual, AEC auto) — clearing the whole
    // register to "auto" would silently turn its AGC on too.
    if (caps.has_auto_ae) {
        if (!sensor->ae_mode_valid) {
            uint8_t mode = 0;
            if (camera_sensor_read_reg(sensor, OV_REG_AEC_MODE, &mode) == ESP_OK) {
                sensor->ae_mode_saved = mode;
                sensor->ae_mode_valid = true;
            }
        }
        uint8_t base = sensor->ae_mode_valid ? sensor->ae_mode_saved : 0x00;
        esp_err_t merr = camera_sensor_write_reg(sensor, OV_REG_AEC_MODE,
                                                 (uint8_t)(base | OV_AEC_MANUAL_BITS));
        if (merr != ESP_OK) return merr;
    }

    esp_err_t err;
    if (caps.gain_reg_h != 0) {
        err = camera_sensor_write_reg(sensor, caps.gain_reg_h,
                                      (uint8_t)((gain_q4 >> 8) & 0x03u));
        if (err != ESP_OK) return err;
    }
    err = camera_sensor_write_reg(sensor, caps.gain_reg_l, (uint8_t)(gain_q4 & 0xFFu));
    if (err != ESP_OK) return err;

    // Exposure register holds lines * 16 (the low nibble is a
    // fractional row we never use).
    const uint32_t exp_reg = lines << 4;
    err = camera_sensor_write_reg(sensor, OV_REG_EXP_H, (uint8_t)((exp_reg >> 16) & 0x0Fu));
    if (err != ESP_OK) return err;
    err = camera_sensor_write_reg(sensor, OV_REG_EXP_M, (uint8_t)((exp_reg >> 8) & 0xFFu));
    if (err != ESP_OK) return err;
    err = camera_sensor_write_reg(sensor, OV_REG_EXP_L, (uint8_t)(exp_reg & 0xFFu));
    if (err != ESP_OK) return err;

    if (out) {
        out->exposure_us     = got_us;
        out->exposure_lines  = lines;
        out->gain_q4         = (uint16_t)gain_q4;
        out->exposure_capped = (eg_us > max_us);
    }
    ESP_LOGD(TAG, "exposure %" PRIu32 " us (%" PRIu32 "/%" PRIu32 " lines), gain %u.%02ux%s",
             got_us, lines, max_lines,
             (unsigned)(gain_q4 / 16u), (unsigned)((gain_q4 % 16u) * 100u / 16u),
             (eg_us > max_us) ? " [exposure capped]" : "");
    return ESP_OK;
}

esp_err_t camera_sensor_set_brightness(camera_sensor_t *sensor, int step,
                                       camera_exposure_t *out) {
    if (step < CAMERA_BRIGHT_MIN) step = CAMERA_BRIGHT_MIN;
    if (step > CAMERA_BRIGHT_MAX) step = CAMERA_BRIGHT_MAX;

    camera_exposure_t got = {0};
    esp_err_t err = camera_sensor_set_exposure_eg(sensor, bright_eg_us(step), &got);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "brightness step %d: %" PRIu32 " us (%" PRIu32 " lines), gain %u.%02ux%s",
             step, got.exposure_us, got.exposure_lines,
             (unsigned)(got.gain_q4 / 16u), (unsigned)((got.gain_q4 % 16u) * 100u / 16u),
             got.exposure_capped ? " [exposure capped]" : "");
    if (out) *out = got;
    return ESP_OK;
}

esp_err_t camera_sensor_read_exposure(camera_sensor_t *sensor, camera_exposure_t *out) {
    if (sensor == NULL || sensor->device == NULL || out == NULL) return ESP_ERR_INVALID_ARG;
    const sensor_ae_caps_t caps = ae_caps(sensor->kind);
    if (!caps.has_ov_regs) return ESP_ERR_NOT_SUPPORTED;

    uint8_t h = 0, m = 0, l = 0;
    if (camera_sensor_read_reg(sensor, OV_REG_EXP_H, &h) != ESP_OK) return ESP_FAIL;
    if (camera_sensor_read_reg(sensor, OV_REG_EXP_M, &m) != ESP_OK) return ESP_FAIL;
    if (camera_sensor_read_reg(sensor, OV_REG_EXP_L, &l) != ESP_OK) return ESP_FAIL;

    uint16_t gain_q4 = 16;
    uint8_t  gl = 0, gh = 0;
    if (camera_sensor_read_reg(sensor, caps.gain_reg_l, &gl) != ESP_OK) return ESP_FAIL;
    if (caps.gain_reg_h != 0 &&
        camera_sensor_read_reg(sensor, caps.gain_reg_h, &gh) != ESP_OK) return ESP_FAIL;
    gain_q4 = (uint16_t)(((uint16_t)(gh & 0x03u) << 8) | gl);
    if (gain_q4 < 16) gain_q4 = 16;

    const uint32_t lines = (((uint32_t)(h & 0x0Fu) << 16) |
                            ((uint32_t)m << 8) | (uint32_t)l) >> 4;

    out->exposure_lines  = lines;
    out->exposure_us     = sensor->row_time_ns
                             ? (uint32_t)(((uint64_t)lines * sensor->row_time_ns) / 1000ULL)
                             : 0;
    out->gain_q4         = gain_q4;
    out->exposure_capped = false;
    return ESP_OK;
}

int camera_sensor_brightness_step_for(const camera_sensor_t *sensor,
                                      const camera_exposure_t *exp) {
    if (sensor == NULL || exp == NULL) return CAMERA_BRIGHT_DEFAULT;
    const uint32_t gain = exp->gain_q4 ? exp->gain_q4 : 16u;
    const uint64_t eg   = ((uint64_t)exp->exposure_us * gain) / 16ULL;
    if (eg == 0) return CAMERA_BRIGHT_MIN;

    // Ladder rungs are a stop apart, so the nearest one is found by
    // comparing against the geometric midpoint between neighbours.
    // 1.5x stands in for sqrt(2) — within 6% of the true midpoint and
    // it keeps this integer-only.
    int best = CAMERA_BRIGHT_MIN;
    for (int s = CAMERA_BRIGHT_MIN; s < CAMERA_BRIGHT_MAX; s++) {
        if (eg >= ((uint64_t)bright_eg_us(s) * 3ULL) / 2ULL) best = s + 1;
    }
    return best;
}

esp_err_t camera_sensor_set_auto_exposure(camera_sensor_t *sensor) {
    if (sensor == NULL || sensor->device == NULL) return ESP_ERR_INVALID_ARG;
    const sensor_ae_caps_t caps = ae_caps(sensor->kind);
    if (!caps.has_ov_regs)      return ESP_ERR_NOT_SUPPORTED;
    if (!caps.has_auto_ae)      return ESP_ERR_NOT_SUPPORTED;
    if (!sensor->ae_mode_valid) return ESP_OK;  // never took over, nothing to undo
    return camera_sensor_write_reg(sensor, OV_REG_AEC_MODE, sensor->ae_mode_saved);
}
