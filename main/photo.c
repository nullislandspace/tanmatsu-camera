#include "photo.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "driver/jpeg_encode.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "camera_pipeline.h"
#include "camera_sensor.h"
#include "fastopen.h"

static const char *TAG = "photo";

#define JPEG_QUALITY     85
#define JPEG_TIMEOUT_MS  5000
// Drop this many frames after the format switch before grabbing the
// one we save to disk — gives AE/AWB time to settle on the new format.
#define PHOTO_DISCARD_FRAMES 5
// q85 1920x1080 JPEGs from the hardware encoder typically land at
// 400-900 KB; leave a generous ceiling so even very noisy frames fit.
#define JPEG_OUTBUF_SIZE (3 * 1024 * 1024)

static esp_err_t encode_and_write(const uint8_t *rgb565, uint32_t width, uint32_t height,
                                  const char *out_path) {
    jpeg_encoder_handle_t enc        = NULL;
    uint8_t              *jpg_buf    = NULL;
    size_t                jpg_buf_sz = 0;
    esp_err_t             err        = ESP_OK;

    jpeg_encode_engine_cfg_t eng_cfg = {
        .timeout_ms = JPEG_TIMEOUT_MS,
    };
    err = jpeg_new_encoder_engine(&eng_cfg, &enc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg_new_encoder_engine: %d", err);
        return err;
    }

    // Pick an output buffer big enough for a pathological q85 frame;
    // the actual used size is returned via out_size.
    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    jpg_buf = jpeg_alloc_encoder_mem(JPEG_OUTBUF_SIZE, &mem_cfg, &jpg_buf_sz);
    if (!jpg_buf) {
        ESP_LOGE(TAG, "jpeg_alloc_encoder_mem failed");
        err = ESP_ERR_NO_MEM;
        goto done;
    }

    // YUV444 MCU is 8x8, so width/height only need to be multiples of 8.
    // The preview pipeline snaps to 1/16 PPA scale steps which give
    // 50*k x 40*k dimensions — always multiples of 8 but not always of
    // 16, so YUV420 (MCU 16x16) and YUV422 (MCU 16x8) would reject them.
    jpeg_encode_cfg_t enc_cfg = {
        .src_type      = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample    = JPEG_DOWN_SAMPLING_YUV444,
        .image_quality = JPEG_QUALITY,
        .width         = width,
        .height        = height,
    };
    uint32_t jpg_size = 0;
    err = jpeg_encoder_process(enc, &enc_cfg,
                               rgb565, (uint32_t)(width * height * 2u),
                               jpg_buf, (uint32_t)jpg_buf_sz, &jpg_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg_encoder_process: %d", err);
        goto done;
    }
    ESP_LOGI(TAG, "JPEG encoded, %" PRIu32 " bytes (%" PRIu32 "x%" PRIu32 ")",
             jpg_size, width, height);

    FILE *f = fastopen(out_path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "fastopen('%s') failed", out_path);
        err = ESP_FAIL;
        goto done;
    }
    size_t wrote = fwrite(jpg_buf, 1, jpg_size, f);
    fastclose(f);
    if (wrote != jpg_size) {
        ESP_LOGE(TAG, "short write: %zu / %" PRIu32, wrote, jpg_size);
        err = ESP_FAIL;
        goto done;
    }
    ESP_LOGI(TAG, "wrote %s", out_path);

done:
    if (jpg_buf) {
        free(jpg_buf);
    }
    if (enc) {
        jpeg_del_encoder_engine(enc);
    }
    return err;
}

// Build <dir>/IMG_YYYYMMDD_HHMMSS.jpg from wall-clock time. If a file
// by that name already exists, suffix _1, _2, … until we get a free
// name.
static void build_filename(const char *dir, char *out, size_t n) {
    time_t    now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);

    snprintf(out, n, "%s/IMG_%s.jpg", dir, ts);
    struct stat st;
    if (stat(out, &st) != 0) {
        return;
    }
    for (int suffix = 1; suffix < 100; ++suffix) {
        snprintf(out, n, "%s/IMG_%s_%d.jpg", dir, ts, suffix);
        if (stat(out, &st) != 0) {
            return;
        }
    }
}

// Restart the live preview pipeline after a photo capture. Tries to
// switch the sensor back to the preview format and bring the CSI/ISP/PPA
// chain up. Logs and returns the first error it encounters but always
// attempts every step so we don't leave the pipeline half-configured.
static esp_err_t restart_preview(camera_sensor_t *sensor,
                                 uint32_t preview_w, uint32_t preview_h) {
    esp_err_t first_err = ESP_OK;
    esp_err_t err;

    err = camera_sensor_set_format_preview(sensor, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "restart_preview: set_format_preview: %d", err);
        if (first_err == ESP_OK) first_err = err;
    }

    err = camera_preview_start(preview_w, preview_h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "restart_preview: camera_preview_start: %d", err);
        if (first_err == ESP_OK) first_err = err;
    }

    err = camera_sensor_stream(sensor, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "restart_preview: sensor stream on: %d", err);
        if (first_err == ESP_OK) first_err = err;
    }
    return first_err;
}

esp_err_t photo_capture(camera_sensor_t *sensor,
                        uint32_t         preview_req_w,
                        uint32_t         preview_req_h,
                        const char      *dcim_dir,
                        char            *out_path,
                        size_t           out_path_sz) {
    if (!sensor || !dcim_dir || !out_path || out_path_sz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    out_path[0] = '\0';

    // 1. Snapshot the OV5647's current exposure/gain so we can restore
    //    them across the format switch. The sensor's built-in AE/AGC
    //    converges over many frames, but camera_sensor_set_format_photo()
    //    rewrites the entire register table and resets AEC/AGC to the
    //    photo format's cold-start defaults — which are very dark.
    //
    //    Preview (800x640 @50fps) and photo (1920x1080 @30fps) have
    //    very similar line times (~20us vs ~18.5us), so copying the
    //    AEC register value verbatim lands within a few percent of
    //    the correct exposure time; the sensor's AE loop then refines
    //    it over the warm-up frames.
    //
    //    Registers (from OV5647 datasheet):
    //      0x3500..0x3502 AEC exposure, 20-bit, LSB = 1/16 line
    //      0x350A..0x350B AGC gain,     10-bit
    uint8_t aec_h = 0, aec_m = 0, aec_l = 0, agc_h = 0, agc_l = 0;
    bool    have_3a = true;
    if (camera_sensor_read_reg(sensor, 0x3500, &aec_h) != ESP_OK ||
        camera_sensor_read_reg(sensor, 0x3501, &aec_m) != ESP_OK ||
        camera_sensor_read_reg(sensor, 0x3502, &aec_l) != ESP_OK ||
        camera_sensor_read_reg(sensor, 0x350A, &agc_h) != ESP_OK ||
        camera_sensor_read_reg(sensor, 0x350B, &agc_l) != ESP_OK) {
        ESP_LOGW(TAG, "failed to snapshot AEC/AGC, capture may be dark");
        have_3a = false;
    } else {
        ESP_LOGI(TAG, "preview 3A: AEC=%02x%02x%02x AGC=%02x%02x",
                 aec_h, aec_m, aec_l, agc_h, agc_l);
    }

    // 2. Stop the live preview. Order matters: sensor stream off first
    //    so no frames are in-flight, then tear the CSI chain down.
    esp_err_t err = camera_sensor_stream(sensor, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sensor stream off: %d", err);
        return err;
    }
    camera_preview_stop();

    // 3. Switch the sensor to the highest-res MIPI format (1920x1080 RAW10).
    err = camera_sensor_set_format_photo(sensor, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set_format_photo: %d", err);
        // Best effort: try to get back to the preview so the app isn't dead.
        restart_preview(sensor, preview_req_w, preview_req_h);
        return err;
    }

    // 4. Re-apply the exposure/gain we captured from the preview so the
    //    very first 1080p frame lands at roughly the same brightness.
    //    Done AFTER set_format so the format table's defaults don't
    //    overwrite it.
    if (have_3a) {
        esp_err_t w1 = camera_sensor_write_reg(sensor, 0x3500, aec_h);
        esp_err_t w2 = camera_sensor_write_reg(sensor, 0x3501, aec_m);
        esp_err_t w3 = camera_sensor_write_reg(sensor, 0x3502, aec_l);
        esp_err_t w4 = camera_sensor_write_reg(sensor, 0x350A, agc_h);
        esp_err_t w5 = camera_sensor_write_reg(sensor, 0x350B, agc_l);
        if (w1 != ESP_OK || w2 != ESP_OK || w3 != ESP_OK || w4 != ESP_OK || w5 != ESP_OK) {
            ESP_LOGW(TAG, "AEC/AGC restore failed, first frame may be dark");
        } else {
            ESP_LOGI(TAG, "restored AEC/AGC to photo format");
        }
    }

    // 3. Capture one mirrored frame at full resolution. The pipeline
    //    inside here starts the sensor, discards warm-up frames, grabs
    //    the next one, stops the sensor and tears itself down. On success
    //    we get back a heap-allocated RGB565 buffer that we own.
    uint8_t *cap_buf = NULL;
    uint32_t cap_w   = 0;
    uint32_t cap_h   = 0;
    err = camera_photo_capture(sensor, PHOTO_DISCARD_FRAMES, &cap_buf, &cap_w, &cap_h);
    if (err != ESP_OK || !cap_buf) {
        ESP_LOGE(TAG, "camera_photo_capture: %d", err);
        restart_preview(sensor, preview_req_w, preview_req_h);
        return err != ESP_OK ? err : ESP_FAIL;
    }

    // 4. Restart the preview BEFORE the slow JPEG encode so the UI comes
    //    back alive as quickly as possible. The capture buffer we own is
    //    separate from the preview pipeline's buffers, so they don't
    //    conflict.
    esp_err_t preview_err = restart_preview(sensor, preview_req_w, preview_req_h);
    if (preview_err != ESP_OK) {
        ESP_LOGW(TAG, "preview restart returned %d, continuing with save", preview_err);
    }

    // 5. JPEG-encode and write to disk.
    build_filename(dcim_dir, out_path, out_path_sz);
    err = encode_and_write(cap_buf, cap_w, cap_h, out_path);
    heap_caps_free(cap_buf);
    if (err != ESP_OK) {
        out_path[0] = '\0';
        return err;
    }

    return preview_err;  // ESP_OK unless preview restart had trouble
}
