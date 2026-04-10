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

    jpeg_encode_memory_alloc_cfg_t mem_cfg = {
        .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER,
    };
    jpg_buf = jpeg_alloc_encoder_mem(JPEG_OUTBUF_SIZE, &mem_cfg, &jpg_buf_sz);
    if (!jpg_buf) {
        ESP_LOGE(TAG, "jpeg_alloc_encoder_mem failed");
        err = ESP_ERR_NO_MEM;
        goto done;
    }

    // 1920x1080 is a multiple of 16 so YUV420 works fine here.
    jpeg_encode_cfg_t enc_cfg = {
        .src_type      = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample    = JPEG_DOWN_SAMPLING_YUV420,
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

esp_err_t photo_capture(camera_sensor_t *sensor,
                        uint32_t         preview_req_w,
                        uint32_t         preview_req_h,
                        uint32_t         preview_fps,
                        const char      *dcim_dir,
                        char            *out_path,
                        size_t           out_path_sz) {
    (void)preview_req_w;
    (void)preview_req_h;
    (void)preview_fps;

    (void)sensor;
    if (!dcim_dir || !out_path || out_path_sz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    out_path[0] = '\0';

    // The sensor is already running at 1920x1080 RAW10 and the CSI
    // writes into double-buffered targets managed by camera_pipeline.
    // camera_photo_snapshot serialises against the render task via
    // s_render_ready and freezes the stable pointer via the photo
    // lock, so we can just call it and get a fresh, tear-free,
    // orientation-corrected 1920x1080 RGB565 buffer back.
    uint8_t *snap = NULL;
    uint32_t snap_w = 0, snap_h = 0;
    esp_err_t err = camera_photo_snapshot(&snap, &snap_w, &snap_h);
    if (err != ESP_OK || snap == NULL) {
        ESP_LOGE(TAG, "camera_photo_snapshot: %d", err);
        return err != ESP_OK ? err : ESP_FAIL;
    }

    // JPEG-encode and write to disk.
    build_filename(dcim_dir, out_path, out_path_sz);
    err = encode_and_write(snap, snap_w, snap_h, out_path);
    heap_caps_free(snap);
    if (err != ESP_OK) {
        out_path[0] = '\0';
        return err;
    }

    return ESP_OK;
}
