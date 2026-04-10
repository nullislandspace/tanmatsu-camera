#include "photo.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "driver/jpeg_encode.h"
#include "esp_log.h"

#include "camera_pipeline.h"
#include "fastopen.h"

static const char *TAG = "photo";

#define JPEG_QUALITY    85
#define JPEG_TIMEOUT_MS 5000

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
    jpg_buf = jpeg_alloc_encoder_mem(512 * 1024, &mem_cfg, &jpg_buf_sz);
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

esp_err_t photo_capture(const char *dcim_dir, char *out_path, size_t out_path_sz) {
    if (!dcim_dir || !out_path || out_path_sz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    out_path[0] = '\0';

    const uint8_t *src = camera_preview_get_pixels();
    uint32_t       w   = camera_preview_get_width();
    uint32_t       h   = camera_preview_get_height();
    if (!src || w == 0 || h == 0) {
        ESP_LOGE(TAG, "no preview frame available");
        return ESP_ERR_INVALID_STATE;
    }

    build_filename(dcim_dir, out_path, out_path_sz);
    esp_err_t err = encode_and_write(src, w, h, out_path);
    if (err != ESP_OK) {
        out_path[0] = '\0';
    }
    return err;
}
