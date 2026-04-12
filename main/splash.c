#include "splash.h"

#include <stdlib.h>
#include <string.h>

#include "bsp/display.h"
#include "driver/jpeg_decode.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fastopen.h"

static const char *TAG = "splash";

// Splash screen JPEG — installed as an app asset by the launcher.
// Try both SD and internal flash install locations.
#define SPLASH_PATH_SD  "/sd/apps/at.cavac.tanmatype/tanmatype.jpg"
#define SPLASH_PATH_INT "/int/apps/at.cavac.tanmatype/tanmatype.jpg"
#define SPLASH_DURATION_MS 2000

void splash_show(fbdraw_t *fb) {
    if (fb == NULL || fb->pixels == NULL) return;

    // Try both install locations.
    FILE *f = fastopen(SPLASH_PATH_SD, "rb");
    if (!f) f = fastopen(SPLASH_PATH_INT, "rb");
    if (!f) {
        ESP_LOGW(TAG, "splash not found at %s or %s",
                 SPLASH_PATH_SD, SPLASH_PATH_INT);
        return;
    }

    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsz <= 0 || fsz > 2 * 1024 * 1024) {
        ESP_LOGE(TAG, "bad file size %ld", fsz);
        fastclose(f);
        return;
    }

    // Allocate JPEG input buffer via the decoder's alignment helper.
    jpeg_decode_memory_alloc_cfg_t in_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER,
    };
    size_t   in_alloc = 0;
    uint8_t *in_buf   = jpeg_alloc_decoder_mem((size_t)fsz, &in_cfg, &in_alloc);
    if (!in_buf) {
        ESP_LOGE(TAG, "alloc input (%ld B) failed", fsz);
        fastclose(f);
        return;
    }
    size_t got = fread(in_buf, 1, (size_t)fsz, f);
    fastclose(f);
    if (got != (size_t)fsz) {
        ESP_LOGE(TAG, "short read %zu != %ld", got, fsz);
        free(in_buf);
        return;
    }

    // Create a temporary decoder engine.
    jpeg_decoder_handle_t dec = NULL;
    jpeg_decode_engine_cfg_t dec_eng = { .timeout_ms = 3000 };
    esp_err_t err = jpeg_new_decoder_engine(&dec_eng, &dec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg_new_decoder_engine: %d", err);
        free(in_buf);
        return;
    }

    // Get image dimensions from the header.
    jpeg_decode_picture_info_t info = {0};
    err = jpeg_decoder_get_info(in_buf, (uint32_t)fsz, &info);
    if (err != ESP_OK || info.width == 0 || info.height == 0) {
        ESP_LOGE(TAG, "bad JPEG header");
        free(in_buf);
        jpeg_del_decoder_engine(dec);
        return;
    }

    // HW decoder emits full MCU rows -> pad to multiple of 16.
    uint32_t padded_w = (info.width  + 15u) & ~15u;
    uint32_t padded_h = (info.height + 15u) & ~15u;
    size_t   out_sz   = (size_t)padded_w * padded_h * 2u;

    jpeg_decode_memory_alloc_cfg_t out_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    size_t   out_alloc = 0;
    uint8_t *out_buf   = jpeg_alloc_decoder_mem(out_sz, &out_cfg, &out_alloc);
    if (!out_buf) {
        ESP_LOGE(TAG, "alloc output (%zu B) failed", out_sz);
        free(in_buf);
        jpeg_del_decoder_engine(dec);
        return;
    }

    jpeg_decode_cfg_t decode_cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order     = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
    };
    uint32_t dec_out_sz = 0;
    err = jpeg_decoder_process(dec, &decode_cfg, in_buf, (uint32_t)fsz,
                               out_buf, out_alloc, &dec_out_sz);
    free(in_buf);
    jpeg_del_decoder_engine(dec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg decode failed: %d", err);
        free(out_buf);
        return;
    }

    // The decoded image is 800x480 landscape RGB565 matching the user
    // coordinate space. Centre it and blit with CPU rotation into the
    // panel-native framebuffer.
    fbdraw_clear(fb, FBDRAW_BLACK);
    int dx = ((int)fb->user_w - (int)info.width) / 2;
    int dy = ((int)fb->user_h - (int)info.height) / 2;
    if (dx < 0) dx = 0;
    if (dy < 0) dy = 0;
    fbdraw_blit_rotated(fb, dx, dy, (const uint16_t *)out_buf,
                        info.width, info.height);
    free(out_buf);

    // Blit and hold for 1 second minimum. The splash stays visible
    // longer if sensor detection / pipeline startup takes more time —
    // the next blit only happens once the preview is ready (or an
    // error splash replaces it if sensor detection fails).
    bsp_display_blit(0, 0, fb->panel_w, fb->panel_h, fb->pixels);
    vTaskDelay(pdMS_TO_TICKS(SPLASH_DURATION_MS));
}
