#include "viewer.h"

#include <dirent.h>
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "driver/jpeg_decode.h"
#include "driver/ppa.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "hal/ppa_types.h"

#include "fastopen.h"

static const char *TAG = "viewer";

#define MAX_FILES              2048
#define JPEG_DECODE_TIMEOUT_MS 5000
#define MAX_JPEG_SIZE          (8u * 1024u * 1024u)

// Sorted filename list (basenames, newest first).
static char **s_files = NULL;
static int    s_count = 0;
static int    s_index = 0;
static char   s_dcim[96] = {0};

// Fit box (display-space) passed to viewer_open().
static uint32_t s_max_w = 0;
static uint32_t s_max_h = 0;

// Decoder + PPA client + SRM completion semaphore.
static jpeg_decoder_handle_t s_jpeg_dec = NULL;
static ppa_client_handle_t   s_ppa      = NULL;
static SemaphoreHandle_t     s_ppa_done = NULL;

// Pre-allocated display-sized RGB565 output. Sized for the worst case
// (max_w x max_h x 2 bytes, rounded up to a cache line) so we don't
// reallocate on navigation. Each load writes its own scaled image into
// the top-left corner of this buffer at stride = s_scaled_w * 2.
static uint8_t *s_disp_buf    = NULL;
static size_t   s_disp_buf_sz = 0;
static uint32_t s_scaled_w    = 0;
static uint32_t s_scaled_h    = 0;

static bool IRAM_ATTR on_ppa_done(ppa_client_handle_t h,
                                  ppa_event_data_t *event,
                                  void *user_data) {
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_ppa_done, &hpw);
    return hpw == pdTRUE;
}

static int file_cmp_desc(const void *a, const void *b) {
    const char *sa = *(const char *const *)a;
    const char *sb = *(const char *const *)b;
    return strcmp(sb, sa);  // reversed -> newest first
}

static void free_file_list(void) {
    if (s_files) {
        for (int i = 0; i < s_count; i++) {
            free(s_files[i]);
        }
        free(s_files);
        s_files = NULL;
    }
    s_count = 0;
    s_index = 0;
}

static esp_err_t scan_dcim(const char *dcim_dir) {
    DIR *dir = opendir(dcim_dir);
    if (!dir) {
        ESP_LOGE(TAG, "opendir('%s'): %d", dcim_dir, errno);
        return ESP_FAIL;
    }

    s_files = calloc(MAX_FILES, sizeof(char *));
    if (!s_files) {
        closedir(dir);
        return ESP_ERR_NO_MEM;
    }

    struct dirent *de;
    while ((de = readdir(dir)) != NULL && s_count < MAX_FILES) {
        size_t nlen = strlen(de->d_name);
        if (nlen < 8) continue;
        if (strncmp(de->d_name, "IMG_", 4) != 0) continue;
        if (strcmp(de->d_name + nlen - 4, ".jpg") != 0) continue;

        s_files[s_count] = strdup(de->d_name);
        if (!s_files[s_count]) {
            closedir(dir);
            return ESP_ERR_NO_MEM;
        }
        s_count++;
    }
    closedir(dir);

    if (s_count == 0) {
        ESP_LOGW(TAG, "no IMG_*.jpg files in %s", dcim_dir);
        return ESP_ERR_NOT_FOUND;
    }

    qsort(s_files, s_count, sizeof(char *), file_cmp_desc);
    ESP_LOGI(TAG, "found %d image(s); newest: %s", s_count, s_files[0]);
    return ESP_OK;
}

static esp_err_t load_image(int index) {
    if (index < 0 || index >= s_count) return ESP_ERR_INVALID_ARG;

    char path[160];
    snprintf(path, sizeof(path), "%s/%s", s_dcim, s_files[index]);

    // Slurp the JPEG into a DMA-capable buffer.
    FILE *f = fastopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "fastopen('%s') failed", path);
        return ESP_FAIL;
    }
    fseek(f, 0, SEEK_END);
    long fsz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (fsz <= 0 || (size_t)fsz > MAX_JPEG_SIZE) {
        ESP_LOGE(TAG, "bad JPEG size %ld for %s", fsz, path);
        fastclose(f);
        return ESP_FAIL;
    }

    jpeg_decode_memory_alloc_cfg_t in_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_INPUT_BUFFER,
    };
    size_t   in_alloc = 0;
    uint8_t *in_buf   = jpeg_alloc_decoder_mem((size_t)fsz, &in_cfg, &in_alloc);
    if (!in_buf) {
        ESP_LOGE(TAG, "alloc jpeg input buffer (%ld B) failed", fsz);
        fastclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t got = fread(in_buf, 1, (size_t)fsz, f);
    fastclose(f);
    if (got != (size_t)fsz) {
        ESP_LOGE(TAG, "short read %zu != %ld", got, fsz);
        free(in_buf);
        return ESP_FAIL;
    }

    // Peek header to get actual image dimensions (needed both for PPA
    // scaling and for sizing the decoder's output buffer).
    jpeg_decode_picture_info_t info = {0};
    esp_err_t err = jpeg_decoder_get_info(in_buf, (uint32_t)fsz, &info);
    if (err != ESP_OK || info.width == 0 || info.height == 0) {
        ESP_LOGE(TAG, "jpeg_decoder_get_info: %d (%" PRIu32 "x%" PRIu32 ")",
                 err, info.width, info.height);
        free(in_buf);
        return err != ESP_OK ? err : ESP_FAIL;
    }

    // The hardware decoder emits full MCU rows/columns, so the physical
    // output dimensions are rounded up to a multiple of 16 even if the
    // declared image size isn't. Allocate for the padded size.
    uint32_t padded_w = (info.width  + 15u) & ~15u;
    uint32_t padded_h = (info.height + 15u) & ~15u;
    size_t   out_sz   = (size_t)padded_w * padded_h * 2u;

    jpeg_decode_memory_alloc_cfg_t out_cfg = {
        .buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER,
    };
    size_t   out_alloc = 0;
    uint8_t *out_buf   = jpeg_alloc_decoder_mem(out_sz, &out_cfg, &out_alloc);
    if (!out_buf) {
        ESP_LOGE(TAG, "alloc jpeg output buffer (%zu B) failed", out_sz);
        free(in_buf);
        return ESP_ERR_NO_MEM;
    }

    // rgb_order selects the byte order of the RGB565 output:
    //   _RGB = "big endian"   -> R byte first in memory (high bits last)
    //   _BGR = "small endian" -> layout of a little-endian uint16 with
    //                            the PAX_BUF_16_565RGB convention (R in
    //                            the top 5 bits of the word).
    // Without this set explicitly the default (RGB/big endian) produces
    // byte-swapped words and the image renders as chaotic dithered colour.
    jpeg_decode_cfg_t dec_cfg = {
        .output_format = JPEG_DECODE_OUT_FORMAT_RGB565,
        .rgb_order     = JPEG_DEC_RGB_ELEMENT_ORDER_BGR,
    };
    uint32_t dec_out_sz = 0;
    err = jpeg_decoder_process(s_jpeg_dec, &dec_cfg, in_buf, (uint32_t)fsz,
                               out_buf, out_alloc, &dec_out_sz);
    free(in_buf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg_decoder_process: %d", err);
        free(out_buf);
        return err;
    }

    // Snap the scale factor to the largest n/16 that still fits the
    // image inside the display-space box while preserving aspect ratio.
    // This is the same PPA-frag quantisation dance as the preview
    // pipeline (see camera_pipeline.c) — the PPA scale register only
    // has 4 fractional bits.
    float sx = (float)s_max_w / (float)info.width;
    float sy = (float)s_max_h / (float)info.height;
    float s  = sx < sy ? sx : sy;
    uint32_t frag = (uint32_t)(s * 16.0f);
    if (frag < 1)  frag = 1;
    if (frag > 16) frag = 16;
    float scale = (float)frag / 16.0f;

    uint32_t scaled_w = (info.width  * frag) / 16u;
    uint32_t scaled_h = (info.height * frag) / 16u;
    if (scaled_w > s_max_w) scaled_w = s_max_w;
    if (scaled_h > s_max_h) scaled_h = s_max_h;

    ppa_srm_oper_config_t srm = {
        .in = {
            .buffer         = out_buf,
            .pic_w          = padded_w,
            .pic_h          = padded_h,
            .block_w        = info.width,
            .block_h        = info.height,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer         = s_disp_buf,
            .buffer_size    = s_disp_buf_sz,
            .pic_w          = scaled_w,
            .pic_h          = scaled_h,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x        = scale,
        .scale_y        = scale,
        .mirror_x       = false,
        .mirror_y       = false,
        .rgb_swap       = false,
        .byte_swap      = false,
        .mode           = PPA_TRANS_MODE_NON_BLOCKING,
    };
    err = ppa_do_scale_rotate_mirror(s_ppa, &srm);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa srm: %d", err);
        free(out_buf);
        return err;
    }
    if (xSemaphoreTake(s_ppa_done, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE(TAG, "ppa srm timeout");
        free(out_buf);
        return ESP_ERR_TIMEOUT;
    }
    free(out_buf);

    s_scaled_w = scaled_w;
    s_scaled_h = scaled_h;
    s_index    = index;
    ESP_LOGI(TAG, "loaded %s: src %" PRIu32 "x%" PRIu32 " -> disp %" PRIu32
                  "x%" PRIu32 " (frag %" PRIu32 "/16)",
             s_files[index], info.width, info.height,
             s_scaled_w, s_scaled_h, frag);
    return ESP_OK;
}

esp_err_t viewer_open(const char *dcim_dir, uint32_t max_w, uint32_t max_h) {
    if (!dcim_dir || max_w == 0 || max_h == 0) return ESP_ERR_INVALID_ARG;

    // viewer_close is idempotent and leaves the module in a clean state.
    viewer_close();

    strncpy(s_dcim, dcim_dir, sizeof(s_dcim) - 1);
    s_dcim[sizeof(s_dcim) - 1] = '\0';
    s_max_w = max_w;
    s_max_h = max_h;

    jpeg_decode_engine_cfg_t dec_eng_cfg = {
        .timeout_ms = JPEG_DECODE_TIMEOUT_MS,
    };
    esp_err_t err = jpeg_new_decoder_engine(&dec_eng_cfg, &s_jpeg_dec);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "jpeg_new_decoder_engine: %d", err);
        goto fail;
    }

    ppa_client_config_t ppa_cfg = {
        .oper_type             = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
        .data_burst_length     = PPA_DATA_BURST_LENGTH_128,
    };
    err = ppa_register_client(&ppa_cfg, &s_ppa);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa_register_client: %d", err);
        goto fail;
    }
    ppa_event_callbacks_t ppa_cbs = { .on_trans_done = on_ppa_done };
    err = ppa_client_register_event_callbacks(s_ppa, &ppa_cbs);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa register cbs: %d", err);
        goto fail;
    }

    s_ppa_done = xSemaphoreCreateBinary();
    if (!s_ppa_done) {
        ESP_LOGE(TAG, "semaphore alloc failed");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    // Pre-allocate the display-sized output buffer, cache-line aligned
    // as required by ppa_do_scale_rotate_mirror().
    uint32_t cache_line = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
    s_disp_buf_sz = (size_t)max_w * max_h * 2u;
    s_disp_buf_sz = (s_disp_buf_sz + cache_line - 1u) & ~((size_t)cache_line - 1u);
    s_disp_buf    = heap_caps_aligned_calloc(cache_line, 1, s_disp_buf_sz,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!s_disp_buf) {
        ESP_LOGE(TAG, "alloc disp buffer (%zu B) failed", s_disp_buf_sz);
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    err = scan_dcim(dcim_dir);
    if (err != ESP_OK) goto fail;

    err = load_image(0);
    if (err != ESP_OK) goto fail;

    return ESP_OK;

fail:
    viewer_close();
    return err;
}

esp_err_t viewer_next(void) {
    if (s_count == 0) return ESP_ERR_INVALID_STATE;
    if (s_index + 1 >= s_count) return ESP_OK;
    return load_image(s_index + 1);
}

esp_err_t viewer_prev(void) {
    if (s_count == 0) return ESP_ERR_INVALID_STATE;
    if (s_index == 0) return ESP_OK;
    return load_image(s_index - 1);
}

void viewer_close(void) {
    free_file_list();

    if (s_ppa)      { ppa_unregister_client(s_ppa);     s_ppa = NULL; }
    if (s_ppa_done) { vSemaphoreDelete(s_ppa_done);     s_ppa_done = NULL; }
    if (s_jpeg_dec) { jpeg_del_decoder_engine(s_jpeg_dec); s_jpeg_dec = NULL; }
    if (s_disp_buf) { free(s_disp_buf); s_disp_buf = NULL; s_disp_buf_sz = 0; }

    s_scaled_w = 0;
    s_scaled_h = 0;
    s_max_w    = 0;
    s_max_h    = 0;
}

bool           viewer_has_image(void)  { return s_count > 0 && s_disp_buf != NULL && s_scaled_w > 0; }
const uint8_t *viewer_get_pixels(void) { return s_disp_buf; }
uint32_t       viewer_get_width(void)  { return s_scaled_w; }
uint32_t       viewer_get_height(void) { return s_scaled_h; }

const char *viewer_get_filename(void) {
    return (s_count > 0) ? s_files[s_index] : "";
}
int viewer_get_index(void) { return s_index; }
int viewer_get_total(void) { return s_count; }
