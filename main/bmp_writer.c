#include "bmp_writer.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "fastopen.h"

static const char *TAG = "bmp_writer";

#pragma pack(push, 1)
typedef struct {
    uint16_t bf_type;        // 'BM'
    uint32_t bf_size;        // total file size
    uint16_t bf_reserved1;
    uint16_t bf_reserved2;
    uint32_t bf_off_bits;    // offset from file start to pixel data
} bmp_file_header_t;

typedef struct {
    uint32_t bi_size;          // 40
    int32_t  bi_width;
    int32_t  bi_height;        // positive = bottom-up
    uint16_t bi_planes;        // 1
    uint16_t bi_bit_count;     // 24
    uint32_t bi_compression;   // 0 = BI_RGB
    uint32_t bi_size_image;    // may be 0 for BI_RGB
    int32_t  bi_x_ppm;
    int32_t  bi_y_ppm;
    uint32_t bi_clr_used;
    uint32_t bi_clr_important;
} bmp_info_header_t;
#pragma pack(pop)

esp_err_t bmp_writer_save_rgb565(const char *path, const uint16_t *pixels, uint32_t width, uint32_t height) {
    if (!path || !pixels || width == 0 || height == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 24-bit BMP rows must be padded to a multiple of 4 bytes.
    const uint32_t row_bytes    = width * 3u;
    const uint32_t row_padding  = (4u - (row_bytes & 3u)) & 3u;
    const uint32_t padded_row   = row_bytes + row_padding;
    const uint32_t image_bytes  = padded_row * height;
    const uint32_t headers_size = sizeof(bmp_file_header_t) + sizeof(bmp_info_header_t);

    bmp_file_header_t fh = {
        .bf_type     = 0x4D42, // 'BM'
        .bf_size     = headers_size + image_bytes,
        .bf_off_bits = headers_size,
    };
    bmp_info_header_t ih = {
        .bi_size       = sizeof(bmp_info_header_t),
        .bi_width      = (int32_t)width,
        .bi_height     = (int32_t)height,
        .bi_planes     = 1,
        .bi_bit_count  = 24,
        .bi_compression = 0,
        .bi_size_image = image_bytes,
    };

    // A single row of RGB888 (BGR-order) bytes, allocated once and reused.
    uint8_t *row_buf = heap_caps_malloc(padded_row, MALLOC_CAP_8BIT | MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (!row_buf) {
        ESP_LOGE(TAG, "row_buf alloc failed (%" PRIu32 " bytes)", padded_row);
        return ESP_ERR_NO_MEM;
    }
    if (row_padding) {
        memset(row_buf + row_bytes, 0, row_padding);
    }

    FILE *f = fastopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "fastopen('%s') failed", path);
        free(row_buf);
        return ESP_FAIL;
    }

    if (fwrite(&fh, sizeof(fh), 1, f) != 1 ||
        fwrite(&ih, sizeof(ih), 1, f) != 1) {
        ESP_LOGE(TAG, "header write failed");
        fastclose(f);
        free(row_buf);
        return ESP_FAIL;
    }

    // BMP is stored bottom-up: walk source rows in reverse order.
    for (int32_t y = (int32_t)height - 1; y >= 0; --y) {
        const uint16_t *src = pixels + (uint32_t)y * width;
        uint8_t *dst = row_buf;
        for (uint32_t x = 0; x < width; ++x) {
            uint16_t p = src[x];
            uint8_t r5 = (p >> 11) & 0x1F;
            uint8_t g6 = (p >> 5)  & 0x3F;
            uint8_t b5 =  p        & 0x1F;
            // Expand to 8 bits with bit-replication in the low bits.
            uint8_t r = (uint8_t)((r5 << 3) | (r5 >> 2));
            uint8_t g = (uint8_t)((g6 << 2) | (g6 >> 4));
            uint8_t b = (uint8_t)((b5 << 3) | (b5 >> 2));
            // BMP 24-bit is BGR.
            *dst++ = b;
            *dst++ = g;
            *dst++ = r;
        }
        if (fwrite(row_buf, padded_row, 1, f) != 1) {
            ESP_LOGE(TAG, "row %" PRId32 " write failed", y);
            fastclose(f);
            free(row_buf);
            return ESP_FAIL;
        }
    }

    fastclose(f);
    free(row_buf);
    ESP_LOGI(TAG, "wrote %s (%" PRIu32 "x%" PRIu32 ", %" PRIu32 " bytes)",
             path, width, height, (uint32_t)(headers_size + image_bytes));
    return ESP_OK;
}
