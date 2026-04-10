#include "fbdraw.h"

#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"

// Single include of the Hershey simplex font table (~40 KB of vector
// vertex data). hershey.h is now marked static const so this is the
// only translation unit that holds a copy, and the symbol is local to
// this file — no link conflicts, no need for a separate .c file.
#include "hershey.h"

#define HERSHEY_BASE_HEIGHT 21  // Cap height of the Hershey simplex font

static const char *TAG = "fbdraw";

esp_err_t fbdraw_init(fbdraw_t *fb, size_t panel_w, size_t panel_h) {
    if (fb == NULL || panel_w == 0 || panel_h == 0) return ESP_ERR_INVALID_ARG;
    memset(fb, 0, sizeof(*fb));

    fb->panel_w = panel_w;
    fb->panel_h = panel_h;
    fb->user_w  = panel_h;   // landscape view swaps the axes
    fb->user_h  = panel_w;

    uint32_t cache_line = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
    size_t raw_sz = panel_w * panel_h * sizeof(uint16_t);
    // Round up to the cache line so esp_cache_msync (inside the DSI
    // blit path) is happy.
    fb->pixels_sz = (raw_sz + cache_line - 1u) & ~((size_t)cache_line - 1u);
    fb->pixels    = heap_caps_aligned_calloc(cache_line, 1, fb->pixels_sz,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (fb->pixels == NULL) {
        ESP_LOGE(TAG, "fb alloc failed (%zu bytes)", fb->pixels_sz);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void fbdraw_deinit(fbdraw_t *fb) {
    if (fb == NULL) return;
    if (fb->pixels) {
        free(fb->pixels);
    }
    memset(fb, 0, sizeof(*fb));
}

// Fast memset-of-16-bit-words. Collapses to one memset for colours
// whose high and low bytes match (all greys including black/white).
static inline void memset16(uint16_t *dst, uint16_t val, size_t count) {
    const uint8_t hi = (uint8_t)(val >> 8);
    const uint8_t lo = (uint8_t)(val & 0xFFu);
    if (hi == lo) {
        memset(dst, (int)lo, count * 2);
        return;
    }
    for (size_t i = 0; i < count; i++) {
        dst[i] = val;
    }
}

void fbdraw_clear(fbdraw_t *fb, uint16_t color) {
    if (fb == NULL || fb->pixels == NULL) return;
    memset16(fb->pixels, color, fb->panel_w * fb->panel_h);
}

// offset(ux, uy) = ux * panel_w + (panel_w - 1 - uy)
// (Matches pax_orient_ccw3 for PAX_O_ROT_CW.)
static inline size_t user_offset(const fbdraw_t *fb, int ux, int uy) {
    return (size_t)ux * fb->panel_w + (fb->panel_w - 1u - (size_t)uy);
}

void fbdraw_put_pixel(fbdraw_t *fb, int user_x, int user_y, uint16_t color) {
    if (fb == NULL || fb->pixels == NULL) return;
    if (user_x < 0 || user_y < 0) return;
    if ((size_t)user_x >= fb->user_w || (size_t)user_y >= fb->user_h) return;
    fb->pixels[user_offset(fb, user_x, user_y)] = color;
}

void fbdraw_fill_rect(fbdraw_t *fb, int user_x, int user_y,
                      int user_w, int user_h, uint16_t color) {
    if (fb == NULL || fb->pixels == NULL) return;
    if (user_w <= 0 || user_h <= 0) return;

    // Clip to the user-visible area.
    int x0 = user_x;
    int y0 = user_y;
    int x1 = user_x + user_w;
    int y1 = user_y + user_h;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if ((size_t)x1 > fb->user_w) x1 = (int)fb->user_w;
    if ((size_t)y1 > fb->user_h) y1 = (int)fb->user_h;
    if (x0 >= x1 || y0 >= y1) return;

    // For each column in the USER-space rect: the column maps to one
    // contiguous row in panel memory (panel_y = user_x), and the rect's
    // y range maps to a contiguous slice of panel_x = [panel_w - 1 - (y1-1)
    // .. panel_w - 1 - y0] inside that row. Sequential writes.
    const size_t row_stride = fb->panel_w;
    const size_t col_base_x = fb->panel_w - (size_t)y1;          // inclusive start
    const size_t col_span   = (size_t)(y1 - y0);
    for (int ux = x0; ux < x1; ux++) {
        uint16_t *row = fb->pixels + (size_t)ux * row_stride + col_base_x;
        memset16(row, color, col_span);
    }
}

void fbdraw_line(fbdraw_t *fb, int x0, int y0, int x1, int y1, uint16_t color) {
    if (fb == NULL || fb->pixels == NULL) return;
    // Standard integer Bresenham. put_pixel handles clipping.
    int dx =  (x1 > x0) ? (x1 - x0) : (x0 - x1);
    int dy = -((y1 > y0) ? (y1 - y0) : (y0 - y1));
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        fbdraw_put_pixel(fb, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void fbdraw_blit_panel(fbdraw_t *fb, int panel_x, int panel_y,
                       const uint16_t *src, size_t src_panel_w, size_t src_panel_h) {
    if (fb == NULL || fb->pixels == NULL || src == NULL) return;
    if (panel_x < 0 || panel_y < 0) return;
    if ((size_t)panel_x + src_panel_w > fb->panel_w) return;
    if ((size_t)panel_y + src_panel_h > fb->panel_h) return;

    const size_t row_bytes = src_panel_w * sizeof(uint16_t);
    for (size_t row = 0; row < src_panel_h; row++) {
        uint16_t *dst = fb->pixels + ((size_t)panel_y + row) * fb->panel_w + (size_t)panel_x;
        const uint16_t *s = src + row * src_panel_w;
        memcpy(dst, s, row_bytes);
    }
}

void fbdraw_blit_rotated(fbdraw_t *fb, int dst_x, int dst_y,
                         const uint16_t *src, size_t src_w, size_t src_h) {
    if (fb == NULL || fb->pixels == NULL || src == NULL) return;
    if (dst_x < 0 || dst_y < 0) return;
    if ((size_t)dst_x + src_w > fb->user_w) return;
    if ((size_t)dst_y + src_h > fb->user_h) return;

    // For each source column (= user_x) we fill one panel row.
    // Inside that row, pixels are written in reverse x order because
    // panel_x = panel_w - 1 - user_y.
    const size_t panel_w = fb->panel_w;
    for (size_t sx = 0; sx < src_w; sx++) {
        size_t ux = (size_t)dst_x + sx;
        uint16_t *row = fb->pixels + ux * panel_w;
        // Target span inside the row: panel_x in [panel_w-1-(dst_y+src_h-1)
        // .. panel_w-1-dst_y]. Fill it right-to-left while reading source
        // column top-to-bottom.
        size_t right_panel_x = panel_w - 1u - (size_t)dst_y;
        for (size_t sy = 0; sy < src_h; sy++) {
            row[right_panel_x - sy] = src[sy * src_w + sx];
        }
    }
}

int fbdraw_hershey_char(fbdraw_t *fb, uint16_t color, int sx, int sy,
                        char c, int font_height) {
    const float scale = (float)font_height / (float)HERSHEY_BASE_HEIGHT;
    int idx = (int)(unsigned char)c - 32;
    if (idx < 0 || idx >= 95) {
        return (int)(16 * scale);
    }

    const int *glyph = simplex[idx];
    int num_vertices = glyph[0];
    int char_width   = glyph[1];

    if (num_vertices == 0) {
        return (int)(char_width * scale);
    }

    bool pen_down = false;
    int  prev_px = 0, prev_py = 0;

    for (int i = 0; i < num_vertices; i++) {
        int vx = glyph[2 + i * 2];
        int vy = glyph[2 + i * 2 + 1];

        // Pen-up marker (-1, -1) splits a glyph into multiple strokes.
        if (vx == -1 && vy == -1) {
            pen_down = false;
            continue;
        }

        // Hershey glyph space has y=0 at the baseline and y=21 at the
        // cap-height, so we flip to user-screen space (y=0 at top).
        int px = sx + (int)((float)vx * scale);
        int py = sy + (int)((float)(HERSHEY_BASE_HEIGHT - vy) * scale);

        if (pen_down) {
            fbdraw_line(fb, prev_px, prev_py, px, py, color);
        }
        prev_px  = px;
        prev_py  = py;
        pen_down = true;
    }

    return (int)((float)char_width * scale);
}

int fbdraw_hershey_string(fbdraw_t *fb, uint16_t color, int x, int y,
                          const char *str, int font_height) {
    int start_x = x;
    while (*str) {
        x += fbdraw_hershey_char(fb, color, x, y, *str, font_height);
        str++;
    }
    return x - start_x;
}

int fbdraw_hershey_string_width(const char *str, int font_height) {
    const float scale = (float)font_height / (float)HERSHEY_BASE_HEIGHT;
    int width = 0;
    while (*str) {
        int idx = (int)(unsigned char)*str - 32;
        if (idx >= 0 && idx < 95) {
            width += (int)((float)simplex[idx][1] * scale);
        } else {
            width += (int)(16 * scale);
        }
        str++;
    }
    return width;
}
