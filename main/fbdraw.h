#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// Tiny RGB565 framebuffer + drawing primitives, written specifically
// for the Tanmatsu's 480 x 800 portrait panel displayed in landscape
// via the BSP's BSP_DISPLAY_ROTATION_270 default (= PAX_O_ROT_CW).
//
// The caller works entirely in USER / LANDSCAPE coordinates:
//
//     user_w, user_h = 800 x 480, origin top-left as perceived by the user
//
// while the backing framebuffer is stored in PANEL / PORTRAIT layout:
//
//     panel_w, panel_h = 480 x 800, row-major, suitable for
//     bsp_display_blit(0, 0, panel_w, panel_h, pixels)
//
// The forward transform (matches pax_orient_ccw3_vec2i inside
// managed_components/robotman2412__pax-gfx/core/src/pax_orientation.c):
//
//     panel_x = panel_w - 1 - user_y
//     panel_y = user_x
//
// Which in linear memory becomes:
//
//     offset(user_x, user_y) = user_x * panel_w + (panel_w - 1 - user_y)
//
// Two consequences the primitives exploit:
//   - iterating user_y at fixed user_x produces sequential reverse-order
//     writes inside one panel row (fast, memset-ish)
//   - iterating user_x at fixed user_y produces stride writes across
//     panel rows (slow — use only when unavoidable)

typedef struct {
    uint16_t *pixels;       // cache-line aligned, panel_w * panel_h * 2 bytes
    size_t    pixels_sz;
    size_t    panel_w;      // 480 on Tanmatsu
    size_t    panel_h;      // 800
    size_t    user_w;       // 800 (= panel_h)
    size_t    user_h;       // 480 (= panel_w)
} fbdraw_t;

// Convenience RGB565 colour builder (5/6/5 with R in top bits).
static inline uint16_t fbdraw_rgb(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8u) << 8) | ((g & 0xFCu) << 3) | (b >> 3));
}

#define FBDRAW_BLACK ((uint16_t)0x0000u)
#define FBDRAW_WHITE ((uint16_t)0xFFFFu)
#define FBDRAW_RED   ((uint16_t)0xF800u)

// Allocate the framebuffer in PSRAM, cache-line aligned (required by
// bsp_display_blit's DMA path). panel_w / panel_h are the physical
// panel dimensions; user_w / user_h are derived from those (swap).
esp_err_t fbdraw_init(fbdraw_t *fb, size_t panel_w, size_t panel_h);

// Free the framebuffer.
void fbdraw_deinit(fbdraw_t *fb);

// Fill the entire panel with a single colour. For colours whose high
// and low bytes are equal (0x0000, 0xFFFF, ...) this collapses to one
// memset; otherwise it's a short loop over the word array.
void fbdraw_clear(fbdraw_t *fb, uint16_t color);

// Single pixel at user coordinates. Silently clipped.
void fbdraw_put_pixel(fbdraw_t *fb, int user_x, int user_y, uint16_t color);

// Axis-aligned rectangle fill at user coordinates. Clipped to the
// user-visible area. Inner loop is sequential in panel memory.
void fbdraw_fill_rect(fbdraw_t *fb, int user_x, int user_y,
                      int user_w, int user_h, uint16_t color);

// Bresenham line between two user-space endpoints, inclusive.
void fbdraw_line(fbdraw_t *fb, int x0, int y0, int x1, int y1, uint16_t color);

// Copy a pre-rotated (panel-native layout) RGB565 image directly into
// panel memory at panel coordinates (panel_x, panel_y). Used when
// something upstream (e.g. the camera PPA) already produced the image
// in the correct orientation — each source row becomes one panel row,
// so the copy is a sequence of row-sized memcpys.
void fbdraw_blit_panel(fbdraw_t *fb, int panel_x, int panel_y,
                       const uint16_t *src, size_t src_panel_w, size_t src_panel_h);

// Copy an unrotated (user-landscape layout) RGB565 image into the fb at
// user coordinates (dst_x, dst_y). CPU rotation loop — avoid on the hot
// path; prefer fbdraw_blit_panel + PPA pre-rotation.
void fbdraw_blit_rotated(fbdraw_t *fb, int dst_x, int dst_y,
                         const uint16_t *src, size_t src_w, size_t src_h);

// Draw one Hershey simplex character at user coordinates (x, y is the
// TOP-LEFT corner of the glyph box). Returns the character's advance
// width in user pixels.
int fbdraw_hershey_char(fbdraw_t *fb, uint16_t color, int x, int y,
                        char c, int font_height);

// Draw a NUL-terminated string, returning the total advance width.
int fbdraw_hershey_string(fbdraw_t *fb, uint16_t color, int x, int y,
                          const char *str, int font_height);

// Measure the advance width of a string without drawing it.
int fbdraw_hershey_string_width(const char *str, int font_height);
