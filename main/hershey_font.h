// Hershey Vector Font - PAX graphics rendering
// Based on the Hershey Simplex font from paulbourke.net/dataformats/hershey/
// Public domain
// Adapted for pax_buf_t rendering

#ifndef HERSHEY_FONT_H
#define HERSHEY_FONT_H

#include <stdlib.h>
#include "pax_gfx.h"
#include "hershey.h"

// Font metrics
#define HERSHEY_BASE_HEIGHT 21  // Capital letter height in font units

// Draw a single character from the Hershey font
// screen_x, screen_y: screen coordinates (y=0 at top)
// font_height: desired font height in pixels
// Returns: scaled character width for horizontal advance
static inline int hershey_draw_char(pax_buf_t *buf, pax_col_t color,
                                    float screen_x, float screen_y, char c, float font_height) {
    float scale = font_height / HERSHEY_BASE_HEIGHT;
    // Map ASCII to array index (ASCII 32-126 -> index 0-94)
    int idx = (int)c - 32;
    if (idx < 0 || idx >= 95) {
        return (int)(16 * scale);  // Default width for unsupported chars
    }

    const int *glyph = simplex[idx];
    int num_vertices = glyph[0];
    int char_width = glyph[1];

    // No vertices means space or empty character
    if (num_vertices == 0) {
        return (int)(char_width * scale);
    }

    // Process vertex pairs
    int pen_down = 0;
    float prev_sx = 0, prev_sy = 0;

    for (int i = 0; i < num_vertices; i++) {
        int vx = glyph[2 + i * 2];
        int vy = glyph[2 + i * 2 + 1];

        // Check for pen-up marker
        if (vx == -1 && vy == -1) {
            pen_down = 0;
            continue;
        }

        // Scale glyph coordinates
        // Font Y goes up (0=bottom, 21=top), flip for screen Y (down)
        float gx = vx * scale;
        float gy = (HERSHEY_BASE_HEIGHT - vy) * scale;

        float sx = screen_x + gx;
        float sy = screen_y + gy;

        if (pen_down) {
            pax_simple_line(buf, color, prev_sx, prev_sy, sx, sy);
        }

        prev_sx = sx;
        prev_sy = sy;
        pen_down = 1;
    }

    return (int)(char_width * scale);
}

// Draw a string using the Hershey font
// screen_x, screen_y: screen position (top-left of first character)
// font_height: desired font height in pixels
// Returns: pax_vec2f with total width and font height
static inline pax_vec2f hershey_draw_string(pax_buf_t *buf, pax_col_t color,
                                            float screen_x, float screen_y, const char *str, float font_height) {
    float start_x = screen_x;
    while (*str) {
        screen_x += hershey_draw_char(buf, color, screen_x, screen_y, *str, font_height);
        str++;
    }
    return (pax_vec2f){screen_x - start_x, font_height};
}

// Calculate the width of a string without drawing it
// font_height: desired font height in pixels
static inline pax_vec2f hershey_string_size(float font_height, const char *str) {
    float scale = font_height / HERSHEY_BASE_HEIGHT;
    int width = 0;
    while (*str) {
        int idx = (int)*str - 32;
        if (idx >= 0 && idx < 95) {
            width += (int)(simplex[idx][1] * scale);
        } else {
            width += (int)(16 * scale);
        }
        str++;
    }
    return (pax_vec2f){(float)width, font_height};
}

#endif // HERSHEY_FONT_H
