#include "icons.h"

#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "pax_codecs.h"

static const char TAG[] = "icons";

#define ICON_W           32
#define ICON_H           32
#define ICON_BUFFER_SIZE (ICON_W * ICON_H * 4)

// Single-path table for all icons. ICON_ESC is special-cased in
// icons_load() to first try the app-supplied override before falling
// back to this launcher-default path.
static const char *icon_filenames[ICON_KEY_COUNT] = {
    [ICON_ESC] = "/int/icons/esc.png",
    [ICON_F1]  = "/int/icons/f1.png",
    [ICON_F2]  = "/int/icons/f2.png",
    [ICON_F3]  = "/int/icons/f3.png",
    [ICON_F4]  = "/int/icons/f4.png",
    [ICON_F5]  = "/int/icons/f5.png",
    [ICON_F6]  = "/int/icons/f6.png",
};

// App-supplied esc icon (better contrast against the HUD's dark
// background). Tried before the launcher default. Same path scheme as
// splash.c.
#define APP_ESC_PATH_SD  "/sd/apps/at.cavac.tanmatype/esc.png"
#define APP_ESC_PATH_INT "/int/apps/at.cavac.tanmatype/esc.png"

static pax_buf_t s_icons[ICON_KEY_COUNT]  = {0};
static bool      s_loaded[ICON_KEY_COUNT] = {false};
static bool      s_any_missing            = false;

// Open an icon file, trying the app-supplied esc override first for
// ICON_ESC. *out_path is set to whichever path actually opened (for
// log messages). Returns NULL if no path opens.
static FILE *open_icon(int i, const char **out_path) {
    if (i == ICON_ESC) {
        FILE *f = fopen(APP_ESC_PATH_SD, "rb");
        if (f) { *out_path = APP_ESC_PATH_SD;  return f; }
        f      = fopen(APP_ESC_PATH_INT, "rb");
        if (f) { *out_path = APP_ESC_PATH_INT; return f; }
    }
    *out_path = icon_filenames[i];
    return fopen(icon_filenames[i], "rb");
}

void icons_load(void) {
    for (int i = 0; i < ICON_KEY_COUNT; i++) {
        const char *opened_path = NULL;
        FILE *fd = open_icon(i, &opened_path);
        if (!fd) {
            ESP_LOGW(TAG, "icon not found: %s", icon_filenames[i]);
            s_any_missing = true;
            continue;
        }

        void *buffer = heap_caps_calloc(1, ICON_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
        if (!buffer) {
            ESP_LOGE(TAG, "alloc failed for %s", icon_filenames[i]);
            fclose(fd);
            s_any_missing = true;
            continue;
        }

        pax_buf_init(&s_icons[i], buffer, ICON_W, ICON_H, PAX_BUF_32_8888ARGB);

        if (!pax_insert_png_fd(&s_icons[i], fd, 0, 0, 0)) {
            ESP_LOGE(TAG, "decode failed: %s", opened_path);
            pax_buf_destroy(&s_icons[i]);
            free(buffer);
            memset(&s_icons[i], 0, sizeof(pax_buf_t));
            s_any_missing = true;
        } else {
            s_loaded[i] = true;
            ESP_LOGI(TAG, "loaded %s", opened_path);
        }

        fclose(fd);
    }
}

pax_buf_t *icons_get(icon_key_t key) {
    if (key < 0 || key >= ICON_KEY_COUNT) return NULL;
    if (!s_loaded[key]) return NULL;
    return &s_icons[key];
}

bool icons_any_missing(void) { return s_any_missing; }

int icons_width(icon_key_t key) {
    pax_buf_t *b = icons_get(key);
    return b ? pax_buf_get_width(b) : 0;
}
int icons_height(icon_key_t key) {
    pax_buf_t *b = icons_get(key);
    return b ? pax_buf_get_height(b) : 0;
}

// ---- Alpha-blend a PAX ARGB icon into our RGB565 framebuffer --------

// Inline copy of fbdraw's user_offset() — kept private here so we
// don't need to add a getter to fbdraw.h just for the icon path.
static inline size_t fb_user_offset(const fbdraw_t *fb, int ux, int uy) {
    return (size_t)ux * fb->panel_w + (fb->panel_w - 1u - (size_t)uy);
}

// 8-bit-per-channel alpha blend, then re-pack to RGB565.
// dst = src * a + dst * (1 - a)   for each channel.
static inline uint16_t blend_to_565(uint16_t dst565,
                                    uint8_t sr, uint8_t sg, uint8_t sb,
                                    uint8_t a) {
    if (a == 0xFF) return ((sr & 0xF8u) << 8) | ((sg & 0xFCu) << 3) | (sb >> 3);
    uint8_t dr = ((dst565 >> 11) & 0x1F) * 255 / 31;
    uint8_t dg = ((dst565 >> 5)  & 0x3F) * 255 / 63;
    uint8_t db = ( dst565        & 0x1F) * 255 / 31;
    uint8_t inv = 255 - a;
    uint8_t r = (uint8_t)((sr * a + dr * inv + 127) / 255);
    uint8_t g = (uint8_t)((sg * a + dg * inv + 127) / 255);
    uint8_t b = (uint8_t)((sb * a + db * inv + 127) / 255);
    return ((r & 0xF8u) << 8) | ((g & 0xFCu) << 3) | (b >> 3);
}

void icons_blit(fbdraw_t *fb, icon_key_t key, int dst_x, int dst_y) {
    pax_buf_t *src = icons_get(key);
    if (!src || !fb || !fb->pixels) return;

    int sw = pax_buf_get_width(src);
    int sh = pax_buf_get_height(src);
    int uw = (int)fb->user_w;
    int uh = (int)fb->user_h;

    for (int sy = 0; sy < sh; sy++) {
        int uy = dst_y + sy;
        if (uy < 0 || uy >= uh) continue;
        for (int sx = 0; sx < sw; sx++) {
            int ux = dst_x + sx;
            if (ux < 0 || ux >= uw) continue;

            pax_col_t c = pax_get_pixel(src, sx, sy);
            uint8_t   a = (c >> 24) & 0xFFu;
            if (a == 0) continue;
            uint8_t r = (c >> 16) & 0xFFu;
            uint8_t g = (c >>  8) & 0xFFu;
            uint8_t b =  c        & 0xFFu;

            size_t   off = fb_user_offset(fb, ux, uy);
            fb->pixels[off] = blend_to_565(fb->pixels[off], r, g, b, a);
        }
    }
}
