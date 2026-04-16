#pragma once

#include <stdbool.h>

#include "fbdraw.h"
#include "pax_gfx.h"

// Function-key icons loaded from /int/icons/*.png at boot. The
// launcher installs a fresh set there; this module reads whatever
// is there and converts each PNG into a pax_buf_t kept in PSRAM
// for the lifetime of the app.
//
// Missing or undecodable files are tolerated: icons_get() returns
// NULL and the HUD falls back to plain text labels.

typedef enum {
    ICON_ESC = 0,
    ICON_F1,
    ICON_F2,
    ICON_F3,
    ICON_F4,
    ICON_F5,
    ICON_F6,
    ICON_KEY_COUNT,
} icon_key_t;

// Decode all icons from /int/icons/. Safe to call once at boot. The
// /int filesystem must already be mounted.
void       icons_load(void);
pax_buf_t *icons_get(icon_key_t key);
bool       icons_any_missing(void);
int        icons_width (icon_key_t key);  // 0 if not loaded
int        icons_height(icon_key_t key);  // 0 if not loaded

// Alpha-blend an icon into our RGB565 framebuffer at user
// coordinates (dst_x, dst_y). No-op if the icon is missing.
void       icons_blit(fbdraw_t *fb, icon_key_t key, int dst_x, int dst_y);
