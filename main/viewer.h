#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

// Image viewer: scans the DCIM folder for IMG_*.jpg, sorts them
// descending by filename (which, with our YYYYMMDD_HHMMSS stamps,
// puts the newest first), hardware-decodes one image at a time and
// pre-scales it to fit inside (max_w, max_h) while preserving aspect
// ratio. The pre-scaled RGB565 buffer is what the main loop blits.
//
// Decoder and PPA client are created on open() and reused across
// navigation, so stepping through the list is just one JPEG decode +
// one PPA pass per image.

esp_err_t viewer_open(const char *dcim_dir, uint32_t max_w, uint32_t max_h);

// Step to the next older (► / NEXT) or next newer (◄ / PREV) image.
// No-op (returns ESP_OK) at the ends of the list.
esp_err_t viewer_next(void);
esp_err_t viewer_prev(void);

// Release every resource the viewer holds. Idempotent — safe to call
// without a matching open().
void viewer_close(void);

// True iff the viewer has at least one decoded image in s_disp_buf.
bool viewer_has_image(void);

// The pre-scaled RGB565 bitmap of the current image. Stride equals
// viewer_get_width(). The pointer stays valid until the next
// viewer_next/prev/close call.
const uint8_t *viewer_get_pixels(void);
uint32_t       viewer_get_width(void);
uint32_t       viewer_get_height(void);

// Basename of the current file, e.g. "IMG_20260410_150809.jpg".
// Returns an empty string if no image is loaded.
const char *viewer_get_filename(void);

// 0-based index into the sorted list and the total count, for HUD.
int viewer_get_index(void);
int viewer_get_total(void);
