#pragma once

#include <stddef.h>

#include "esp_err.h"

// Capture the current on-screen preview frame as a JPEG.
//
// **Precondition**: must be called from the main loop between
// camera_preview_wait_frame() and camera_preview_give_render_ready().
// Inside that window the render task is blocked on render_ready and
// cannot overwrite s_preview_buffer, so it is safe to read while the
// JPEG encoder processes it.
//
// The source is the scaled/mirrored preview buffer (RGB565), so the
// resulting image matches exactly what was on screen at the moment of
// the shutter press — correct orientation, correct colours.
//
// Writes the JPEG to <dcim_dir>/IMG_YYYYMMDD_HHMMSS.jpg via fastopen().
// On success the full path is copied into out_path; on failure out_path
// is set to an empty string.
esp_err_t photo_capture(const char *dcim_dir,
                        char       *out_path,
                        size_t      out_path_sz);
