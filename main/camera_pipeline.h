#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#include "camera_sensor.h"

// Live preview pipeline based on the OV5647 + CSI + ISP + PPA reference
// in camera.md §8 / camera_old/camera.c. Fixed to the 800x640 RAW8 sensor
// format, RGB565 inline via the ISP, and PPA SRM scaling into a preview
// buffer that the main UI blits to the display.

// Bring up the pipeline. req_w/req_h are the MAX display-space dimensions
// the preview should fit inside; the actual preview size is chosen so the
// PPA scale factor lands exactly on an n/16 boundary (the PPA scale
// register has only 4 fractional bits), keeping aspect ratio and avoiding
// the unwritten-edge artefact caused by scale quantisation. Query the
// final preview size via camera_preview_get_width()/get_height() after
// start. Allocates every buffer it needs, starts the receive and render
// tasks, and begins CSI streaming. The sensor must already be detected
// and configured to the 800x640 RAW8 format.
esp_err_t camera_preview_start(uint32_t req_w, uint32_t req_h);

// Tear down the pipeline and free all buffers.
void camera_preview_stop(void);

// Signal that the main loop has finished consuming the latest preview frame
// and is ready to accept the next CSI transaction. This replaces the
// LV_EVENT_REFR_READY hook the old code used.
void camera_preview_give_render_ready(void);

// Wait (with timeout) for the next scaled preview frame to be available.
// Returns ESP_OK if a fresh frame is ready, ESP_ERR_TIMEOUT otherwise.
esp_err_t camera_preview_wait_frame(uint32_t timeout_ms);

// Pointer to the latest preview frame (RGB565), plus its dimensions. Valid
// until the pipeline is stopped. Safe to read while the main loop holds a
// frame — back-pressure via camera_preview_give_render_ready() prevents the
// pipeline from overwriting the buffer before the UI is done with it.
const uint8_t *camera_preview_get_pixels(void);
uint32_t       camera_preview_get_width(void);
uint32_t       camera_preview_get_height(void);

// Capture a single full-resolution (1920x1080) frame from the camera.
//
// Preconditions: the preview pipeline must already be stopped
// (camera_preview_stop()) and the sensor must already be switched to
// the 1920x1080 RAW10 format via camera_sensor_set_format_photo(). This
// call will stream the sensor, discard `discard_frames` warm-up frames
// so AE/AWB can settle, capture the next one, PPA-mirror it to match the
// preview orientation, then stop the sensor and tear down the CSI/ISP/PPA
// chain it built internally.
//
// On success, *out_buf points to a newly allocated cache-line aligned
// RGB565 buffer in PSRAM containing the final mirrored frame. The caller
// owns the buffer and MUST free it with heap_caps_free(). *out_w and
// *out_h receive the image dimensions (1920x1080). After this returns
// the caller is responsible for switching the sensor back to the preview
// format and restarting camera_preview_start().
esp_err_t camera_photo_capture(camera_sensor_t *sensor,
                               int discard_frames,
                               uint8_t **out_buf,
                               uint32_t *out_w,
                               uint32_t *out_h);

// Pointer to the full-size 800x640 RGB565 frame that the ISP writes into
// (pre-PPA, no scale/mirror applied). CSI DMA is continuously overwriting
// this buffer — reads are inherently racy — but it's useful for diagnostic
// dumps to see what the ISP is producing before the PPA touches it.
// Dimensions are fixed at 800x640 for the preview format.
const uint8_t *camera_preview_get_raw_pixels(void);
uint32_t       camera_preview_get_raw_width(void);
uint32_t       camera_preview_get_raw_height(void);
