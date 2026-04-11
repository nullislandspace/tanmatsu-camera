#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_err.h"

#include "camera_sensor.h"

// Live preview pipeline based on the OV5647 + CSI + ISP + PPA reference
// in camera.md §8 / camera_old/camera.c. The sensor source format is
// runtime-configurable via `camera_source_t` so the same code can drive
// both the photo/preview 1920x1080 RAW10 path and the video recording
// 800x640 RAW8 path — the caller switches formats on mode transitions
// (main.c handles this on F1/F2/F3 key presses) and rebuilds the
// pipeline with the appropriate source descriptor.

// Description of the sensor source currently feeding the pipeline. The
// caller gets these values from camera_sensor_set_format_*() /
// esp_cam_sensor_format_t and passes them through here so the CSI
// controller, ISP processor, and PPA SRM all agree on dimensions and
// on the raw pixel format.
typedef struct {
    uint32_t width;              // sensor active area width in pixels
    uint32_t height;             // sensor active area height in pixels
    bool     is_raw10;           // true = 10-bit raw, false = 8-bit raw
    uint32_t lane_rate_mbps;     // MIPI CSI-2 bit rate per lane, Mbps
} camera_source_t;

// Bring up the pipeline. `src` describes the sensor output format the
// CSI / ISP should expect; `req_w` / `req_h` are the MAX display-space
// dimensions the preview should fit inside; the actual preview size is
// chosen so the PPA scale factor lands exactly on an n/16 boundary
// (the PPA scale register has only 4 fractional bits), keeping aspect
// ratio and avoiding the unwritten-edge artefact caused by scale
// quantisation. Query the final preview size via
// camera_preview_get_width()/get_height() after start. Allocates every
// buffer it needs, starts the render task, and begins CSI streaming.
// The sensor must already be detected and configured to the format
// described by `src` before this call.
esp_err_t camera_preview_start(const camera_source_t *src,
                               uint32_t req_w, uint32_t req_h);

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

// Snapshot the current live frame at full sensor resolution
// (1920x1080 RGB565) for JPEG encoding, applying the preview
// pipeline's mirror_x + mirror_y correction but no rotation (so the
// resulting JPEG is a standard landscape image, not portrait).
//
// Preconditions: the preview pipeline is running AND the caller has
// paused the sensor stream so s_camera_buffer is stable (see photo.c
// for the stop-stream → wait-frame → snapshot → resume-stream flow).
//
// On success *out_buf points to a cache-line-aligned PSRAM buffer the
// caller owns and must heap_caps_free(). *out_w / *out_h return
// 1920 / 1080.
esp_err_t camera_photo_snapshot(uint8_t **out_buf,
                                uint32_t *out_w,
                                uint32_t *out_h);

// Snapshot the current live frame scaled down to a caller-provided
// YUV420 "packed" buffer for the H.264 hardware encoder. The PPA SRM
// path does colour-space conversion (RGB565 → YUV420), a uniform
// scale, and the same mirror_x + mirror_y as the preview / photo
// paths — so the recorded video matches the on-screen preview and
// any saved JPEG exactly.
//
// Output layout: the ESP32-P4 "PPA YUV420" / "O_UYY_E_VYY" packed
// format, 1.5 bytes per pixel, which is what the esp_h264 hardware
// encoder accepts directly on all ESP32-P4 silicon revisions.
//
// Parameters:
//   out_buf  — preallocated cache-line-aligned PSRAM buffer owned
//              by the caller. Must be at least (out_stride_pixels *
//              out_height * 3 / 2) bytes.
//   out_buf_sz — size of out_buf in bytes.
//   out_w, out_h — requested output block dimensions. Must match a
//              scale factor that lands on PPA's 1/16 resolution
//              boundary against the 1920x1080 source — for the
//              current recording config this is 480 × 270 (scale
//              4/16 = 0.25 exact, full source aspect preserved).
//   stride_w — stride width in pixels. For H.264 this is
//              (out_w + 15) & ~15 rounded up to the next mult of 16
//              so the encoder's internal macroblock grid is a
//              multiple of 16; the PPA writes out_w x out_h at
//              offset (0, 0) and the rest of the buffer stays at
//              its initialised value (zero on first allocation).
//   stride_h — analogous vertical stride.
//
// Preconditions: the preview pipeline is running. Uses the same
// s_ppa client as render_task / photo_snapshot — serialised via
// the internal PPA mutex, so it is safe to call from a dedicated
// recording task in parallel with the UI.
esp_err_t camera_video_snapshot(uint8_t  *out_buf,
                                size_t    out_buf_sz,
                                uint32_t  out_w,
                                uint32_t  out_h,
                                uint32_t  stride_w,
                                uint32_t  stride_h);

// Pointer to the full-size 800x640 RGB565 frame that the ISP writes into
// (pre-PPA, no scale/mirror applied). CSI DMA is continuously overwriting
// this buffer — reads are inherently racy — but it's useful for diagnostic
// dumps to see what the ISP is producing before the PPA touches it.
// Dimensions are fixed at 800x640 for the preview format.
const uint8_t *camera_preview_get_raw_pixels(void);
uint32_t       camera_preview_get_raw_width(void);
uint32_t       camera_preview_get_raw_height(void);
