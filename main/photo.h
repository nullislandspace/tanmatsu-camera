#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "camera_sensor.h"

// Capture a full-resolution 1920x1080 JPEG photo.
//
// Sequence (see camera.md §10):
//   1. Stop the sensor stream and the live preview pipeline.
//   2. Switch the sensor to its highest-resolution MIPI CSI format
//      (1920x1080 RAW10 on the OV5647).
//   3. Bring up a one-shot CSI/ISP/PPA chain at 1920x1080, drop a few
//      warm-up frames so AE/AWB has a chance to settle, capture the
//      next frame and PPA-mirror it to match the preview orientation.
//   4. Tear the capture pipeline down, switch the sensor back to the
//      preview format, and rebuild the live preview pipeline at the
//      caller-supplied target dimensions.
//   5. Hardware-JPEG-encode the captured RGB565 frame and write it to
//      <dcim_dir>/IMG_YYYYMMDD_HHMMSS.jpg via fastopen().
//
// Blocks the caller for roughly the time required to perform the
// format switch + warm-up + capture + encode + write (typically
// 600-1500 ms). The preview is frozen on the last rendered frame
// during that window.
//
// preview_req_w/preview_req_h are forwarded to camera_preview_start()
// when the preview is brought back up, so the caller should pass the
// same values it used the first time around.
//
// On success the full path is copied into out_path; on failure out_path
// is set to an empty string.
esp_err_t photo_capture(camera_sensor_t *sensor,
                        uint32_t         preview_req_w,
                        uint32_t         preview_req_h,
                        const char      *dcim_dir,
                        char            *out_path,
                        size_t           out_path_sz);
