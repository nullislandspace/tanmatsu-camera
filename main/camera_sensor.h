#pragma once

#include <stdbool.h>
#include "esp_cam_sensor.h"
#include "esp_err.h"
#include "esp_sccb_intf.h"

// Identifies which sensor model camera_sensor_detect() bound to. Used
// by the pipeline + main loop to dispatch on per-sensor differences:
// the OV5647 path uses the P4 ISP demosaicer (RAW Bayer input), while
// OV5640/OV5645 are configured to deliver RGB565 directly and the ISP
// runs in bypass.
typedef enum {
    CAMERA_SENSOR_UNKNOWN = 0,
    CAMERA_SENSOR_OV5647,
    CAMERA_SENSOR_OV5640,
    CAMERA_SENSOR_OV5645,
    // OV9281: 1MP global-shutter monochrome RAW10. Reuses the OV5647-
    // class RAW Bayer pipeline; the demosaicer produces approximately
    // grayscale output because every input pixel carries the same
    // luminance signal regardless of which Bayer position the
    // demosaicer assumes.
    CAMERA_SENSOR_OV9281,
} camera_sensor_kind_t;

// A single detected camera sensor plus the SCCB handle it was opened with.
// Kept in one struct so the owning module can release both on teardown.
typedef struct {
    esp_cam_sensor_device_t *device;
    esp_sccb_io_handle_t     sccb;
    camera_sensor_kind_t     kind;
    // Sensor-side base values for camera_sensor_set_preview_fps. Captured
    // from the active esp_cam_sensor_format_t + a VTS register read-back
    // immediately after every set_format_* call, so the same fps math
    // works regardless of which sensor + format pair is active.
    uint32_t base_vts_lines;
    uint32_t base_fps;
    // VTS actually programmed right now — equal to base_vts_lines until
    // camera_sensor_set_preview_fps() stretches it. The manual-exposure
    // path needs the *current* value, because the integration time
    // ceiling is (VTS - a few lines) and capping the frame rate is
    // precisely what buys the extra headroom.
    uint32_t cur_vts_lines;
    // Duration of one sensor row, derived from the format's nominal fps
    // and the VTS the driver wrote for it: a frame is base_vts_lines
    // rows long and lasts 1/base_fps seconds. Independent of any later
    // VTS override (stretching VTS adds rows, it doesn't slow them),
    // which is what lets us express exposure in real microseconds
    // instead of sensor-specific line counts. 0 = unknown.
    uint32_t row_time_ns;
    // Original AEC/AGC mode register (0x3503) as the driver's init
    // table left it, saved the first time we take manual control so
    // camera_sensor_set_auto_exposure() can put back exactly what was
    // there rather than a guess at what "auto" means on this part.
    uint8_t  ae_mode_saved;
    bool     ae_mode_valid;
} camera_sensor_t;

// Detect a camera sensor on the BSP's primary I2C bus. Iterates every
// registered esp_cam_sensor detect function; the first one that responds
// wins. Wraps bus access in the BSP's I2C claim/release semaphore.
esp_err_t camera_sensor_detect(camera_sensor_t *out);

// Release the SCCB handle for a previously-detected sensor.
void camera_sensor_release(camera_sensor_t *sensor);

// Select the sensor format matching exact_name. On success, camera_width /
// camera_height / lane count are reported through out_fmt (may be NULL to
// discard). SCCB access is serialised.
esp_err_t camera_sensor_set_format_by_name(camera_sensor_t *sensor, const char *exact_name,
                                           esp_cam_sensor_format_t *out_fmt);

// Convenience wrappers that pick named formats per sensor:
//   - preview = the sensor's preferred preview mode. On OV5647 this is
//                the highest-resolution MIPI CSI mode (1920x1080 RAW10
//                @30fps). On OV5640/OV5645 it is a single shared
//                RGB565 mode (1280x720 / 1280x960) that doubles as the
//                photo + video format — see camera.md.
//   - video   = a sensor-specific recording mode. On OV5647 this is a
//                lower-resolution RAW8 preset (800x640 @50fps) that
//                leaves enough PPA throughput for real-time encoding.
//                On OV5640/OV5645 it returns the same RGB565 format as
//                preview — those sensors have a single shared format
//                across all three modes, so the PHOTO↔VIDEO transition
//                is effectively a no-op.
//   - photo   = walks the format list and picks the highest-resolution
//                CSI mode (useful when driver version ships a new
//                high-res preset and we want to auto-pick it).
esp_err_t camera_sensor_set_format_preview(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt);
esp_err_t camera_sensor_set_format_video(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt);
esp_err_t camera_sensor_set_format_photo(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt);

// Returns the sensor model name string ("OV5647" / "OV5640" / "OV5645" /
// "?"). The pointer is owned by the underlying esp_cam_sensor driver
// (compile-time string literal) and is valid for the lifetime of the
// detected sensor handle.
const char *camera_sensor_name(const camera_sensor_t *sensor);

// Start or stop the sensor output stream via S_STREAM ioctl.
esp_err_t camera_sensor_stream(camera_sensor_t *sensor, bool enable);

// Read a single 8-bit sensor register over SCCB via ESP_CAM_SENSOR_IOC_G_REG.
// regaddr is the sensor-side 16-bit register address. SCCB access is serialised.
esp_err_t camera_sensor_read_reg(camera_sensor_t *sensor, uint16_t regaddr, uint8_t *out_val);

// Write a single 8-bit sensor register over SCCB via ESP_CAM_SENSOR_IOC_S_REG.
// regaddr is the sensor-side 16-bit register address. SCCB access is serialised.
esp_err_t camera_sensor_write_reg(camera_sensor_t *sensor, uint16_t regaddr, uint8_t value);

// Override the sensor's frame rate on the CURRENT format by
// reprogramming the OmniVision Timing Group VTS (vertical total size)
// register pair (0x380E/0x380F). Must be called AFTER one of the
// camera_sensor_set_format_*() functions — those record the format's
// native fps and the driver's freshly-written base VTS into
// `sensor->base_fps` / `sensor->base_vts_lines`, which this function
// then scales.
//
// target_fps must be <= the format's native framerate. The 0x380E/F
// register pair lives at the same address on OV5640/OV5645/OV5647 (it
// is part of the standard OV timing bank) so this works across all
// supported sensors. The VTS register ceiling is 16 bits, which caps
// the minimum reachable framerate at ~native_fps * base_vts / 65535 —
// well below anything we'd want in practice.
esp_err_t camera_sensor_set_preview_fps(camera_sensor_t *sensor, uint32_t target_fps);

// ---------------------------------------------------------------------
// Manual exposure / analog gain
// ---------------------------------------------------------------------
//
// Brightness is exposed to the UI as a single integer "step" rather
// than as two independent registers, because the two are not
// independently useful: you want more light, and exposure time is
// always the cheaper way to buy it (analog gain amplifies read noise,
// integration time does not). One step is exactly one stop — each
// press doubles or halves the light reaching the sensor — and the
// ladder spends its budget on exposure first, rolling over into
// analog gain only once the frame period leaves no more room.
//
// Step 1 is ~30 us at unity gain; step CAMERA_BRIGHT_MAX is ~15 stops
// brighter. That is a deliberately wide range: the OV9281 has no
// on-chip AE at all, so this control is the *only* thing standing
// between direct sunlight and a dim indoor room.
#define CAMERA_BRIGHT_MIN      1
#define CAMERA_BRIGHT_MAX      16
#define CAMERA_BRIGHT_DEFAULT  10

// What a brightness step actually resolved to on this sensor, at this
// frame rate. Surfaced to the HUD because when a frame comes back
// black during bring-up, "1.9ms 1.0x" tells you whether to reach for
// the exposure ladder or for the lens cap.
typedef struct {
    uint32_t exposure_us;     // integration time actually programmed
    uint32_t exposure_lines;  // ... the same thing in sensor rows
    uint16_t gain_q4;         // analog gain, 4 fractional bits (16 = 1.0x)
    bool     exposure_capped; // ladder hit the VTS ceiling and used gain
} camera_exposure_t;

// True when the sensor runs its own AE loop that would fight a manual
// exposure write until it is switched off. OV5640/OV5645/OV5647 do;
// the OV9281 is a machine-vision part with no on-chip AE whatsoever,
// which is why it needs this control far more than the others.
bool camera_sensor_has_auto_exposure(const camera_sensor_t *sensor);

// Apply a brightness step (clamped to CAMERA_BRIGHT_MIN..MAX). Switches
// the sensor's AEC/AGC to manual first when it has an auto loop, then
// writes the exposure and analog-gain registers. The resolved values
// are reported through out (may be NULL).
//
// Must be called AFTER camera_sensor_set_format_*() and after any
// camera_sensor_set_preview_fps() — set_format rewrites the sensor's
// whole register bank (wiping the exposure) and the fps override moves
// the exposure ceiling. Returns ESP_ERR_INVALID_STATE if no format has
// been bound yet or the row-time could not be derived.
esp_err_t camera_sensor_set_brightness(camera_sensor_t *sensor, int step,
                                       camera_exposure_t *out);

// Read the exposure + gain registers back. Used to seed the ladder from
// whatever an on-chip AE loop had converged to at the moment the user
// takes manual control, so the picture doesn't jump on the first press.
esp_err_t camera_sensor_read_exposure(camera_sensor_t *sensor,
                                      camera_exposure_t *out);

// Nearest ladder step to an arbitrary exposure/gain pair — the inverse
// of camera_sensor_set_brightness(). Returns a value in
// CAMERA_BRIGHT_MIN..CAMERA_BRIGHT_MAX.
int camera_sensor_brightness_step_for(const camera_sensor_t *sensor,
                                      const camera_exposure_t *exp);

// Hand exposure control back to the sensor's own AE loop by restoring
// the AEC/AGC mode register saved on the first manual write. No-op
// (ESP_ERR_NOT_SUPPORTED) on sensors with no auto loop to return to.
esp_err_t camera_sensor_set_auto_exposure(camera_sensor_t *sensor);
