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
