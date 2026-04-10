#pragma once

#include <stdbool.h>
#include "esp_cam_sensor.h"
#include "esp_err.h"
#include "esp_sccb_intf.h"

// A single detected camera sensor plus the SCCB handle it was opened with.
// Kept in one struct so the owning module can release both on teardown.
typedef struct {
    esp_cam_sensor_device_t *device;
    esp_sccb_io_handle_t     sccb;
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

// Convenience wrappers that pick the preview (800x640 RAW8) and photo
// (1920x1080 RAW10) formats on the OV5647. Both walk the sensor's exposed
// format list so behaviour is driver-version tolerant — the format chosen
// is written back to out_fmt when provided.
esp_err_t camera_sensor_set_format_preview(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt);
esp_err_t camera_sensor_set_format_photo(camera_sensor_t *sensor, esp_cam_sensor_format_t *out_fmt);

// Start or stop the sensor output stream via S_STREAM ioctl.
esp_err_t camera_sensor_stream(camera_sensor_t *sensor, bool enable);

// Read a single 8-bit sensor register over SCCB via ESP_CAM_SENSOR_IOC_G_REG.
// regaddr is the sensor-side 16-bit register address. SCCB access is serialised.
esp_err_t camera_sensor_read_reg(camera_sensor_t *sensor, uint16_t regaddr, uint8_t *out_val);

// Write a single 8-bit sensor register over SCCB via ESP_CAM_SENSOR_IOC_S_REG.
// regaddr is the sensor-side 16-bit register address. SCCB access is serialised.
esp_err_t camera_sensor_write_reg(camera_sensor_t *sensor, uint16_t regaddr, uint8_t value);

// Override the sensor's frame rate on the CURRENT (preview) format by
// reprogramming the Timing Group VTS (vertical total size) register pair.
// Must be called AFTER camera_sensor_set_format_preview() — the format
// register table writes a default VTS that we then override.
//
// target_fps must be <= the format's native framerate (lower = longer
// VTS = longer frame time). On the OV5647 800x640 preset the native
// rate is 50 fps, so valid values are roughly 5..50 fps.
//
// The VTS register ceiling is 16 bits, which caps the minimum reachable
// framerate at ~native_fps * base_vts / 65535 — well below anything
// we'd want in practice.
esp_err_t camera_sensor_set_preview_fps(camera_sensor_t *sensor, uint32_t target_fps);
