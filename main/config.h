#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Plain-text camera configuration stored at /sd/camera.cfg. Readable
// and editable by the user on any PC — just pull the SD card and open
// the file. Format is `key=value` per line, `#` introduces a comment.
//
// Schema (see config.c for the file header that gets seeded):
//   focus_driver=<name>     name of the focus driver to use; one of
//                            the entries in focus_driver_registry[].
//                            "simulator" by default — simulator is
//                            always present so the focus + autofocus
//                            UI is testable on stock hardware.
//   focus_enabled=<0|1>     master enable for the focus subsystem.
//                            When 1 the boot path activates the
//                            chosen driver; when 0 nothing happens.
//   autofocus_enabled=<0|1> enable the hardware AF state machine.
//                            Ignored unless focus_enabled=1 *and*
//                            the driver actually probes successfully.
//   rotate_180=<0|1>        flip the camera image 180° (preview AND
//                            saved photos/videos). Use when the
//                            sensor is physically mounted upside
//                            down. Does NOT affect viewing of
//                            already-saved photos.
//   mic_type=<name>         I2S microphone type wired to I2S1
//                            (pins E8/E9/E10). One of:
//                              none      — no mic (silent audio track)
//                              inmp441   — INMP441 MEMS mic
//                              ics43434  — ICS43434 MEMS mic
//                            When not "none", video mode captures
//                            live audio and shows a HUD level meter.
//                            For backwards compatibility, the legacy
//                            `mic_enabled=0|1` key is still parsed
//                            (1 → inmp441, 0 → none).
//   auto_exposure=<0|1>     let exposure be chosen automatically. On a
//                            sensor with its own AE loop (OV5640/45/47)
//                            that is the on-chip loop, left untouched;
//                            on one without (OV9281) it is the software
//                            loop in autoexposure.c. Turning this off
//                            switches to the fixed cam_brightness step.
//                            Toggled in the F4 menu, or by pressing Q/A
//                            (off) / stepping below step 1 (on).
//   cam_brightness=<1..16>  the manual exposure STEP used when
//                            auto_exposure=0. One step is one stop;
//                            step 1 is ~30 us at unity gain. Adjusted
//                            live with Q (brighter) / A (darker) in
//                            photo and video mode.
//   mic_gain=<1..8>         digital gain STEP applied during slot
//                            extraction (before the LPF + resampler).
//                            Not a raw multiplier — each step is
//                            ~5.7 dB, from 1x at step 1 to 100x at
//                            step 8 (see MIC_GAIN_MULT in
//                            microphone.c). Raise for quiet
//                            environments, lower if loud speech is
//                            clipping.

#define CONFIG_PATH                 "/sd/camera.cfg"
#define CONFIG_FOCUS_DRIVER_MAXLEN  16
#define CONFIG_MIC_GAIN_MIN         1
#define CONFIG_MIC_GAIN_MAX         8
#define CONFIG_MIC_GAIN_DEFAULT     4
// Mirrors CAMERA_BRIGHT_MIN..MAX in camera_sensor.h. Kept as plain
// literals here so config.c does not have to pull in the
// esp_cam_sensor headers; main.c static-asserts that the two agree.
#define CONFIG_CAM_BRIGHTNESS_MIN       1
#define CONFIG_CAM_BRIGHTNESS_MAX       16
#define CONFIG_CAM_BRIGHTNESS_DEFAULT   10
#define CONFIG_AUTO_EXPOSURE_DEFAULT    true

typedef enum {
    MIC_TYPE_NONE = 0,
    MIC_TYPE_INMP441,
    MIC_TYPE_ICS43434,
} mic_type_t;

// Short name suitable for the config file (e.g. "inmp441").
const char *mic_type_config_name(mic_type_t t);
// Human-readable label for the config menu (e.g. "INMP441").
const char *mic_type_display_name(mic_type_t t);

typedef struct {
    char       focus_driver[CONFIG_FOCUS_DRIVER_MAXLEN];
    bool       focus_enabled;
    bool       autofocus_enabled;
    bool       rotate_180;
    mic_type_t mic_type;
    int        mic_gain;
    bool       auto_exposure;
    int        cam_brightness;
} camera_config_t;

// Populate *out with defaults, then overlay any values found in
// CONFIG_PATH. If the file does not exist yet and the SD card is
// mounted, write a seed file containing the defaults + a comment
// header. Returns ESP_OK if defaults were produced even when the
// file could not be read, ESP_FAIL only on a truly unexpected error.
esp_err_t config_load(camera_config_t *out);

// Rewrite CONFIG_PATH with the given values plus a comment header.
// Fails with ESP_ERR_INVALID_STATE if the SD card is not mounted.
esp_err_t config_save(const camera_config_t *cfg);
