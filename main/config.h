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

#define CONFIG_PATH                 "/sd/camera.cfg"
#define CONFIG_FOCUS_DRIVER_MAXLEN  16

typedef struct {
    char focus_driver[CONFIG_FOCUS_DRIVER_MAXLEN];
    bool focus_enabled;
    bool autofocus_enabled;
    bool rotate_180;
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
