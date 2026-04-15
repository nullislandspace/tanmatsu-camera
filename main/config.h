#pragma once

#include <stdbool.h>
#include "esp_err.h"

// Plain-text camera configuration stored at /sd/camera.cfg. Readable
// and editable by the user on any PC — just pull the SD card and open
// the file. Format is `key=value` per line, `#` introduces a comment.
//
// Default for every option is OFF / conservative so the out-of-the-box
// experience on stock hardware is unchanged.

#define CONFIG_PATH "/sd/camera.cfg"

typedef struct {
    // Enable DW9714P manual focus controls (UP/DOWN scan + HUD readout).
    // Most users don't have this chip — default false.
    bool focus_enabled;
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
