#pragma once

#include "esp_err.h"
#include <stdbool.h>

// Initialize SD card power and mount filesystem at /sd
esp_err_t sdcard_init(void);

// Check if SD card is mounted
bool sdcard_is_mounted(void);
