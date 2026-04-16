#pragma once

#include "esp_err.h"

// Mount the wear-levelled FAT partition (label "locfd") at /int. The
// shared internal filesystem holds icons, app data, etc. — must be
// mounted before any /int/* path is opened.
esp_err_t intfs_init(void);
