#include "intfs.h"

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "wear_levelling.h"

static const char TAG[] = "intfs";

esp_err_t intfs_init(void) {
    static wl_handle_t wl = WL_INVALID_HANDLE;
    if (wl != WL_INVALID_HANDLE) return ESP_OK;  // already mounted

    const esp_vfs_fat_mount_config_t cfg = {
        .format_if_mount_failed = false,
        .max_files              = 10,
        .allocation_unit_size   = CONFIG_WL_SECTOR_SIZE,
    };

    esp_err_t r = esp_vfs_fat_spiflash_mount_rw_wl("/int", "locfd", &cfg, &wl);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "mount /int failed: %s", esp_err_to_name(r));
        return r;
    }
    ESP_LOGI(TAG, "/int mounted");
    return ESP_OK;
}
