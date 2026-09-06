#include "radio.h"

#include "esp_heap_caps.h"
#include "esp_hosted.h"
#include "esp_log.h"
#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "wifi_connection.h"
#include "wifi_remote.h"

static const char *TAG = "radio";

static bool s_up            = false;
static bool s_scan_on_sync  = false;

bool radio_is_up(void) { return s_up; }

void radio_log_heap(const char *stage) {
    ESP_LOGI(TAG, "heap @ %-14s internal=%7u  psram=%8u  (largest int block=%7u)",
             stage,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

// Passive scan, duplicates filtered, no connect. Phase 3 only needs the
// radio to be *busy* in the way the printer would make it busy -- the
// PR scans with BLE_HS_FOREVER, so that is what gets measured.
static void start_scan(void) {
    uint8_t own_addr_type;
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto: %d", rc);
        return;
    }

    struct ble_gap_disc_params params = {
        .passive           = 1,
        .filter_duplicates = 1,
    };
    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &params, NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_disc: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE scanning (passive, forever)");
}

static void on_sync(void) {
    // Address determination has to happen after sync, not before.
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_util_ensure_addr: %d", rc);
        return;
    }
    s_up = true;
    ESP_LOGI(TAG, "NimBLE host synced");
    if (s_scan_on_sync) start_scan();
}

static void on_reset(int reason) {
    s_up = false;
    ESP_LOGW(TAG, "NimBLE host reset, reason %d", reason);
}

static void host_task(void *param) {
    (void)param;
    nimble_port_run();            // returns only on nimble_port_stop()
    nimble_port_freertos_deinit();
}

esp_err_t radio_start(bool scan_on_sync) {
    s_scan_on_sync = scan_on_sync;

    // Brings up the esp-hosted transport to the C6 co-processor. The
    // BT controller lives on that same co-processor and is started
    // separately, below, before the NimBLE host is handed control.
    esp_err_t err = wifi_remote_initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi_remote_initialize: %d", err);
        return err;
    }
    wifi_connection_init_stack();

    // Non-fatal: WiFi is up at this point, and a BLE-less radio is
    // still a working camera. The printer is what suffers.
    if (esp_hosted_bt_controller_init() != ESP_OK) {
        ESP_LOGW(TAG, "BT controller init failed");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (esp_hosted_bt_controller_enable() != ESP_OK) {
        ESP_LOGW(TAG, "BT controller enable failed");
        return ESP_ERR_NOT_SUPPORTED;
    }

    err = nimble_port_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: %d", err);
        return err;
    }

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb  = on_sync;

    int rc = ble_svc_gap_device_name_set("tanmatsu-camera");
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_svc_gap_device_name_set: %d", rc);
    }

    nimble_port_freertos_init(host_task);
    return ESP_OK;
}
