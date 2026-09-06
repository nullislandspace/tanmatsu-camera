#include "radio.h"

#include <stdio.h>

#include "ble_peer.h"
#include "catprinter.h"
#include "esp_heap_caps.h"
#include "esp_hosted.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "wifi_connection.h"
#include "wifi_remote.h"

static const char *TAG = "radio";

static bool s_up           = false;
static bool s_scan_on_sync = false;

bool radio_is_up(void) { return s_up; }

void radio_log_heap(const char *stage) {
    ESP_LOGI(TAG, "heap @ %-14s internal=%7u  psram=%8u  (largest int block=%7u)",
             stage,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

// ---------------------------------------------------------------------------
// BLE central scaffold.
//
// Everything from here down to radio_start() is the PR author's code,
// reindented to this file's style but otherwise unchanged in behaviour. We
// have no BLE printer to test against, so this is the part where staying
// close to a version that has actually talked to hardware is worth more
// than any cleanup we might prefer. The deliberate departures are marked
// individually, and all of them are about the radio never being able to
// take the camera down with it.
// ---------------------------------------------------------------------------

static int ble_gap_event(struct ble_gap_event *event, void *arg);

// Defined by NimBLE's store/config component (CONFIG_BT_NIMBLE_NVS_PERSIST).
void ble_store_config_init(void);

// Not reentrant: the returned buffer is overwritten by the next call. Only
// ever used to build a log line, and only from the NimBLE host task.
static char *ble_addr_str(const void *addr) {
    static char    buf[6 * 2 + 5 + 1];
    const uint8_t *u8p = addr;
    sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", u8p[5], u8p[4], u8p[3], u8p[2],
            u8p[1], u8p[0]);
    return buf;
}

static void ble_on_reset(int reason) {
    s_up = false;
    ESP_LOGE(TAG, "BLE resetting state: reason=%d", reason);
}

// Fires once GATT service/characteristic/descriptor discovery on a newly
// connected peer completes. Dumps the discovered GATT table so it's easy
// to see what a given peripheral exposes, then hands the peer to any
// driver (currently just catprinter.c) that wants to look up its own
// characteristics/descriptors via peer_chr_find_uuid()/peer_dsc_find_uuid().
static void ble_on_disc_complete(const struct peer *peer, int status, void *arg) {
    if (status != 0) {
        ESP_LOGE(TAG, "BLE service discovery failed; status=%d conn_handle=%d",
                 status, peer->conn_handle);
        ble_gap_terminate(peer->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }
    ESP_LOGI(TAG, "BLE service discovery complete; conn_handle=%d",
             peer->conn_handle);
    peer_list_all(peer);
    catprinter_on_disc_complete(peer);
}

// Hook for deciding whether a discovered device is worth connecting to.
// Currently only the cat printer's advertised service UUID is
// recognised; a future feature adding another peripheral type would
// extend this check.
static bool ble_should_connect(const struct ble_gap_disc_desc *disc) {
    return catprinter_matches_disc(disc);
}

static void ble_connect_to_device(const ble_addr_t *addr) {
    uint8_t own_addr_type;
    int     rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    rc = ble_gap_disc_cancel();
    if (rc != 0) {
        ESP_LOGD(TAG, "failed to cancel scan; rc=%d", rc);
        return;
    }

    rc = ble_gap_connect(own_addr_type, addr, 30000, NULL, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to connect to BLE device; addr_type=%d addr=%s; rc=%d",
                 addr->type, ble_addr_str(addr->val), rc);
    }
}

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_gap_conn_desc desc;
    struct ble_hs_adv_fields fields;
    int                      rc;

    switch (event->type) {
        case BLE_GAP_EVENT_DISC:
            rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                         event->disc.length_data);
            if (rc != 0) return 0;
            ESP_LOGI(TAG, "BLE device discovered: addr=%s%s%s",
                     ble_addr_str(event->disc.addr.val),
                     fields.name_len ? " name=" : "",
                     fields.name_len ? (const char *)fields.name : "");
            if (ble_should_connect(&event->disc)) {
                ble_connect_to_device(&event->disc.addr);
            }
            return 0;

        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
                if (rc != 0) return 0;
                ESP_LOGI(TAG, "BLE connected to %s",
                         ble_addr_str(desc.peer_ota_addr.val));
                rc = peer_add(event->connect.conn_handle);
                if (rc != 0) {
                    ESP_LOGE(TAG, "failed to add peer; rc=%d", rc);
                    return 0;
                }
                rc = ble_gattc_exchange_mtu(event->connect.conn_handle, NULL, NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "failed to negotiate MTU; rc=%d", rc);
                }
                rc = peer_disc_all(event->connect.conn_handle, ble_on_disc_complete,
                                   NULL);
                if (rc != 0) {
                    ESP_LOGE(TAG, "failed to discover services; rc=%d", rc);
                }
            } else {
                ESP_LOGE(TAG, "BLE connection failed; status=%d",
                         event->connect.status);
            }
            return 0;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE disconnected; reason=%d", event->disconnect.reason);
            catprinter_on_disconnect(event->disconnect.conn.conn_handle);
            peer_delete(event->disconnect.conn.conn_handle);
            // The PR leaves scanning stopped after a disconnect, which means
            // one unplugged printer ends discovery for the rest of the boot.
            // Resume it, but only if we were scanning to begin with.
            if (s_scan_on_sync) ble_scan_start();
            return 0;

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGI(TAG, "BLE discovery complete; reason=%d",
                     event->disc_complete.reason);
            return 0;

        case BLE_GAP_EVENT_NOTIFY_RX:
            catprinter_on_notify(event->notify_rx.conn_handle,
                                 event->notify_rx.attr_handle, event->notify_rx.om);
            return 0;

        case BLE_GAP_EVENT_ENC_CHANGE:
            rc = ble_gap_conn_find(event->enc_change.conn_handle, &desc);
            if (rc != 0) return rc;
            ESP_LOGI(TAG, "BLE encryption change for %s; status=%d",
                     ble_addr_str(desc.peer_ota_addr.val), event->enc_change.status);
            return 0;

        case BLE_GAP_EVENT_MTU: {
            ESP_LOGI(TAG, "BLE MTU update; conn_handle=%d channel=%d mtu=%d",
                     event->mtu.conn_handle, event->mtu.channel_id,
                     event->mtu.value);
            struct peer *peer = peer_find(event->mtu.conn_handle);
            if (peer != NULL) {
                peer->mtu = event->mtu.value;
            }
            return 0;
        }

        case BLE_GAP_EVENT_REPEAT_PAIRING:
            // We already have a bond with the peer but it's attempting a new
            // secure link — drop the old bond and accept the new one.
            rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
            if (rc != 0) return rc;
            ble_store_util_delete_peer(&desc.peer_id_addr);
            return BLE_GAP_REPEAT_PAIRING_RETRY;

        default:
            return 0;
    }
}

void ble_scan_start(void) {
    uint8_t                    own_addr_type;
    struct ble_gap_disc_params disc_params = {0};

    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "error determining address type; rc=%d", rc);
        return;
    }

    disc_params.filter_duplicates = 1;
    disc_params.passive           = 1;
    disc_params.itvl              = 0;
    disc_params.window            = 0;
    disc_params.filter_policy     = 0;
    disc_params.limited           = 0;

    rc = ble_gap_disc(own_addr_type, BLE_HS_FOREVER, &disc_params, ble_gap_event,
                      NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "error initiating BLE discovery; rc=%d", rc);
    }
}

static void ble_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "BLE util ensure addr failed: rc=%d", rc);
        return;
    }
    s_up = true;
    ESP_LOGI(TAG, "NimBLE host synced");
    // The one behavioural change to the PR's scan handling: it is gated on
    // the BLE Scan setting. Scanning forever is what the printer wants and
    // what phase 3.4 measured, but it is also permanent SDIO traffic, so it
    // stays switchable.
    if (s_scan_on_sync) {
        ESP_LOGI(TAG, "BLE scanning...");
        ble_scan_start();
    } else {
        ESP_LOGI(TAG, "BLE scan disabled by config; printer will not be found");
    }
}

static void ble_host_task(void *param) {
    (void)param;
    ESP_LOGI(TAG, "BLE host task started");
    // Returns only once nimble_port_stop() is called.
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t radio_start(bool scan_on_sync) {
    s_scan_on_sync = scan_on_sync;

    // Brings up the esp-hosted transport to the C6 co-processor. The BT
    // controller lives on that same co-processor and is started separately,
    // below, before the NimBLE host is handed control.
    esp_err_t err = wifi_remote_initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "wifi_remote_initialize: %d", err);
        return err;
    }
    wifi_connection_init_stack();

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

    ble_hs_cfg.reset_cb        = ble_on_reset;
    ble_hs_cfg.sync_cb         = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // The PR asserts on both of these. An assert here would abort the whole
    // app because a printer helper could not allocate, which is exactly the
    // failure mode the rest of this file exists to avoid. peer_init() only
    // fails on OOM, and without it every peer_* call below is unsafe, so
    // that one is fatal to the radio -- but not to the camera.
    int rc = peer_init(MYNEWT_VAL(BLE_MAX_CONNECTIONS), 64, 64, 64);
    if (rc != 0) {
        ESP_LOGE(TAG, "peer_init failed; rc=%d -- BLE unavailable", rc);
        return ESP_ERR_NO_MEM;
    }

    rc = ble_svc_gap_device_name_set("tanmatsu-camera");
    if (rc != 0) {
        ESP_LOGW(TAG, "ble_svc_gap_device_name_set: %d", rc);
    }

    ble_store_config_init();

    nimble_port_freertos_init(ble_host_task);
    return ESP_OK;
}
