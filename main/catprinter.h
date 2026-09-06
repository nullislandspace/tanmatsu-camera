#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "ble_peer.h"
#include "host/ble_gap.h"

struct os_mbuf;

// GATT protocol driver + Floyd-Steinberg image printer for the "cat
// printer" family of thermal BLE receipt printers (GT01/MX10 clones,
// service UUID 0xae30). Owns its own connection/subscription state;
// main.c's BLE central scaffold just routes the relevant GAP events
// here so the rest of the app can stay unaware of the printer's GATT
// protocol.

// Does `disc` advertise the cat printer service? Used by the BLE
// central scaffold's auto-connect filter.
bool catprinter_matches_disc(const struct ble_gap_disc_desc *disc);

// Call once GATT discovery on a newly connected peer completes.
// No-op unless `peer` actually exposes the cat printer service, in
// which case this subscribes to its status-notification
// characteristic.
void catprinter_on_disc_complete(const struct peer *peer);

// Call on BLE_GAP_EVENT_DISCONNECT so the driver drops its state if
// the printer was the device that disconnected.
void catprinter_on_disconnect(uint16_t conn_handle);

// Call on BLE_GAP_EVENT_NOTIFY_RX to feed the printer's status
// notifications (paper present/out) into the driver.
void catprinter_on_notify(uint16_t conn_handle, uint16_t attr_handle,
                          struct os_mbuf *om);

// True once connected AND subscribed, i.e. ready to accept a print job.
bool catprinter_is_ready(void);

// True while a print job's BLE transfer is in flight in the
// background task. catprinter_print_rgb565() refuses to start a
// second job while one is outstanding.
bool catprinter_is_busy(void);

typedef enum {
  CATPRINTER_PAPER_UNKNOWN,
  CATPRINTER_PAPER_PRESENT,
  CATPRINTER_PAPER_OUT,
} catprinter_paper_status_t;

catprinter_paper_status_t catprinter_paper_status(void);

// Scale `pixels` (RGB565, width x height, standard row-major landscape
// layout as returned by camera_photo_snapshot()) down to the printer's
// fixed 384-dot width preserving aspect ratio, Floyd-Steinberg dither
// it to 1-bit, and hand it off to a background task that streams it
// to the printer. Returns once the (synchronous, CPU-only) dithering
// step is done and the background task has been created; does not
// block for the BLE transfer itself.
esp_err_t catprinter_print_rgb565(const uint16_t *pixels, uint32_t width,
                                  uint32_t height);
