#pragma once

#include <stdbool.h>
#include "esp_err.h"

// The WiFi/Bluetooth stack, wanted only by the BLE thermal printer.
//
// Brought up exactly once, at boot, and never torn down. There is
// deliberately no radio_stop(): the config toggle that turns this on
// and off restarts the app instead, which means the NimBLE and
// esp-hosted teardown paths -- the fussy ones -- never have to exist.
//
// Everything here is a no-op unless g_cfg.radio_enabled. Off means
// genuinely absent: no esp-hosted transport to the C6, no NimBLE host
// task, and none of the internal SRAM either of those would take out
// of the same 768 KB the camera pipeline works in.
//
// Flash is the exception. The code is linked in either way -- and
// esp-hosted-tanmatsu is linked whole-archive, so it is all of it --
// so the partition cost is paid whether or not the radio ever runs.

// Bring the stack up. Blocks for as long as the esp-hosted handshake
// with the C6 takes. Returns the first hard failure; a partial bring-up
// is torn back down only in the sense that radio_is_up() stays false
// and nothing else will touch it.
esp_err_t radio_start(bool scan_on_sync);

// True once the NimBLE host has synced and is usable.
bool radio_is_up(void);

// Free internal SRAM and PSRAM, in bytes. Used by the boot-time cost
// logging in app_main -- the app has never had any heap instrumentation
// and phase 3 needs to know what the stack actually costs.
void radio_log_heap(const char *stage);
