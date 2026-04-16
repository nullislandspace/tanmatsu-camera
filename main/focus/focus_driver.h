#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

// Focus driver factory. The application talks only to focus_set_position()
// and the active-driver helpers below. Each chip lives behind a vtable
// (focus_driver_t) and a registry entry, so adding a new VCM is purely a
// matter of dropping in a new file and appending to the registry.
//
// Exactly one driver is "active" at any time. Initially none — the boot
// code calls focus_select(name) once it knows what the user picked in
// the config file, and the config menu can switch drivers at runtime.

typedef struct {
    const char *name;          // config key, e.g. "dw9714p", "simulator"
    const char *display_name;  // human-readable, shown in the menu/HUD

    // Probe the device. Returns ESP_OK if usable, ESP_ERR_NOT_FOUND if
    // absent, other esp_err_t on bus errors. Must be safe to call
    // multiple times.
    esp_err_t (*probe)(void);

    // Optional one-shot setup after probe. NULL means "nothing to do".
    esp_err_t (*init)(void);

    // Move the lens. Position range is [pos_min, pos_max]; the driver
    // clamps internally so callers don't need to.
    esp_err_t (*set_position)(uint16_t pos);

    uint16_t pos_min;
    uint16_t pos_max;
    uint16_t pos_default;  // safe mid-range start position
} focus_driver_t;

// NULL-terminated array of every driver compiled in. The order here is
// the cycle order shown in the config menu.
extern const focus_driver_t *const focus_driver_registry[];

// Lookup helpers — used by the menu cycler and config loader.
const focus_driver_t *focus_driver_by_name (const char *name);
const focus_driver_t *focus_driver_by_index(int idx);
int                   focus_driver_count   (void);

// Activate a driver. NULL or "" deactivates (no driver). On a non-empty
// name: looks up the driver, runs probe + init + set_position(default).
// Returns ESP_OK on success and leaves the driver as the active one;
// returns the underlying error and leaves *no* driver active on
// failure.
esp_err_t focus_select(const char *name);

// Currently active driver, or NULL if none.
const focus_driver_t *focus_active(void);

// Driver name for HUD/log. Returns "off" when no driver is active.
const char *focus_active_name(void);

// Move the lens via the active driver. Returns ESP_ERR_INVALID_STATE if
// no driver is active. Callers should normally gate on focus_active()
// before calling.
esp_err_t focus_set_position(uint16_t pos);
