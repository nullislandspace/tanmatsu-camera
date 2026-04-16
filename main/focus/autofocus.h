#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/isp_types.h"

// Hardware-assisted autofocus.
//
// The ESP32-P4 ISP has a dedicated AF statistics block: it computes
// per-window sharpness ("definition") + luminance every frame, in
// hardware, with zero extra memory bandwidth. This module owns one
// AF controller, captures stats via an ISR callback, and runs a small
// hill-climb that drives the active focus driver
// (focus_set_position()) toward the sharpness peak.
//
// Lifecycle is owned by camera_pipeline: autofocus_init() is called
// after the ISP processor is enabled, autofocus_shutdown() before it
// is destroyed. Both PHOTO and VIDEO modes recreate the ISP via
// camera_preview_start/stop, so AF naturally restarts on mode switch.

esp_err_t autofocus_init(isp_proc_handle_t isp,
                         uint16_t input_w, uint16_t input_h);
void      autofocus_shutdown(void);

// Runtime gates. Cheap, callable from main task.
void autofocus_set_enabled        (bool enabled);
void autofocus_set_manual_override(bool active);

// Called once per main-loop frame. If AF is enabled, not overridden,
// and a fresh stats sample has arrived since the last call, the state
// machine may move the lens via focus_set_position() and update
// *focus_pos to match. No-op otherwise.
void autofocus_tick(uint16_t *focus_pos);

typedef enum {
    AF_HUD_OFF,       // disabled
    AF_HUD_SEARCH,    // sweeping coarse / fine
    AF_HUD_LOCK,      // converged, holding
    AF_HUD_OVERRIDE,  // user holding UP/DN
} af_hud_state_t;

af_hud_state_t autofocus_hud_state    (void);
int            autofocus_hud_sharpness(void);  // last definition[0] sample
