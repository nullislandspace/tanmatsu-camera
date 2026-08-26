#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/isp_types.h"

#include "camera_sensor.h"

// Software auto-exposure for sensors that have none of their own.
//
// The ESP32-P4 ISP has a dedicated AE statistics block: it reports a
// 5x5 grid of 8-bit mean luminances over a sampling window, every
// frame, in hardware, with no extra memory bandwidth. This module owns
// one AE controller, captures the grid via an ISR callback, meters it
// centre-weighted, and servos the sensor's exposure + analog gain
// toward a target luminance via camera_sensor_set_exposure_eg().
//
// Who needs this:
//
//   - OV9281 — a machine-vision part with no on-chip AE at all. Without
//     this it sits at one fixed brightness forever.
//   - Nobody else. OV5640/OV5645/OV5647 run their own AE loop on-chip
//     and are left alone.
//
// That split is enforced by the pipeline rather than by a sensor list:
// the AE statistics block only produces meaningful numbers when the ISP
// is actually demosaicing, and the RGB565 sensors run it in bypass, so
// autoexposure_init() is simply never called for them.
//
// The loop corrects once per statistics frame. Exposure writes take one
// to two frames to appear in the statistics, so the loop models that
// delay explicitly: it remembers which budget each in-flight frame was
// shot under and computes every correction against that, which means a
// correction already sent but not yet visible is never counted twice.
// Without the model, running at frame rate would make it chase its own
// output; with it, the only thing left to choose is damping.
//
// Damping is done in the log domain — each correction closes half the
// remaining error in stops (three quarters when the scene has changed
// outright). Steps are therefore proportional to the error the eye
// actually perceives: a lighting change ramps across in a few frames,
// and a small drift barely moves. A narrow convergence band with
// hysteresis stops it dithering on sensor noise.
//
// Lifecycle mirrors autofocus: camera_pipeline calls
// autoexposure_init() after the ISP processor is enabled and
// autoexposure_shutdown() before it is destroyed, so a PHOTO<->VIDEO
// switch tears it down and brings it back automatically. The enabled
// flag survives that, so the user's intent is not silently dropped.

esp_err_t autoexposure_init(isp_proc_handle_t isp,
                            uint16_t input_w, uint16_t input_h);
void      autoexposure_shutdown(void);

// True once the AE statistics block is up — i.e. this pipeline is a
// RAW/demosaic one and a software AE loop is actually possible. Used
// by the UI to decide whether "AUTO" is an offer it can make.
bool autoexposure_available(void);

// Runtime gate. Cheap, callable from the main task, and callable
// before init: the flag is remembered and picked up when the ISP comes
// up (the boot path decides the mode before the pipeline exists).
void autoexposure_set_enabled(bool enabled);
bool autoexposure_is_enabled(void);

// Called once per main-loop frame. Rate-limits itself internally, so
// calling it every frame is correct and cheap. No-op when disabled, or
// when the statistics block is not running.
void autoexposure_tick(camera_sensor_t *sensor);

// Re-program the sensor with the loop's current light budget. Called
// after a set_format, which replays the driver's init table and wipes
// the exposure registers.
void autoexposure_reapply(camera_sensor_t *sensor);

// The loop's current light budget, in microseconds at unity gain — the
// value a manual ladder should seed itself from when the user takes
// over. 0 if the loop has not measured anything yet.
uint32_t autoexposure_current_eg_us(void);

// Registers the loop last wrote, for the HUD to display. Zeroed if the
// loop has not written anything yet.
void autoexposure_last_exposure(camera_exposure_t *out);

typedef enum {
    AE_HUD_OFF,      // disabled, or no statistics block on this pipeline
    AE_HUD_HUNTING,  // measured luminance outside the deadband, correcting
    AE_HUD_LOCKED,   // inside the deadband, holding
    AE_HUD_LIMIT,    // outside the deadband but the sensor has no range left
} ae_hud_state_t;

ae_hud_state_t autoexposure_hud_state(void);
// Last metered luminance, 0..255. For the HUD and for retuning
// AE_TARGET_LUMA against real scenes.
int autoexposure_hud_luma(void);
