#pragma once

#include "config.h"

// I2S MEMS microphone driver. Currently knows two sensor types,
// INMP441 and ICS43434, selected at start time. Decoding is gated
// on the type so each mic can have its own slot/format quirks
// while sharing the I2S setup, ring buffer, gain, and HUD plumbing.
//
// Wired to a *dedicated* I2S controller (I2S_NUM_1) so it can never
// conflict with the ES8156 speaker codec path, which uses I2S_NUM_0
// in the BSP. Keeps audio playback support possible later without
// retrofitting a second controller.
//
// Pinout on the Tanmatsu internal add-on port (see
// microphone_inmp441.md for the full rationale). Two mics share the
// bus; only the L/R strap differs:
//   SCK → pin 19 (E8,  GPIO 54)   — shared
//   WS  → pin 20 (E9,  GPIO 49)   — shared
//   SD  → pin 21 (E10, GPIO 53)   — shared, each mic tri-states
//                                    during the other's half-frame
//   L/R → GND on the LEFT mic, +3.3V on the RIGHT mic
//   VDD → +3.3V, GND → GND
//
// Internally the I2S controller runs at 44.1 kHz in STEREO Philips
// mode with 32-bit slots (BCLK = 2.8224 MHz, inside INMP441 spec).
// A background reader task drains DMA continuously, recovers the
// signed 24-bit sample from each left-justified slot, averages the
// LEFT and RIGHT mics into one mono channel, applies digital gain
// with saturation, runs a 60 Hz DC-blocking high-pass and a 2-stage
// IIR low-pass, computes a rolling peak for the HUD meter, then
// keeps every other frame to halve the
// rate to 22.05 kHz mono — exactly the rate Shine encodes and the
// AVI file advertises, so no fractional resampling is needed.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

// Allocate the I2S RX channel, spawn the reader task, and begin
// continuous capture for the given mic type. Idempotent — a second
// call while already running returns ESP_OK without doing anything
// (the previously selected type stays in effect; stop and start to
// change it). Passing MIC_TYPE_NONE returns ESP_ERR_INVALID_ARG.
esp_err_t microphone_start(mic_type_t type);

// Signal the reader task to exit, wait for it, and release the I2S
// channel. Safe to call even when not running.
void microphone_stop(void);

// True between a successful start and stop.
bool microphone_is_running(void);

// Peak absolute sample in the most recent DMA block, scaled to
// [0..32767]. Updated roughly every 32 ms. Returns 0 while stopped.
// Safe to call from any task — a single 16-bit read is atomic.
uint16_t microphone_peak_level(void);

// Discard any samples currently buffered for the recorder. Called at
// video_record_start so the first AVI audio chunk aligns with the
// recording wall-clock start, not with stale pre-record data.
void microphone_capture_reset(void);

// Set the digital gain *step* applied during slot extraction. Input
// is clamped to [1, 8]. This is a step on a ~3 dB ladder, not a raw
// multiplier — use microphone_gain_multiplier() to get the factor it
// corresponds to. Safe to call at any time, even while the reader
// task is running: the read side is a single 32-bit load of a
// volatile int, so no locking is required.
void microphone_set_gain(int gain);

// Current digital gain step ([1, 8]).
int microphone_get_gain(void);

// The actual multiplier a gain step maps to (1× at step 1 up to 100×
// at step 8, ≈5.7 dB apart). Pure function of its argument — safe to
// call whether or not the mic is running. For the HUD readout.
int microphone_gain_multiplier(int gain);

// Pull up to `n_samples` int16 mono samples (at 16 kHz) out of the
// internal ring buffer. Blocks up to `timeout` for the first sample;
// returns however many samples were actually received. The caller is
// expected to zero-fill any shortfall itself (the recorder does
// exactly this when the mic is running but a read underruns).
size_t microphone_read_pcm(int16_t *dst, size_t n_samples, TickType_t timeout);

// Debug helper: capture `seconds` worth of raw I2S samples (the
// stereo-interleaved int32 stream BEFORE gain, peak tracking, LEFT
// extraction and the 2:1 decimator) into a PSRAM buffer, then write
// them to `path` as a standard 32-bit PCM stereo WAV claiming
// MIC_I2S_RATE_NOM (44.1 kHz). Blocks until the capture is complete.
// Requires microphone_is_running() == true.
//
// The file opens directly in any audio tool (Audacity, ffmpeg,
// numpy's scipy.io.wavfile). With two mics wired, both channels
// carry real audio (LEFT = the L/R=GND mic, RIGHT = the L/R=+3.3V
// one). With only one mic wired, the unused channel is whatever the
// floating DIN pin looked like during that slot (usually all
// 0xFFFFFFFF) — a quick way to tell a dead mic from a live one.
esp_err_t microphone_debug_raw_dump(const char *path, int seconds);

#ifdef __cplusplus
}
#endif
