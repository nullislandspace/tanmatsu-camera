#pragma once

// INMP441 I2S MEMS microphone driver.
//
// Wired to a *dedicated* I2S controller (I2S_NUM_1) so it can never
// conflict with the ES8156 speaker codec path, which uses I2S_NUM_0
// in the BSP. Keeps audio playback support possible later without
// retrofitting a second controller.
//
// Pinout on the Tanmatsu internal add-on port (see
// microphone_inmp441.md for the full rationale):
//   SCK → pin 19 (E8,  GPIO 54)
//   WS  → pin 20 (E9,  GPIO 49)
//   SD  → pin 21 (E10, GPIO 53)
//   L/R → GND (mic drives the LEFT half-frame)
//   VDD → +3.3V, GND → GND
//
// Internally the I2S controller runs at 44.1 kHz in STEREO Philips
// mode with 32-bit slots (BCLK = 2.8224 MHz, inside INMP441 spec).
// A background reader task drains DMA continuously, picks the LEFT
// slot out of each stereo frame (the INMP441 drives only LEFT
// because its L/R strap is tied to GND), applies a 2-stage IIR
// low-pass + digital gain with saturation, computes a rolling peak
// for the HUD meter, then keeps every other LEFT sample to halve
// the rate to 22.05 kHz mono — exactly the rate Shine encodes and
// the AVI file advertises, so no fractional resampling is needed.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

// Allocate the I2S RX channel, spawn the reader task, and begin
// continuous capture. Idempotent — a second call while already
// running returns ESP_OK without doing anything.
esp_err_t microphone_start(void);

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

// Set the fixed digital gain multiplier applied after the 16-bit slot
// extraction. Input is clamped to [1, 8]. Safe to call at any time,
// even while the reader task is running — the read side is a single
// 32-bit load of a volatile int, so no locking is required.
void microphone_set_gain(int gain);

// Current digital gain multiplier ([1, 8]).
int microphone_get_gain(void);

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
// numpy's scipy.io.wavfile). The LEFT channel carries the actual
// mic audio; the RIGHT channel is whatever the floating DIN pin
// looked like during the unused slot (usually all 0xFFFFFFFF).
esp_err_t microphone_debug_raw_dump(const char *path, int seconds);

#ifdef __cplusplus
}
#endif
