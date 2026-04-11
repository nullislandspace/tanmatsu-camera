#pragma once

// Video recorder — H.264 (hardware) + MP3 (Shine, software) into AVI.
//
// The recorder runs on top of the live preview pipeline: no sensor
// format switch, no stream pause. Each time we want a video frame we
// call camera_video_snapshot(), which PPA-scales the 1920x1080 RGB565
// source down to 480x270 YUV420 packed (the "O_UYY_E_VYY" format the
// ESP32-P4 hardware encoder accepts) inside a 480x272 macroblock-
// aligned buffer.
//
// The audio path runs in its own FreeRTOS task pinned to core 1 so
// that a future I²S microphone capture has deterministic timing. For
// now the audio source is a 440 Hz square wave generator used as a
// placeholder — it validates the full "audio in → Shine MP3 encode →
// AVI 01wb chunks" pipeline without hardware dependencies. Replacing
// the generator with a real I²S RX ring buffer is the only change
// the audio path will need when the mic lands.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Start recording. Allocates the H.264 encoder, the Shine MP3 encoder,
// the PPA YUV scratch buffer, opens /sd/DCIM/VID_YYYYMMDD_HHMMSS.avi
// via fastopen, writes the AVI headers, and spawns the video + audio
// recording tasks. The video task PPA-snaps one frame per
// PREVIEW_TARGET_FPS period and feeds it through the hardware H.264
// encoder. The audio task generates a 440 Hz square wave at the
// configured sample rate and feeds it through Shine, writing 01wb
// chunks as MP3 frames come out.
//
// `out_path` receives the absolute path of the file that was opened
// so the caller can show it to the user on the HUD. `dcim_dir` is
// the directory to create the file in (typically /sd/DCIM).
esp_err_t video_record_start(const char *dcim_dir, char *out_path, size_t out_path_sz);

// Stop the running recording: signal both tasks to flush + exit,
// wait for them, write the final idx1 and patch the AVI header
// sizes, and fastclose() the file.
esp_err_t video_record_stop(void);

// True between a successful record_start and record_stop.
bool video_is_recording(void);

// Elapsed recording time in milliseconds since the last successful
// record_start. Returns 0 if not recording. Used by the HUD to show
// "REC 00:12" live.
uint32_t video_recording_duration_ms(void);

#ifdef __cplusplus
}
#endif
