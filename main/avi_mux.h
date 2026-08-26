#pragma once

// Small streaming AVI muxer for the camera recorder.
//
// Produces a RIFF/AVI file with one video stream and one audio
// stream, interleaved as 00dc (video) and 01wb (audio) chunks inside
// a single `movi` list. An `idx1` legacy index is appended at close
// time and the RIFF / movi / frame-count sizes patched back into
// the header. The resulting layout is what the Zōtorōpu videoplayer
// (avi_parser.c) expects.
//
// This is a minimal muxer: single movi list (files ≤ 2 GB), fixed
// stream layout (0=vids, 1=auds), no extended OpenDML headers. That
// is plenty for on-device camera captures — a ~10 minute clip at
// 400 kbps video + 64 kbps audio is under 35 MB.
//
// TIMING MODEL. AVI has no per-frame timestamps anywhere — not in
// idx1 (16 bytes of ckid/flags/offset/length) and not in OpenDML's
// AVISTDINDEX (8 bytes of offset/size). Playback time for frame N is
// purely N × dwScale / dwRate, so the container is structurally
// constant-frame-rate. The capture loop, however, is not: H.264
// encode time depends on scene content, so it overruns its period on
// busy frames and the real rate wanders within a single recording.
//
// The fix is to lay the file out on a fine fixed grid — see
// AVI_TIMELINE_FPS — and place each captured frame in the grid slot
// nearest its capture timestamp, filling the slots in between with
// ZERO-LENGTH 00dc chunks. A zero-length video chunk is AVI's
// long-standing "nothing new, keep showing the previous frame"
// convention; it costs 24 bytes (8 chunk header + 16 idx1 entry) and
// no decode work at all, since there are no bytes to decode. The
// result is variable frame timing expressed inside a
// constant-frame-rate container, accurate to half a grid period.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Frame rate of the AVI timeline. Every recording is written on this
// grid regardless of what the encoder actually sustains, so the
// worst-case timing error for any frame is half a grid period — 8.3 ms
// at 60. Raising it costs 24 bytes per unused slot (about 11 KB for a
// ten-second clip) and nothing in decode time; lowering it coarsens
// the timing. 60 is a round multiple of the rates we might ever
// capture at, which keeps a genuinely constant capture rate landing on
// exact slots instead of dithering between two of them.
#define AVI_TIMELINE_FPS 60u

typedef struct {
    // File handle opened via fastopen("wb"). The muxer owns it for
    // the duration of the recording and fastclose()s it in
    // avi_mux_close.
    FILE *f;

    // Stream parameters captured at open() time and echoed into the
    // AVI header. None of these change during a recording.
    uint32_t video_width;
    uint32_t video_height;
    uint32_t video_fps;         // rate the capture loop AIMS for; informational only
    uint32_t timeline_fps;      // AVI_TIMELINE_FPS — the grid the file is actually written on
    uint32_t audio_sample_rate;
    uint16_t audio_channels;
    uint16_t audio_bits_per_sample;   // for the WAVEFORMATEX header only; MP3 uses 0 here conventionally but 16 is accepted
    uint32_t audio_format_tag;        // WAVEFORMATEX wFormatTag (0x0055 = MP3)
    uint32_t audio_avg_bytes_per_sec; // CBR bitrate/8
    uint16_t audio_samples_per_frame; // MP3 samples per frame (1152 for MPEG-1, 576 for MPEG-2)

    // File offsets captured during write so we can patch sizes at close.
    size_t   riff_size_pos;   // offset of RIFF size field
    size_t   movi_list_start; // offset of the "LIST...movi" chunk header
    size_t   movi_data_start; // offset of the "movi" FourCC (first byte of movi contents)
    size_t   avih_total_frames_pos;
    size_t   avih_max_bytes_per_sec_pos;
    size_t   vids_length_pos;    // strh.dwLength (video stream, frame count)
    size_t   auds_length_pos;    // strh.dwLength (audio stream, sample count in AVI "samples" = MP3 frames)

    // Running counters. video_frames_written counts GRID SLOTS, real
    // and null alike — that is what dwTotalFrames and the video
    // strh.dwLength must report, since the timeline is measured in
    // slots. video_real_frames_written counts the ones that carry
    // actual encoded data.
    uint32_t video_frames_written;
    uint32_t video_real_frames_written;
    uint32_t video_null_frames_written;
    uint32_t audio_samples_written;
    uint32_t audio_frames_written;  // MP3-chunk count — goes into the audio strh dwLength
    uint32_t video_bytes_written;   // raw payload bytes, for max_bytes_per_sec estimate
    uint32_t audio_bytes_written;

    // Capture-time span of the video frames actually written, kept for
    // the summary log line so the sustained encoder rate stays visible
    // even though it no longer drives the header.
    int64_t  video_first_ts_us;
    int64_t  video_last_ts_us;

    // idx1 entries — grown dynamically, flushed to disk at close time.
    // Each entry is one AVI 16-byte idx1 record (FourCC, flags, offset, size).
    void    *idx_entries;      // uint8_t *, 16 bytes per entry
    uint32_t idx_count;
    uint32_t idx_capacity;

    bool     is_open;
} avi_mux_t;

// Open a new AVI file for writing. Writes the RIFF + hdrl headers
// and leaves the file positioned at the start of the movi chunk
// contents, ready for chunk writes. `audio_avg_bytes_per_sec` is
// the WAVEFORMATEX nAvgBytesPerSec (CBR bitrate / 8; 16000 for
// 128 kbps). `audio_samples_per_frame` is how many PCM samples one
// encoded MP3 frame contains (1152 for MPEG-1, 576 for MPEG-2) —
// the muxer uses it for the audio-stream time base so the stream
// length is expressed in frames, matching what ffmpeg writes and
// what the Zōtorōpu videoplayer expects to parse.
esp_err_t avi_mux_open(avi_mux_t *mux, const char *path,
                       uint32_t video_width, uint32_t video_height,
                       uint32_t video_fps,
                       uint32_t audio_sample_rate,
                       uint16_t audio_channels,
                       uint32_t audio_avg_bytes_per_sec,
                       uint16_t audio_samples_per_frame);

// Write one compressed video frame (00dc). The caller must know
// whether this was an IDR / keyframe and pass `keyframe=true` so the
// idx1 entry gets the AVIIF_KEYFRAME flag set — without that the
// videoplayer's seek support skips straight past non-keyframes and
// lands nowhere. For plain sequential playback keyframe flags are
// optional but still a good idea.
//
// ts_us is the frame's capture timestamp relative to the start of
// recording. It does not go into the file directly — AVI cannot carry
// per-frame timestamps — but it selects which AVI_TIMELINE_FPS grid
// slot the frame lands in, and any slots skipped over are filled with
// zero-length chunks. It must therefore be the time the frame was
// GRABBED, not the time it reached the writer queue, or queue latency
// shows up as jitter in the played-back timing.
esp_err_t avi_mux_write_video(avi_mux_t *mux, const void *data, size_t size, bool keyframe,
                              int64_t ts_us);

// Write one compressed audio chunk (01wb). For MP3 this is typically
// one or more full MP3 frames concatenated — the decoder side of
// Zōtorōpu feeds whatever is in the chunk straight into
// esp_mp3_dec which re-parses frame boundaries from the sync words,
// so chunk boundaries need not line up with MP3 frame boundaries.
// `samples_covered` is the number of PCM samples per channel this
// chunk represents — used to advance the audio stream's dwLength
// counter for the timeline.
esp_err_t avi_mux_write_audio(avi_mux_t *mux, const void *data, size_t size,
                              uint32_t samples_covered);

// Finalise the file: write the idx1 chunk, seek back to the RIFF /
// hdrl / strh fields and patch in the running counters, then close
// the file.
esp_err_t avi_mux_close(avi_mux_t *mux);

// Best-effort cleanup without finalising — for use on error paths
// when we cannot or do not want to patch the header back in. The
// resulting file is not playable.
void avi_mux_abort(avi_mux_t *mux);

#ifdef __cplusplus
}
#endif
