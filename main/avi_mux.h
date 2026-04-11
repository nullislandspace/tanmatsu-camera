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

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // File handle opened via fastopen("wb"). The muxer owns it for
    // the duration of the recording and fastclose()s it in
    // avi_mux_close.
    FILE *f;

    // Stream parameters captured at open() time and echoed into the
    // AVI header. None of these change during a recording.
    uint32_t video_width;
    uint32_t video_height;
    uint32_t video_fps;
    uint32_t audio_sample_rate;
    uint16_t audio_channels;
    uint16_t audio_bits_per_sample;   // for the WAVEFORMATEX header only; MP3 uses 0 here conventionally but 16 is accepted
    uint32_t audio_format_tag;        // WAVEFORMATEX wFormatTag (0x0055 = MP3)
    uint32_t audio_avg_bytes_per_sec; // CBR bitrate/8

    // File offsets captured during write so we can patch sizes at close.
    size_t   riff_size_pos;   // offset of RIFF size field
    size_t   movi_list_start; // offset of the "LIST...movi" chunk header
    size_t   movi_data_start; // offset of the "movi" FourCC (first byte of movi contents)
    size_t   avih_total_frames_pos;
    size_t   avih_max_bytes_per_sec_pos;
    size_t   vids_length_pos;    // strh.dwLength (video stream, frame count)
    size_t   auds_length_pos;    // strh.dwLength (audio stream, sample count in AVI "samples" = MP3 frames)

    // Running counters.
    uint32_t video_frames_written;
    uint32_t audio_samples_written;
    uint32_t video_bytes_written;   // raw payload bytes, for max_bytes_per_sec estimate
    uint32_t audio_bytes_written;

    // idx1 entries — grown dynamically, flushed to disk at close time.
    // Each entry is one AVI 16-byte idx1 record (FourCC, flags, offset, size).
    void    *idx_entries;      // uint8_t *, 16 bytes per entry
    uint32_t idx_count;
    uint32_t idx_capacity;

    bool     is_open;
} avi_mux_t;

// Open a new AVI file for writing. Writes the RIFF + hdrl headers
// and leaves the file positioned at the start of the movi chunk
// contents, ready for chunk writes. The `audio_avg_bytes_per_sec`
// parameter determines the declared dwAvgBytesPerSec in the audio
// strh/strf — for 64 kbps MP3 pass 8000.
esp_err_t avi_mux_open(avi_mux_t *mux, const char *path,
                       uint32_t video_width, uint32_t video_height,
                       uint32_t video_fps,
                       uint32_t audio_sample_rate,
                       uint16_t audio_channels,
                       uint32_t audio_avg_bytes_per_sec);

// Write one compressed video frame (00dc). The caller must know
// whether this was an IDR / keyframe and pass `keyframe=true` so the
// idx1 entry gets the AVIIF_KEYFRAME flag set — without that the
// videoplayer's seek support skips straight past non-keyframes and
// lands nowhere. For plain sequential playback keyframe flags are
// optional but still a good idea.
esp_err_t avi_mux_write_video(avi_mux_t *mux, const void *data, size_t size, bool keyframe);

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
