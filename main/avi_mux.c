#include "avi_mux.h"

#include <errno.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "fastopen.h"

static const char *TAG = "avi_mux";

// FourCC constants — stored little-endian in the file so the most
// compact way to spell them in source is ('D'<<24)|('C'<<16)|... but
// readable macros are nicer.
#define FOURCC(a, b, c, d) \
    ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))

#define FCC_RIFF  FOURCC('R','I','F','F')
#define FCC_AVI   FOURCC('A','V','I',' ')
#define FCC_LIST  FOURCC('L','I','S','T')
#define FCC_HDRL  FOURCC('h','d','r','l')
#define FCC_AVIH  FOURCC('a','v','i','h')
#define FCC_STRL  FOURCC('s','t','r','l')
#define FCC_STRH  FOURCC('s','t','r','h')
#define FCC_STRF  FOURCC('s','t','r','f')
#define FCC_VIDS  FOURCC('v','i','d','s')
#define FCC_AUDS  FOURCC('a','u','d','s')
#define FCC_MOVI  FOURCC('m','o','v','i')
#define FCC_IDX1  FOURCC('i','d','x','1')
#define FCC_H264  FOURCC('H','2','6','4')
#define FCC_00DC  FOURCC('0','0','d','c')
#define FCC_01WB  FOURCC('0','1','w','b')

// AVIIF_KEYFRAME flag for idx1 entries.
#define AVIIF_KEYFRAME 0x00000010

// Simple little-endian writers — Xtensa/RISC-V are both native
// little-endian so these could be bare fwrites, but keeping the
// conversion explicit makes the format self-documenting.
static int wr_u32(FILE *f, uint32_t v) {
    uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    return fwrite(b, 1, 4, f) == 4 ? 0 : -1;
}
static int wr_u16(FILE *f, uint16_t v) {
    uint8_t b[2] = { (uint8_t)v, (uint8_t)(v >> 8) };
    return fwrite(b, 1, 2, f) == 2 ? 0 : -1;
}
static int wr_fcc(FILE *f, uint32_t fcc) {
    return wr_u32(f, fcc);
}
static int wr_bytes(FILE *f, const void *p, size_t n) {
    return fwrite(p, 1, n, f) == n ? 0 : -1;
}
static size_t cur_pos(FILE *f) {
    long p = ftell(f);
    return p < 0 ? 0 : (size_t)p;
}

// Grow the idx1 entry list. Each entry is 16 bytes of data stored
// as-is, written in a single fwrite at close time for speed.
static esp_err_t idx_reserve(avi_mux_t *mux, uint32_t want_count) {
    if (want_count <= mux->idx_capacity) return ESP_OK;
    uint32_t new_cap = mux->idx_capacity ? mux->idx_capacity * 2u : 512u;
    if (new_cap < want_count) new_cap = want_count + 256u;
    size_t new_bytes = (size_t)new_cap * 16u;
    void *p = realloc(mux->idx_entries, new_bytes);
    if (!p) {
        // idx1 allocation on PSRAM can fail under pressure — try
        // heap_caps_realloc targeting PSRAM explicitly.
        p = heap_caps_realloc(mux->idx_entries, new_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (!p) {
        ESP_LOGE(TAG, "idx1 realloc failed (%u entries)", (unsigned)new_cap);
        return ESP_ERR_NO_MEM;
    }
    mux->idx_entries = p;
    mux->idx_capacity = new_cap;
    return ESP_OK;
}

// Append one 16-byte idx1 entry. Offsets here are "movi-relative",
// i.e. relative to the start of the movi LIST data (including the
// "movi" FourCC — that's the AVI 1.0 idx1 convention).
static esp_err_t idx_append(avi_mux_t *mux, uint32_t fourcc, uint32_t flags,
                            uint32_t rel_offset, uint32_t size) {
    esp_err_t err = idx_reserve(mux, mux->idx_count + 1u);
    if (err != ESP_OK) return err;
    uint8_t *slot = (uint8_t *)mux->idx_entries + (size_t)mux->idx_count * 16u;
    // All fields little-endian u32.
    slot[0] = (uint8_t)fourcc; slot[1] = (uint8_t)(fourcc >> 8);
    slot[2] = (uint8_t)(fourcc >> 16); slot[3] = (uint8_t)(fourcc >> 24);
    slot[4] = (uint8_t)flags; slot[5] = (uint8_t)(flags >> 8);
    slot[6] = (uint8_t)(flags >> 16); slot[7] = (uint8_t)(flags >> 24);
    slot[8] = (uint8_t)rel_offset; slot[9] = (uint8_t)(rel_offset >> 8);
    slot[10] = (uint8_t)(rel_offset >> 16); slot[11] = (uint8_t)(rel_offset >> 24);
    slot[12] = (uint8_t)size; slot[13] = (uint8_t)(size >> 8);
    slot[14] = (uint8_t)(size >> 16); slot[15] = (uint8_t)(size >> 24);
    mux->idx_count++;
    return ESP_OK;
}

// Write the complete AVI header (RIFF + LIST hdrl + avih + two strl
// lists) up to the start of the movi LIST data. Records every file
// offset we need to patch later. Leaves the file positioned right
// after the "movi" FourCC, ready for chunk data.
static esp_err_t write_headers(avi_mux_t *mux) {
    FILE *f = mux->f;

    // RIFF <size> AVI
    if (wr_fcc(f, FCC_RIFF)) return ESP_FAIL;
    mux->riff_size_pos = cur_pos(f);
    if (wr_u32(f, 0)) return ESP_FAIL;     // patched at close
    if (wr_fcc(f, FCC_AVI)) return ESP_FAIL;

    // LIST <size> hdrl
    if (wr_fcc(f, FCC_LIST)) return ESP_FAIL;
    size_t hdrl_size_pos = cur_pos(f);
    if (wr_u32(f, 0)) return ESP_FAIL;     // patched below once we know the size
    size_t hdrl_data_start = cur_pos(f);
    if (wr_fcc(f, FCC_HDRL)) return ESP_FAIL;

    // avih <56> <MainAVIHeader>
    if (wr_fcc(f, FCC_AVIH)) return ESP_FAIL;
    if (wr_u32(f, 56)) return ESP_FAIL;
    uint32_t us_per_frame = mux->video_fps ? 1000000u / mux->video_fps : 66666u;
    if (wr_u32(f, us_per_frame)) return ESP_FAIL;                // dwMicroSecPerFrame
    mux->avih_max_bytes_per_sec_pos = cur_pos(f);
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwMaxBytesPerSec (patched)
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwPaddingGranularity
    if (wr_u32(f, 0x10 | 0x20)) return ESP_FAIL;                  // dwFlags (AVIF_HASINDEX | AVIF_ISINTERLEAVED)
    mux->avih_total_frames_pos = cur_pos(f);
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwTotalFrames (patched)
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwInitialFrames
    if (wr_u32(f, 2)) return ESP_FAIL;                            // dwStreams
    if (wr_u32(f, 0x10000)) return ESP_FAIL;                      // dwSuggestedBufferSize (~64 KB)
    if (wr_u32(f, mux->video_width)) return ESP_FAIL;             // dwWidth
    if (wr_u32(f, mux->video_height)) return ESP_FAIL;            // dwHeight
    for (int i = 0; i < 4; i++) if (wr_u32(f, 0)) return ESP_FAIL; // dwReserved[4]

    // LIST <size> strl (video stream)
    if (wr_fcc(f, FCC_LIST)) return ESP_FAIL;
    if (wr_u32(f, 4 + 8 + 56 + 8 + 40)) return ESP_FAIL;          // LIST payload = "strl" + strh chunk + strf chunk
    if (wr_fcc(f, FCC_STRL)) return ESP_FAIL;

    // strh video (56 bytes)
    if (wr_fcc(f, FCC_STRH)) return ESP_FAIL;
    if (wr_u32(f, 56)) return ESP_FAIL;
    if (wr_fcc(f, FCC_VIDS)) return ESP_FAIL;                     // fccType
    if (wr_fcc(f, FCC_H264)) return ESP_FAIL;                     // fccHandler
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwFlags
    if (wr_u16(f, 0)) return ESP_FAIL;                            // wPriority
    if (wr_u16(f, 0)) return ESP_FAIL;                            // wLanguage
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwInitialFrames
    if (wr_u32(f, 1)) return ESP_FAIL;                            // dwScale
    if (wr_u32(f, mux->video_fps)) return ESP_FAIL;               // dwRate
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwStart
    mux->vids_length_pos = cur_pos(f);
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwLength (patched)
    if (wr_u32(f, 0x10000)) return ESP_FAIL;                      // dwSuggestedBufferSize
    if (wr_u32(f, 0xFFFFFFFFu)) return ESP_FAIL;                  // dwQuality (-1 = unspecified)
    if (wr_u32(f, 0)) return ESP_FAIL;                            // dwSampleSize (video: 0 = VBR)
    // rcFrame left=0, top=0, right=width, bottom=height
    if (wr_u16(f, 0)) return ESP_FAIL;
    if (wr_u16(f, 0)) return ESP_FAIL;
    if (wr_u16(f, (uint16_t)mux->video_width)) return ESP_FAIL;
    if (wr_u16(f, (uint16_t)mux->video_height)) return ESP_FAIL;

    // strf video: BITMAPINFOHEADER (40 bytes)
    if (wr_fcc(f, FCC_STRF)) return ESP_FAIL;
    if (wr_u32(f, 40)) return ESP_FAIL;
    if (wr_u32(f, 40)) return ESP_FAIL;                            // biSize
    if (wr_u32(f, mux->video_width)) return ESP_FAIL;              // biWidth
    if (wr_u32(f, mux->video_height)) return ESP_FAIL;             // biHeight
    if (wr_u16(f, 1)) return ESP_FAIL;                             // biPlanes
    if (wr_u16(f, 24)) return ESP_FAIL;                            // biBitCount
    if (wr_fcc(f, FCC_H264)) return ESP_FAIL;                      // biCompression
    if (wr_u32(f, mux->video_width * mux->video_height * 3u)) return ESP_FAIL; // biSizeImage
    if (wr_u32(f, 0)) return ESP_FAIL;                             // biXPelsPerMeter
    if (wr_u32(f, 0)) return ESP_FAIL;                             // biYPelsPerMeter
    if (wr_u32(f, 0)) return ESP_FAIL;                             // biClrUsed
    if (wr_u32(f, 0)) return ESP_FAIL;                             // biClrImportant

    // LIST <size> strl (audio stream)
    if (wr_fcc(f, FCC_LIST)) return ESP_FAIL;
    // LIST payload = "strl" + strh chunk + strf chunk (strf payload 30 bytes for MPEGLAYER3)
    if (wr_u32(f, 4 + 8 + 56 + 8 + 30)) return ESP_FAIL;
    if (wr_fcc(f, FCC_STRL)) return ESP_FAIL;

    // strh audio (56 bytes).
    //
    // For MP3 in AVI we use the "MP3 frame" time base that ffmpeg
    // writes and that the Zōtorōpu videoplayer's parser expects:
    //   dwScale       = samples_per_frame (1152 for MPEG-1)
    //   dwRate        = sample_rate (44100)
    //   dwLength      = number of MP3 frames in the stream
    //   dwSampleSize  = 0   (variable — each chunk is one MP3 frame)
    //   nBlockAlign   = samples_per_frame
    //
    // Stream duration is dwLength × dwScale / dwRate seconds. The
    // "fake-PCM" alternative (dwSampleSize=1, dwRate=bytes/sec)
    // played ok in desktop tools but made the videoplayer lose
    // sync because its AVI parser treats the audio stream duration
    // as the total in the header, not as the per-chunk frame count
    // — see the A/V sync regression documented when this was
    // changed from that form.
    if (wr_fcc(f, FCC_STRH)) return ESP_FAIL;
    if (wr_u32(f, 56)) return ESP_FAIL;
    if (wr_fcc(f, FCC_AUDS)) return ESP_FAIL;                        // fccType
    if (wr_fcc(f, 0)) return ESP_FAIL;                               // fccHandler (0 for audio by convention)
    if (wr_u32(f, 0)) return ESP_FAIL;                               // dwFlags
    if (wr_u16(f, 0)) return ESP_FAIL;                               // wPriority
    if (wr_u16(f, 0)) return ESP_FAIL;                               // wLanguage
    if (wr_u32(f, 0)) return ESP_FAIL;                               // dwInitialFrames
    if (wr_u32(f, mux->audio_samples_per_frame)) return ESP_FAIL;    // dwScale = samples/frame
    if (wr_u32(f, mux->audio_sample_rate)) return ESP_FAIL;          // dwRate = sample rate
    if (wr_u32(f, 0)) return ESP_FAIL;                               // dwStart
    mux->auds_length_pos = cur_pos(f);
    if (wr_u32(f, 0)) return ESP_FAIL;                               // dwLength (patched: total MP3 frames at close)
    if (wr_u32(f, 0x4000)) return ESP_FAIL;                          // dwSuggestedBufferSize (~16 KB)
    if (wr_u32(f, 0xFFFFFFFFu)) return ESP_FAIL;                     // dwQuality
    if (wr_u32(f, 0)) return ESP_FAIL;                               // dwSampleSize = 0 (VBR/framed)
    if (wr_u16(f, 0)) return ESP_FAIL;
    if (wr_u16(f, 0)) return ESP_FAIL;
    if (wr_u16(f, 0)) return ESP_FAIL;
    if (wr_u16(f, 0)) return ESP_FAIL;

    // strf audio: MPEGLAYER3WAVEFORMAT (30 bytes = WAVEFORMATEX(18) + MPEGLAYER3(12))
    // Required for Windows compatibility and matches what ffmpeg writes.
    if (wr_fcc(f, FCC_STRF)) return ESP_FAIL;
    if (wr_u32(f, 30)) return ESP_FAIL;
    if (wr_u16(f, 0x0055)) return ESP_FAIL;                          // wFormatTag: WAVE_FORMAT_MPEGLAYER3
    if (wr_u16(f, mux->audio_channels)) return ESP_FAIL;             // nChannels
    if (wr_u32(f, mux->audio_sample_rate)) return ESP_FAIL;          // nSamplesPerSec
    if (wr_u32(f, mux->audio_avg_bytes_per_sec)) return ESP_FAIL;    // nAvgBytesPerSec
    if (wr_u16(f, mux->audio_samples_per_frame)) return ESP_FAIL;    // nBlockAlign = samples/frame (matches ffmpeg)
    if (wr_u16(f, 0)) return ESP_FAIL;                               // wBitsPerSample (0 for MP3)
    if (wr_u16(f, 12)) return ESP_FAIL;                              // cbSize (12 = MPEGLAYER3 extension)
    // MPEGLAYER3WAVEFORMAT extension
    if (wr_u16(f, 1)) return ESP_FAIL;                             // wID = MPEGLAYER3_ID_MPEG
    if (wr_u32(f, 2)) return ESP_FAIL;                             // fdwFlags = MPEGLAYER3_FLAG_PADDING_OFF
    if (wr_u16(f, mux->audio_samples_per_frame)) return ESP_FAIL;  // nBlockSize (samples per MPEG frame: 1152 Layer-III MPEG-I, 576 MPEG-II)
    if (wr_u16(f, 1)) return ESP_FAIL;                             // nFramesPerBlock
    if (wr_u16(f, 1393)) return ESP_FAIL;                          // nCodecDelay (Fraunhofer default for MP3)

    // Patch hdrl LIST size now that we know the full hdrl payload.
    size_t hdrl_end = cur_pos(f);
    uint32_t hdrl_list_size = (uint32_t)(hdrl_end - hdrl_data_start);
    if (fseek(f, (long)hdrl_size_pos, SEEK_SET)) return ESP_FAIL;
    if (wr_u32(f, hdrl_list_size)) return ESP_FAIL;
    if (fseek(f, (long)hdrl_end, SEEK_SET)) return ESP_FAIL;

    // LIST <size> movi
    if (wr_fcc(f, FCC_LIST)) return ESP_FAIL;
    mux->movi_list_start = cur_pos(f);           // points at the size DWORD
    if (wr_u32(f, 0)) return ESP_FAIL;            // movi LIST size (patched at close)
    mux->movi_data_start = cur_pos(f);           // points at the "movi" FourCC
    if (wr_fcc(f, FCC_MOVI)) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t avi_mux_open(avi_mux_t *mux, const char *path,
                       uint32_t video_width, uint32_t video_height,
                       uint32_t video_fps,
                       uint32_t audio_sample_rate,
                       uint16_t audio_channels,
                       uint32_t audio_avg_bytes_per_sec,
                       uint16_t audio_samples_per_frame) {
    if (!mux || !path) return ESP_ERR_INVALID_ARG;
    if (audio_samples_per_frame == 0) return ESP_ERR_INVALID_ARG;
    memset(mux, 0, sizeof(*mux));
    mux->video_width             = video_width;
    mux->video_height            = video_height;
    mux->video_fps               = video_fps;
    mux->audio_sample_rate       = audio_sample_rate;
    mux->audio_channels          = audio_channels;
    mux->audio_bits_per_sample   = 0;           // MP3 convention
    mux->audio_format_tag        = 0x0055;      // MPEGLAYER3
    mux->audio_avg_bytes_per_sec = audio_avg_bytes_per_sec;
    mux->audio_samples_per_frame = audio_samples_per_frame;

    mux->f = fastopen(path, "wb");
    if (!mux->f) {
        ESP_LOGE(TAG, "fastopen('%s') failed: %s", path, strerror(errno));
        return ESP_FAIL;
    }

    esp_err_t err = write_headers(mux);
    if (err != ESP_OK) {
        fastclose(mux->f);
        mux->f = NULL;
        return err;
    }
    mux->is_open = true;
    ESP_LOGI(TAG, "opened %s  video %" PRIu32 "x%" PRIu32 " @%" PRIu32 " fps  audio %" PRIu32 " Hz x%u",
             path, video_width, video_height, video_fps,
             audio_sample_rate, (unsigned)audio_channels);
    return ESP_OK;
}

// Common chunk-write helper. AVI chunks are 8-byte header (FourCC +
// u32 size) followed by `size` bytes of data, with a 1-byte pad if
// size is odd. Caller supplies the FourCC and is responsible for
// appending an idx1 entry if it wants the chunk to be seekable.
static esp_err_t write_chunk(avi_mux_t *mux, uint32_t fourcc, const void *data, size_t size,
                             size_t *out_data_offset) {
    FILE *f = mux->f;
    if (wr_fcc(f, fourcc)) return ESP_FAIL;
    if (wr_u32(f, (uint32_t)size)) return ESP_FAIL;
    size_t data_off = cur_pos(f);
    if (size > 0) {
        if (wr_bytes(f, data, size)) return ESP_FAIL;
    }
    if (size & 1u) {
        uint8_t pad = 0;
        if (wr_bytes(f, &pad, 1)) return ESP_FAIL;
    }
    if (out_data_offset) *out_data_offset = data_off;
    return ESP_OK;
}

esp_err_t avi_mux_write_video(avi_mux_t *mux, const void *data, size_t size, bool keyframe) {
    if (!mux || !mux->is_open) return ESP_ERR_INVALID_STATE;

    // Capture the chunk's absolute file offset (of the chunk HEADER,
    // not the data). idx1 offsets are relative to movi_data_start so
    // subtract that before storing.
    size_t chunk_hdr_off = cur_pos(mux->f);
    size_t data_off = 0;
    esp_err_t err = write_chunk(mux, FCC_00DC, data, size, &data_off);
    if (err != ESP_OK) return err;

    uint32_t rel = (uint32_t)(chunk_hdr_off - mux->movi_data_start);
    err = idx_append(mux, FCC_00DC, keyframe ? AVIIF_KEYFRAME : 0u, rel, (uint32_t)size);
    if (err != ESP_OK) return err;

    mux->video_frames_written++;
    mux->video_bytes_written += (uint32_t)size;
    return ESP_OK;
}

esp_err_t avi_mux_write_audio(avi_mux_t *mux, const void *data, size_t size,
                              uint32_t samples_covered) {
    if (!mux || !mux->is_open) return ESP_ERR_INVALID_STATE;

    size_t chunk_hdr_off = cur_pos(mux->f);
    size_t data_off = 0;
    esp_err_t err = write_chunk(mux, FCC_01WB, data, size, &data_off);
    if (err != ESP_OK) return err;

    uint32_t rel = (uint32_t)(chunk_hdr_off - mux->movi_data_start);
    err = idx_append(mux, FCC_01WB, 0u, rel, (uint32_t)size);
    if (err != ESP_OK) return err;

    mux->audio_samples_written += samples_covered;
    mux->audio_bytes_written   += (uint32_t)size;
    // Each call represents one MP3 frame (Shine emits one frame per
    // shine_encode_buffer call). Track the frame count for the audio
    // strh.dwLength patch at close time.
    if (samples_covered > 0) {
        mux->audio_frames_written++;
    }
    return ESP_OK;
}

esp_err_t avi_mux_close(avi_mux_t *mux) {
    if (!mux || !mux->is_open) return ESP_ERR_INVALID_STATE;
    FILE *f = mux->f;

    // Record the position right after the last movi chunk; that
    // becomes the END of the movi LIST. Everything written BEFORE
    // this point from movi_data_start is the movi contents.
    size_t movi_end = cur_pos(f);

    // Append idx1. Size = 8-byte chunk header + 16 * count.
    if (wr_fcc(f, FCC_IDX1)) return ESP_FAIL;
    uint32_t idx_bytes = mux->idx_count * 16u;
    if (wr_u32(f, idx_bytes)) return ESP_FAIL;
    if (idx_bytes > 0) {
        if (wr_bytes(f, mux->idx_entries, idx_bytes)) return ESP_FAIL;
    }
    size_t riff_end = cur_pos(f);

    // Patch movi LIST size: from movi_data_start (inclusive of "movi"
    // FourCC) to movi_end, that's the LIST's payload. The LIST size
    // field sits 4 bytes before movi_data_start.
    uint32_t movi_list_size = (uint32_t)(movi_end - mux->movi_data_start) + 4u; // +4 because LIST size includes the "movi" FourCC
    // Correction: movi_data_start already points AT the "movi" FourCC
    // — so movi_end - movi_data_start covers the FourCC plus the
    // chunk data. That is exactly the LIST size (LIST size field is
    // the count of bytes FOLLOWING the size field itself).
    movi_list_size = (uint32_t)(movi_end - mux->movi_data_start);
    if (fseek(f, (long)mux->movi_list_start, SEEK_SET)) return ESP_FAIL;
    if (wr_u32(f, movi_list_size)) return ESP_FAIL;

    // Patch RIFF size: bytes from file offset 8 (just after "RIFF"
    // and size field) to end of file.
    uint32_t riff_size = (uint32_t)(riff_end - 8u);
    if (fseek(f, (long)mux->riff_size_pos, SEEK_SET)) return ESP_FAIL;
    if (wr_u32(f, riff_size)) return ESP_FAIL;

    // Patch avih.dwTotalFrames and dwMaxBytesPerSec.
    if (fseek(f, (long)mux->avih_total_frames_pos, SEEK_SET)) return ESP_FAIL;
    if (wr_u32(f, mux->video_frames_written)) return ESP_FAIL;
    // Rough max bytes/sec estimate: video_bytes + audio_bytes divided
    // by duration in seconds. Guard against zero fps / zero frames.
    uint32_t dur_s = mux->video_fps ? (mux->video_frames_written / mux->video_fps) : 0u;
    uint32_t total_bps = 0;
    if (dur_s > 0) {
        total_bps = (mux->video_bytes_written + mux->audio_bytes_written) / dur_s;
    }
    if (fseek(f, (long)mux->avih_max_bytes_per_sec_pos, SEEK_SET)) return ESP_FAIL;
    if (wr_u32(f, total_bps)) return ESP_FAIL;

    // Patch video strh.dwLength and audio strh.dwLength.
    // Audio dwLength is in MP3 FRAMES — matching the strh layout
    // above (dwScale=samples/frame, dwRate=sample_rate), giving a
    // stream duration of dwLength × dwScale / dwRate seconds.
    if (fseek(f, (long)mux->vids_length_pos, SEEK_SET)) return ESP_FAIL;
    if (wr_u32(f, mux->video_frames_written)) return ESP_FAIL;
    if (fseek(f, (long)mux->auds_length_pos, SEEK_SET)) return ESP_FAIL;
    if (wr_u32(f, mux->audio_frames_written)) return ESP_FAIL;

    // Back to end and close.
    fflush(f);
    fastclose(f);
    mux->f = NULL;
    mux->is_open = false;

    if (mux->idx_entries) {
        free(mux->idx_entries);
        mux->idx_entries = NULL;
    }
    mux->idx_count = 0;
    mux->idx_capacity = 0;

    ESP_LOGI(TAG, "closed: %" PRIu32 " video frames, %" PRIu32 " audio samples, %zu B total",
             mux->video_frames_written, mux->audio_samples_written, riff_end);
    return ESP_OK;
}

void avi_mux_abort(avi_mux_t *mux) {
    if (!mux) return;
    if (mux->f) {
        fflush(mux->f);
        fastclose(mux->f);
        mux->f = NULL;
    }
    if (mux->idx_entries) {
        free(mux->idx_entries);
        mux->idx_entries = NULL;
    }
    mux->idx_count = 0;
    mux->idx_capacity = 0;
    mux->is_open = false;
}
