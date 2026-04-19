#include "video.h"

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_h264_enc_single.h"
#include "esp_h264_enc_single_hw.h"
#include "esp_h264_enc_param.h"
#include "esp_h264_types.h"

#include "layer3.h" // Shine public API

#include "avi_mux.h"
#include "camera_pipeline.h"
#include "microphone.h"

static const char *TAG = "video";

// --- Recording parameters ---------------------------------------------------
//
// Picked to match camera.md §9 and h264_changes.md constraints:
//   * 15 fps matches the target playback rate for h264bsd on the
//     videoplayer side.
//   * 400 x 320 is the exact output of PPA scale 8/16 = 0.5 against
//     an 800x640 sensor source frame. Both dims are already mult-of-16
//     so the H.264 encoder does not need frame cropping, and the 5:4
//     aspect ratio matches the source — no letterboxing or squish.
//   * 400 kbps yields ~25 KB peak / ~15 KB average per second of
//     video, well under even a slow SD card's sustained write rate.
//   * GOP=15 means one keyframe per second, which keeps seeks
//     responsive on the videoplayer side without inflating the
//     bitrate much.
//   * Audio is mono 22.05 kHz 64 kbps MPEG-II Layer III. At 44.1 kHz
//     stereo MPEG-I (our previous config), Shine was spending
//     ~30 ms/frame on RISC-V — 15 % over budget for the 26.1 ms
//     per-frame window — which caused ~13 % of mic samples to be
//     dropped at the stream buffer and the recorded audio to come
//     up short. Dropping to 22.05 kHz mono quarters the per-frame
//     workload (half the samples × half the channels) and leaves
//     generous headroom. Per-frame time budget at 576 samples /
//     22050 Hz is still 26.1 ms — identical to MPEG-I — but Shine
//     comes in around ~7-8 ms so we never drop samples. Quality-
//     wise this is voice-memo territory, appropriate for a handheld
//     mic: 11 kHz Nyquist covers all speech formants and sibilants.
//     Rate-wise the ESP32-P4 I2S still runs at 48 kHz nominal
//     (actual ~44.8 kHz — PLL_F160M is 2^11 × 5^7, no factor of 3)
//     and we resample 44.8 → 22.05 in microphone.c.
#define VIDEO_W                400u
#define VIDEO_H                320u
#define VIDEO_W_PADDED         400u                 // already mult-of-16
#define VIDEO_H_PADDED         320u                 // already mult-of-16
#define VIDEO_FPS              15u
#define VIDEO_BITRATE          400000u
#define VIDEO_GOP              15u

#define AUDIO_SAMPLE_RATE      22050u
#define AUDIO_CHANNELS         1u
#define AUDIO_BITRATE_KBPS     64
#define AUDIO_AVG_BYTES_PER_S  (AUDIO_BITRATE_KBPS * 1000 / 8) // 8000

// YUV420 "packed" (O_UYY_E_VYY) buffer size at padded stride — this
// is the exact size h264_enc expects when width/height are rounded
// up to the nearest multiple of 16 internally.
#define YUV_BUF_SIZE           ((size_t)VIDEO_W_PADDED * VIDEO_H_PADDED * 3u / 2u)

// H.264 output bitstream buffer. The encoder rejects the call
// up-front if out.raw_data.len is smaller than the input buffer
// length, regardless of how small the *actual* compressed frame
// ends up being — the docs say "allocated by the user for
// `in_frame.raw_data.len` bytes". At 400x320 YUV420 the input is
// 192000 bytes, so we round up to 256 KB with lots of headroom for
// any pathological keyframe that might blow through a tighter
// allocation.
#define H264_OUTBUF_SIZE       (256 * 1024)

// --- Recorder state ---------------------------------------------------------

// One compressed chunk queued up for the writer task. Data is owned
// by the producer until the writer pulls it off the queue — the
// writer then `free()`s the payload after writing it to the mux.
typedef struct {
    uint8_t *data;        // heap-allocated payload (writer frees)
    size_t   size;
    int64_t  ts_us;       // wall-clock timestamp (relative to record start)
    bool     is_video;
    bool     keyframe;    // video only
    uint32_t samples;     // audio only: samples covered by this chunk
} mux_chunk_t;

typedef struct {
    // File + mux state.
    avi_mux_t mux;
    char      path[128];

    // Hardware H.264 encoder.
    esp_h264_enc_handle_t enc;
    uint8_t  *yuv_buf;                    // PPA scratch, cache-line-aligned PSRAM
    uint8_t  *bs_buf;                     // encoder output bitstream

    // Shine MP3 encoder (runs on core 1 via the audio task).
    shine_t   shine;
    int       shine_samples_per_frame;    // returned by shine_samples_per_pass()
    int16_t  *pcm_scratch;                // one frame worth of interleaved int16_t PCM

    // Writer pipeline. Producer tasks (video_task, audio_task)
    // encode their frames and push them onto these two queues; the
    // writer task pops them in strict wall-clock timestamp order and
    // writes them to the AVI file, so the on-disk layout is always
    // time-ordered regardless of how the two producer tasks are
    // scheduled. Without this, ffmpeg flags the AVI as
    // "non-interleaved" and the Tanmatsu videoplayer hits audio
    // breaks as its audio decoder starves during runs of
    // consecutive video chunks.
    QueueHandle_t video_q;
    QueueHandle_t audio_q;
    TaskHandle_t  writer_task;
    volatile int  producers_done;         // incremented by each producer on exit

    // Producer task handles + lifecycle.
    TaskHandle_t video_task;
    TaskHandle_t audio_task;
    volatile bool running;
    int64_t  start_us;                    // esp_timer_get_time() at record start
    uint32_t video_frame_ix;              // sequential frame number (for keyframe detection via GOP)
} video_rec_t;

static video_rec_t s_rec = {0};

// Size of the per-stream queues. At 15 fps video + 27.8 Hz audio the
// writer only lags by at most a frame or two, so 32 slots per queue
// is generous.
#define MUX_QUEUE_DEPTH 32

// --- Filename helper (mirror of photo.c's) ---------------------------------

static void build_filename(const char *dir, char *out, size_t n) {
    time_t    now = time(NULL);
    struct tm tmv;
    localtime_r(&now, &tmv);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);

    snprintf(out, n, "%s/VID_%s.avi", dir, ts);
    struct stat st;
    if (stat(out, &st) != 0) return;
    for (int suffix = 1; suffix < 100; ++suffix) {
        snprintf(out, n, "%s/VID_%s_%d.avi", dir, ts, suffix);
        if (stat(out, &st) != 0) return;
    }
}

// --- Video task -------------------------------------------------------------

// Fill `dst` with one Shine frame of audio. If the INMP441 microphone
// is running (config toggle `mic_enabled` plus MODE_VIDEO is active
// in the UI), we pull samples from its ring buffer; any shortfall and
// the disabled case are both handled by zero-filling. The mic task
// delivers samples at its actual ~44.8 kHz frame rate which we label
// as 44.1 kHz here — see the AUDIO_SAMPLE_RATE comment for why.
static void fill_audio_frame(int16_t *dst, int samples) {
    size_t got = 0;
    if (microphone_is_running()) {
        // 100 ms timeout — the mic stream-buffer's trigger level is
        // one full audio frame, so Receive blocks for ~26 ms in the
        // steady state (one mic frame period) and returns as soon as
        // a complete frame is ready. The long timeout only matters if
        // the mic stalls entirely, in which case we zero-fill to keep
        // the recorder running.
        got = microphone_read_pcm(dst, (size_t)samples, pdMS_TO_TICKS(100));
    }
    if (got < (size_t)samples) {
        memset(dst + got, 0, ((size_t)samples - got) * sizeof(int16_t));
    }

    // Drain-rate telemetry — pair with microphone.c's rate log so we
    // can see if the audio task pulls faster or slower than the mic
    // pushes. A mismatch here is what causes chop + pitch shift.
    static uint64_t total_got       = 0;
    static uint64_t total_requested = 0;
    static int64_t  win_start_us    = 0;
    if (win_start_us == 0) win_start_us = esp_timer_get_time();
    total_got       += got;
    total_requested += (uint64_t)samples;
    int64_t now = esp_timer_get_time();
    int64_t win = now - win_start_us;
    if (win >= 5000000) {
        uint32_t got_hz = (uint32_t)(total_got       * 1000000ull / (uint64_t)win);
        uint32_t req_hz = (uint32_t)(total_requested * 1000000ull / (uint64_t)win);
        ESP_LOGI(TAG, "audio drain: got=%u Hz req=%u Hz (cfg=%u Hz)",
                 (unsigned)got_hz, (unsigned)req_hz, (unsigned)AUDIO_SAMPLE_RATE);
        total_got = total_requested = 0;
        win_start_us = now;
    }
}

// Wall-clock paced sleep. The caller tracks the absolute µs target
// for the next wake-up; this function sleeps the FreeRTOS tick count
// that, at worst, wakes us up slightly EARLY (FreeRTOS ticks are
// 10 ms on this build so any drift up to 10 ms per iteration is
// absorbed by the next cycle's shorter sleep). Over many iterations
// the AVERAGE period lines up exactly with the target, which
// keeps the AVI-declared fps and the actual task cadence identical.
// Critical for A/V sync: the naive vTaskDelayUntil(pdMS_TO_TICKS(66))
// approach rounded the video period down to 60 ms (→ 16.67 fps) and
// the audio period down to 30 ms (→ 33.3 Hz → 20 % fast-forwarded
// audio playback).
static void sleep_until_us(int64_t target_us) {
    int64_t now = esp_timer_get_time();
    int64_t delta_us = target_us - now;
    if (delta_us <= 0) return;
    // 10 ms tick at CONFIG_FREERTOS_HZ=100. Floor to ticks so we
    // never oversleep past the target; the remainder (<10 ms) is
    // eaten by the next iteration's shorter sleep.
    uint32_t us_per_tick = 1000000u / (uint32_t)configTICK_RATE_HZ;
    uint32_t ticks = (uint32_t)(delta_us / us_per_tick);
    if (ticks > 0) vTaskDelay(ticks);
}

// --- Writer task ------------------------------------------------------------
//
// Single consumer that pulls chunks from both producer queues and
// writes them to the AVI file in strict timestamp order. Runs on
// core 0 (default affinity); it's not timing-critical itself, just
// needs to keep up with the producers' ~43 chunks/sec combined.
//
// Ordering rule: while both producers are still running, the writer
// waits for BOTH queues to have at least one item before it picks
// the one with the earlier timestamp. That guarantees strict
// monotonic interleave — if it wrote an item from queue A while B
// was empty, B might later arrive with an earlier timestamp and the
// file would be out of order again.
//
// Once both producers have exited (producers_done == 2), the writer
// drains whatever is left in either queue (ordering doesn't matter
// at that point because no further chunks can arrive).
static void writer_task_fn(void *arg) {
    (void)arg;
    while (1) {
        mux_chunk_t v = {0}, a = {0};
        bool have_v = (xQueuePeek(s_rec.video_q, &v, 0) == pdTRUE);
        bool have_a = (xQueuePeek(s_rec.audio_q, &a, 0) == pdTRUE);

        bool producers_finished = (s_rec.producers_done >= 2);

        // Idle poll period: at least 1 FreeRTOS tick (10 ms on this
        // build). `pdMS_TO_TICKS(5)` would silently round DOWN to
        // zero and turn into a yield — which at this task's
        // priority (4, higher than the main app task) starves the
        // entire UI loop on core 0 and locks up the display. Use
        // an explicit tick count instead so the writer actually
        // sleeps and frees the CPU.
        const TickType_t idle_ticks = 1;  // 10 ms

        if (!have_v && !have_a) {
            if (producers_finished) break;  // nothing left to do
            vTaskDelay(idle_ticks);
            continue;
        }

        // If we only have one side AND producers are still running,
        // wait briefly for the other stream to catch up. This is the
        // interleaving-safety step: writing the available side now
        // might put a chunk on disk ahead of an earlier-timestamped
        // one arriving shortly from the other stream.
        if ((!have_v || !have_a) && !producers_finished) {
            vTaskDelay(idle_ticks);
            continue;
        }

        bool pick_v;
        if (have_v && have_a) {
            pick_v = (v.ts_us <= a.ts_us);
        } else {
            // producers finished — drain whichever side still has
            // data, order doesn't matter anymore.
            pick_v = have_v;
        }

        mux_chunk_t item;
        if (pick_v) {
            xQueueReceive(s_rec.video_q, &item, 0);
            esp_err_t werr = avi_mux_write_video(&s_rec.mux, item.data, item.size, item.keyframe);
            if (werr != ESP_OK) ESP_LOGE(TAG, "writer: avi video: %d", werr);
        } else {
            xQueueReceive(s_rec.audio_q, &item, 0);
            esp_err_t werr = avi_mux_write_audio(&s_rec.mux, item.data, item.size, item.samples);
            if (werr != ESP_OK) ESP_LOGE(TAG, "writer: avi audio: %d", werr);
        }
        free(item.data);
    }
    ESP_LOGI(TAG, "writer task exit");
    s_rec.writer_task = NULL;
    vTaskDelete(NULL);
}

static void video_task_fn(void *arg) {
    (void)arg;

    // Drift-free pacing: track the NEXT wake-up target in absolute
    // microseconds from the esp_timer_get_time() monotonic clock and
    // sleep just enough ticks to land on it, leaving any sub-tick
    // residual for the next iteration to absorb. See sleep_until_us
    // comment for why vTaskDelayUntil(pdMS_TO_TICKS(66)) isn't
    // accurate enough for A/V sync.
    const int64_t period_us = 1000000 / (int64_t)VIDEO_FPS;
    int64_t       next_us   = esp_timer_get_time() + period_us;

    // Rolling per-step accumulators logged once a second so we can
    // see where time actually goes if the task falls behind.
    uint64_t acc_snap_us = 0, acc_enc_us = 0, acc_write_us = 0;
    int      acc_n       = 0;

    while (s_rec.running) {
        sleep_until_us(next_us);
        next_us += period_us;
        if (!s_rec.running) break;

        int64_t t0 = esp_timer_get_time();

        esp_err_t err = camera_video_snapshot(s_rec.yuv_buf, YUV_BUF_SIZE,
                                              VIDEO_W, VIDEO_H,
                                              VIDEO_W_PADDED, VIDEO_H_PADDED);
        int64_t t1 = esp_timer_get_time();
        if (err != ESP_OK) {
            if (err != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "video snapshot: %d", err);
            }
            continue;
        }

        esp_h264_enc_in_frame_t  in  = {
            .raw_data = { .buffer = s_rec.yuv_buf, .len = YUV_BUF_SIZE },
            .pts      = (uint32_t)((t0 - s_rec.start_us) / 1000),
        };
        esp_h264_enc_out_frame_t out = {
            .raw_data = { .buffer = s_rec.bs_buf, .len = H264_OUTBUF_SIZE },
        };
        esp_h264_err_t herr = esp_h264_enc_process(s_rec.enc, &in, &out);
        int64_t t2 = esp_timer_get_time();
        if (herr != ESP_H264_ERR_OK) {
            ESP_LOGW(TAG, "h264 enc: %d (len=%" PRIu32 ")", (int)herr, out.length);
            continue;
        }
        if (out.length == 0) continue;

        bool keyframe = (out.frame_type == ESP_H264_FRAME_TYPE_IDR) ||
                        (out.frame_type == ESP_H264_FRAME_TYPE_I);

        // Copy the encoder's output into a fresh heap allocation
        // and push it onto the writer queue. The writer task frees
        // the allocation after it has written the chunk to disk.
        uint8_t *payload = malloc(out.length);
        if (!payload) {
            ESP_LOGE(TAG, "video chunk malloc failed (%" PRIu32 " bytes)", out.length);
            continue;
        }
        memcpy(payload, out.raw_data.buffer, out.length);
        mux_chunk_t chunk = {
            .data     = payload,
            .size     = out.length,
            .ts_us    = t0 - s_rec.start_us,
            .is_video = true,
            .keyframe = keyframe,
        };
        if (xQueueSend(s_rec.video_q, &chunk, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "video queue full, dropping frame");
            free(payload);
        }
        int64_t t3 = esp_timer_get_time();

        s_rec.video_frame_ix++;

        // Per-step profiling.
        acc_snap_us  += (uint64_t)(t1 - t0);
        acc_enc_us   += (uint64_t)(t2 - t1);
        acc_write_us += (uint64_t)(t3 - t2);
        acc_n++;
        if (acc_n >= 15) {
            ESP_LOGI(TAG, "v: snap=%llu enc=%llu write=%llu  avg us/frame (n=%d, len=%" PRIu32 ")",
                     (unsigned long long)(acc_snap_us  / acc_n),
                     (unsigned long long)(acc_enc_us   / acc_n),
                     (unsigned long long)(acc_write_us / acc_n),
                     acc_n, out.length);
            acc_snap_us = acc_enc_us = acc_write_us = 0;
            acc_n = 0;
        }
    }

    ESP_LOGI(TAG, "video task exit (queued %" PRIu32 " frames)", s_rec.video_frame_ix);
    s_rec.producers_done++;
    s_rec.video_task = NULL;
    vTaskDelete(NULL);
}

// --- Audio task (core 1) ----------------------------------------------------

static void audio_task_fn(void *arg) {
    (void)arg;

    int samples_per_frame = s_rec.shine_samples_per_frame;
    if (samples_per_frame <= 0) {
        ESP_LOGE(TAG, "audio task: invalid samples/frame %d", samples_per_frame);
        s_rec.audio_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Pacing: the microphone's stream buffer has a trigger level of
    // one audio frame worth of bytes, so fill_audio_frame's
    // xStreamBufferReceive blocks until a full frame is ready and
    // returns immediately after. That naturally rate-limits this
    // task to the mic's production rate — no need for a separate
    // wall-clock sleep, and no tick-rounding drift that made the
    // audio track come up short of the video track. If the mic
    // stalls entirely, the 100 ms Receive timeout kicks in and we
    // zero-fill to keep the recorder producing chunks.
    //
    // Channel-major pointer array. Shine's mono encoder reads only
    // channel 0 (see layer3.c:shine_encode_buffer — the stride is
    // config->wave.channels which is 1 for PCM_MONO), so a single-
    // entry array is all we need.
    int16_t *chan_bufs[1] = { s_rec.pcm_scratch };

    uint64_t acc_gen_us = 0, acc_enc_us = 0, acc_write_us = 0;
    int      acc_n      = 0;

    while (s_rec.running) {
        int64_t t0 = esp_timer_get_time();
        fill_audio_frame(s_rec.pcm_scratch, samples_per_frame);
        int64_t t1 = esp_timer_get_time();
        if (!s_rec.running) break;

        int written = 0;
        unsigned char *mp3 = shine_encode_buffer(s_rec.shine, chan_bufs, &written);
        int64_t t2 = esp_timer_get_time();
        if (mp3 != NULL && written > 0) {
            // One-shot dump of the first MP3 frame header so we can
            // verify the mode/version/rate bits Shine actually wrote.
            static bool first_header_logged = false;
            if (!first_header_logged && written >= 4) {
                ESP_LOGI(TAG, "mp3 hdr: %02x %02x %02x %02x  len=%d",
                         mp3[0], mp3[1], mp3[2], mp3[3], written);
                first_header_logged = true;
            }

            // Copy Shine's output (which gets overwritten on the
            // next call) into a fresh heap allocation and enqueue
            // for the writer task.
            uint8_t *payload = malloc(written);
            if (!payload) {
                ESP_LOGE(TAG, "audio chunk malloc failed (%d bytes)", written);
            } else {
                memcpy(payload, mp3, written);
                mux_chunk_t chunk = {
                    .data     = payload,
                    .size     = (size_t)written,
                    .ts_us    = t0 - s_rec.start_us,
                    .is_video = false,
                    .samples  = (uint32_t)samples_per_frame,
                };
                if (xQueueSend(s_rec.audio_q, &chunk, pdMS_TO_TICKS(100)) != pdTRUE) {
                    ESP_LOGW(TAG, "audio queue full, dropping frame");
                    free(payload);
                }
            }
        }
        int64_t t3 = esp_timer_get_time();

        acc_gen_us   += (uint64_t)(t1 - t0);
        acc_enc_us   += (uint64_t)(t2 - t1);
        acc_write_us += (uint64_t)(t3 - t2);
        acc_n++;
        if (acc_n >= 25) {
            ESP_LOGI(TAG, "a: gen=%llu enc=%llu write=%llu  avg us/frame (n=%d, len=%d)",
                     (unsigned long long)(acc_gen_us   / acc_n),
                     (unsigned long long)(acc_enc_us   / acc_n),
                     (unsigned long long)(acc_write_us / acc_n),
                     acc_n, written);
            acc_gen_us = acc_enc_us = acc_write_us = 0;
            acc_n = 0;
        }
    }

    // Flush any residual MP3 bytes still buffered inside Shine.
    // Push it through the same writer queue so strict ordering is
    // preserved — the writer task treats "zero samples covered" as
    // a flush marker that doesn't advance the audio timeline.
    int written = 0;
    unsigned char *mp3 = shine_flush(s_rec.shine, &written);
    if (mp3 != NULL && written > 0) {
        uint8_t *payload = malloc(written);
        if (payload) {
            memcpy(payload, mp3, written);
            mux_chunk_t chunk = {
                .data     = payload,
                .size     = (size_t)written,
                .ts_us    = esp_timer_get_time() - s_rec.start_us,
                .is_video = false,
                .samples  = 0u,
            };
            if (xQueueSend(s_rec.audio_q, &chunk, pdMS_TO_TICKS(100)) != pdTRUE) {
                free(payload);
            }
        }
    }

    ESP_LOGI(TAG, "audio task exit");
    s_rec.producers_done++;
    s_rec.audio_task = NULL;
    vTaskDelete(NULL);
}

// --- Setup helpers ---------------------------------------------------------

static esp_err_t setup_h264_encoder(void) {
    esp_h264_enc_cfg_hw_t cfg = {
        .pic_type = ESP_H264_RAW_FMT_O_UYY_E_VYY,  // universal — works on all P4 silicon revs
        .gop      = (uint8_t)VIDEO_GOP,
        .fps      = (uint8_t)VIDEO_FPS,
        .res      = { .width = (uint16_t)VIDEO_W, .height = (uint16_t)VIDEO_H },
        .rc       = {
            .bitrate = VIDEO_BITRATE,
            .qp_min  = 16,
            .qp_max  = 40,
        },
    };

    esp_h264_err_t herr = esp_h264_enc_hw_new(&cfg, &s_rec.enc);
    if (herr != ESP_H264_ERR_OK || s_rec.enc == NULL) {
        ESP_LOGE(TAG, "esp_h264_enc_hw_new: %d", (int)herr);
        return ESP_FAIL;
    }
    herr = esp_h264_enc_open(s_rec.enc);
    if (herr != ESP_H264_ERR_OK) {
        ESP_LOGE(TAG, "esp_h264_enc_open: %d", (int)herr);
        esp_h264_enc_del(s_rec.enc);
        s_rec.enc = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "h264 encoder ready: %ux%u @ %u fps, %u bps, GOP %u",
             (unsigned)VIDEO_W, (unsigned)VIDEO_H, (unsigned)VIDEO_FPS,
             (unsigned)VIDEO_BITRATE, (unsigned)VIDEO_GOP);
    return ESP_OK;
}

static esp_err_t setup_shine(void) {
    shine_config_t cfg;
    shine_set_config_mpeg_defaults(&cfg.mpeg);
    // Mono MPEG-II at 22.05 kHz — see the AUDIO_* defines comment at
    // the top of this file for the reasoning. The videoplayer's
    // convert_h264.sh produces stereo 44.1 kHz today; the videoplayer
    // decoder side will need a pass to confirm it handles mono MP3s
    // cleanly without channel-doubling or speed-doubling artefacts
    // before this is considered done.
    cfg.wave.channels   = PCM_MONO;
    cfg.wave.samplerate = AUDIO_SAMPLE_RATE;
    cfg.mpeg.mode       = MONO;
    cfg.mpeg.bitr       = AUDIO_BITRATE_KBPS;
    cfg.mpeg.emph       = NONE;
    cfg.mpeg.copyright  = 0;
    cfg.mpeg.original   = 1;

    int mpeg_version = shine_check_config(cfg.wave.samplerate, cfg.mpeg.bitr);
    if (mpeg_version < 0) {
        ESP_LOGE(TAG, "shine: invalid sample rate / bitrate combo");
        return ESP_FAIL;
    }

    s_rec.shine = shine_initialise(&cfg);
    if (!s_rec.shine) {
        ESP_LOGE(TAG, "shine_initialise failed");
        return ESP_ERR_NO_MEM;
    }
    s_rec.shine_samples_per_frame = shine_samples_per_pass(s_rec.shine);
    if (s_rec.shine_samples_per_frame <= 0 ||
        s_rec.shine_samples_per_frame > SHINE_MAX_SAMPLES) {
        ESP_LOGE(TAG, "shine: unexpected samples per pass %d",
                 s_rec.shine_samples_per_frame);
        shine_close(s_rec.shine);
        s_rec.shine = NULL;
        return ESP_FAIL;
    }
    s_rec.pcm_scratch = heap_caps_calloc(s_rec.shine_samples_per_frame,
                                         sizeof(int16_t),
                                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!s_rec.pcm_scratch) {
        ESP_LOGE(TAG, "pcm_scratch alloc failed");
        shine_close(s_rec.shine);
        s_rec.shine = NULL;
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "shine ready: %u Hz mono %d kbps, %d samples/frame",
             (unsigned)AUDIO_SAMPLE_RATE, AUDIO_BITRATE_KBPS,
             s_rec.shine_samples_per_frame);
    return ESP_OK;
}

static void teardown_all(void) {
    if (s_rec.enc) {
        esp_h264_enc_close(s_rec.enc);
        esp_h264_enc_del(s_rec.enc);
        s_rec.enc = NULL;
    }
    if (s_rec.shine) {
        shine_close(s_rec.shine);
        s_rec.shine = NULL;
    }
    if (s_rec.yuv_buf)     { free(s_rec.yuv_buf);     s_rec.yuv_buf     = NULL; }
    if (s_rec.bs_buf)      { free(s_rec.bs_buf);      s_rec.bs_buf      = NULL; }
    if (s_rec.pcm_scratch) { free(s_rec.pcm_scratch); s_rec.pcm_scratch = NULL; }
    // Drain any straggler chunks left in the queues and free their
    // payloads before deleting the queues themselves.
    if (s_rec.video_q) {
        mux_chunk_t item;
        while (xQueueReceive(s_rec.video_q, &item, 0) == pdTRUE) {
            free(item.data);
        }
        vQueueDelete(s_rec.video_q);
        s_rec.video_q = NULL;
    }
    if (s_rec.audio_q) {
        mux_chunk_t item;
        while (xQueueReceive(s_rec.audio_q, &item, 0) == pdTRUE) {
            free(item.data);
        }
        vQueueDelete(s_rec.audio_q);
        s_rec.audio_q = NULL;
    }
}

// --- Public API -------------------------------------------------------------

esp_err_t video_record_start(const char *dcim_dir, char *out_path, size_t out_path_sz) {
    if (s_rec.running) return ESP_ERR_INVALID_STATE;
    if (!dcim_dir || !out_path || out_path_sz == 0) return ESP_ERR_INVALID_ARG;
    out_path[0] = '\0';

    memset(&s_rec, 0, sizeof(s_rec));

    // 1. Buffers.
    s_rec.yuv_buf = heap_caps_aligned_calloc(64, 1, YUV_BUF_SIZE,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!s_rec.yuv_buf) {
        ESP_LOGE(TAG, "yuv_buf alloc failed (%zu bytes)", (size_t)YUV_BUF_SIZE);
        return ESP_ERR_NO_MEM;
    }
    s_rec.bs_buf = heap_caps_aligned_calloc(16, 1, H264_OUTBUF_SIZE,
                                            MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!s_rec.bs_buf) {
        ESP_LOGE(TAG, "bs_buf alloc failed");
        teardown_all();
        return ESP_ERR_NO_MEM;
    }

    // Writer-side queues: fixed-depth FreeRTOS queues hold
    // mux_chunk_t descriptors (the actual payloads are heap-
    // allocated by the producers and freed by the writer).
    s_rec.video_q = xQueueCreate(MUX_QUEUE_DEPTH, sizeof(mux_chunk_t));
    s_rec.audio_q = xQueueCreate(MUX_QUEUE_DEPTH, sizeof(mux_chunk_t));
    if (!s_rec.video_q || !s_rec.audio_q) {
        ESP_LOGE(TAG, "queue create failed");
        teardown_all();
        return ESP_ERR_NO_MEM;
    }

    // 2. Encoders.
    esp_err_t err = setup_h264_encoder();
    if (err != ESP_OK) { teardown_all(); return err; }
    err = setup_shine();
    if (err != ESP_OK) { teardown_all(); return err; }

    // 3. File + AVI headers.
    build_filename(dcim_dir, s_rec.path, sizeof(s_rec.path));
    err = avi_mux_open(&s_rec.mux, s_rec.path,
                       VIDEO_W, VIDEO_H, VIDEO_FPS,
                       AUDIO_SAMPLE_RATE, (uint16_t)AUDIO_CHANNELS,
                       AUDIO_AVG_BYTES_PER_S,
                       (uint16_t)s_rec.shine_samples_per_frame);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "avi_mux_open: %d", err);
        teardown_all();
        return err;
    }
    snprintf(out_path, out_path_sz, "%s", s_rec.path);

    // 4. Kick off the tasks. Three tasks total:
    //
    //   - video_task (core 0): encodes H.264 frames and enqueues
    //     chunks onto s_rec.video_q.
    //   - audio_task (core 1, pinned): encodes MP3 frames with
    //     Shine and enqueues chunks onto s_rec.audio_q. Pinning
    //     core 1 keeps the audio path isolated from the camera
    //     pipeline and the main UI loop; when the real I²S mic
    //     lands it will need deterministic timing that shared-core
    //     scheduling can't give.
    //   - writer_task (core 0): consumes both queues in strict
    //     timestamp order and writes AVI chunks. Runs at a lower
    //     priority than the producers so it never steals CPU from
    //     the hardware-bound encode steps, but gets plenty of
    //     wall-clock time to keep up with the ~43 chunks/sec
    //     combined rate.
    // Drop anything the mic has been accumulating while the user sat
    // in the video-mode HUD: we want the first AVI audio chunk to
    // line up with the video start timestamp, not with sound from
    // before the user pressed record.
    microphone_capture_reset();

    s_rec.running        = true;
    s_rec.producers_done = 0;
    s_rec.start_us       = esp_timer_get_time();
    s_rec.video_frame_ix = 0;
    BaseType_t ok1 = xTaskCreate(video_task_fn, "vid_rec", 6144, NULL, 6, &s_rec.video_task);
    BaseType_t ok2 = xTaskCreatePinnedToCore(audio_task_fn, "aud_rec", 6144, NULL, 6,
                                             &s_rec.audio_task, 1);
    BaseType_t ok3 = xTaskCreate(writer_task_fn, "avi_wr",   4096, NULL, 4, &s_rec.writer_task);
    if (ok1 != pdPASS || ok2 != pdPASS || ok3 != pdPASS) {
        ESP_LOGE(TAG, "task create failed (v=%d a=%d w=%d)", ok1, ok2, ok3);
        s_rec.running = false;
        // If any task spawned, it'll exit on next running check.
        vTaskDelay(pdMS_TO_TICKS(50));
        avi_mux_abort(&s_rec.mux);
        teardown_all();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "recording → %s", s_rec.path);
    return ESP_OK;
}

esp_err_t video_record_stop(void) {
    if (!s_rec.running) return ESP_ERR_INVALID_STATE;

    // Shut down the producer tasks first. Each increments
    // producers_done on exit, which lets the writer task switch
    // from "wait for both queues" mode to "drain whichever side has
    // data" mode.
    s_rec.running = false;
    for (int i = 0; i < 100; ++i) {
        if (!s_rec.video_task && !s_rec.audio_task) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_rec.video_task) {
        ESP_LOGW(TAG, "forcing video task delete");
        vTaskDelete(s_rec.video_task);
        s_rec.video_task = NULL;
        s_rec.producers_done++;
    }
    if (s_rec.audio_task) {
        ESP_LOGW(TAG, "forcing audio task delete");
        vTaskDelete(s_rec.audio_task);
        s_rec.audio_task = NULL;
        s_rec.producers_done++;
    }

    // Writer task will observe producers_done == 2 and drain both
    // queues before exiting. Give it up to 2 s — the queues can
    // hold up to MUX_QUEUE_DEPTH chunks on each side and each
    // write is a few ms.
    for (int i = 0; i < 200; ++i) {
        if (!s_rec.writer_task) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_rec.writer_task) {
        ESP_LOGW(TAG, "forcing writer task delete");
        vTaskDelete(s_rec.writer_task);
        s_rec.writer_task = NULL;
    }

    // Finalise + close the file.
    esp_err_t err = avi_mux_close(&s_rec.mux);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "avi_mux_close: %d", err);
    }

    teardown_all();
    return err;
}

bool video_is_recording(void) {
    return s_rec.running;
}

uint32_t video_recording_duration_ms(void) {
    if (!s_rec.running) return 0;
    int64_t d = esp_timer_get_time() - s_rec.start_us;
    if (d < 0) return 0;
    return (uint32_t)(d / 1000);
}
