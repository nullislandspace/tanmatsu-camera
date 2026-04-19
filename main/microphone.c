#include "microphone.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "hal/gpio_types.h"

static const char *TAG = "microphone";

// Port 1 is deliberate: I2S0 belongs to the ES8156 codec path in the
// BSP, so keeping the mic on I2S1 avoids any future conflict with
// audio playback.
#define MIC_I2S_PORT       I2S_NUM_1

// Clock/slot plan (notes from hard-won experience):
//
//   * INMP441 spec min BCLK ≈ 2.048 MHz. Configuring the I2S for
//     48 kHz with 32-bit slots gives BCLK = 48000 × 64 = 3.072 MHz,
//     comfortably in the mic's range.
//
//   * ESP32-P4 I2S has no APLL path — only PLL_F160M / XTAL —
//     and 160 MHz factors as 2^11 × 5^7 (no factor of 3). That
//     means audio rates like 48 / 44.1 / 24 kHz cannot be hit
//     exactly; the driver's fractional divider lands at a nearby
//     rate. For a 48 kHz request we measure ~44.8 kHz actual.
//
//   * The MONO slot mask on the P4 RX path does NOT filter the
//     unused slot — we confirmed experimentally that a MONO+LEFT
//     config still delivers both slots interleaved in DMA, with
//     the RIGHT slot showing the floating DIN pin (all 0xFFFFFFFF
//     because the INMP441 tri-states when L/R=GND makes it drive
//     LEFT only). So we configure STEREO explicitly and pick the
//     LEFT samples ourselves; at ~44.8 kHz frame rate the stream
//     is ~89.6 kHz of interleaved int32 samples, half of which
//     carry the mic audio.
//
//   * Downstream we label the audio as 44.1 kHz (the Shine rate
//     closest to the real ~44.8 kHz). The 1.6 % rate mismatch
//     comes out to a pitch shift of about 0.27 semitones — below
//     the threshold of perception for speech.
#define MIC_I2S_RATE_NOM   48000u

// Resampler constants. Empirically the LEFT-channel frame rate on a
// 48 kHz I2S config lands at ~44,800 Hz (the P4's fractional divider
// can't hit 48 kHz exactly — see the long comment above). We resample
// to 22,050 Hz so the output matches the MPEG-II Layer III rate that
// Shine encodes below (and that the AVI file advertises). Dropping
// to 22.05 kHz mono makes Shine's per-frame workload about a quarter
// of the 44.1 kHz stereo configuration, which was bottlenecking the
// audio task at ~30 ms/frame on RISC-V and causing ~13 % of samples
// to be dropped at the stream buffer.
//
// RESAMPLE_STEP_Q16 is the fixed-point increment added to the phase
// accumulator for each *output* sample, expressed in units where
// 65536 = one source-sample interval. For downsampling
// (src_hz > dst_hz) this is > 65536.
#define MIC_SRC_HZ         44800u
#define MIC_DST_HZ         22050u
#define RESAMPLE_STEP_Q16  ((uint32_t)((MIC_SRC_HZ * 65536ull) / MIC_DST_HZ))

// Pin assignments — see microphone.h header comment and
// microphone_inmp441.md for the wiring table.
#define MIC_BCLK_GPIO      GPIO_NUM_54
#define MIC_WS_GPIO        GPIO_NUM_49
#define MIC_DIN_GPIO       GPIO_NUM_53

// How many raw 32-bit samples we read per DMA pull. 1024 @ 32 kHz is
// 32 ms — roughly 30 Hz peak updates (smooth meter, trivial CPU) and
// one pull is slightly shorter than one Shine frame (36 ms @ 16 kHz)
// so the stream buffer always has data ready when the audio task
// asks for the next encode batch.
#define MIC_DMA_FRAMES     1024u

// Recorder-side ring buffer. 32 KB holds ~370 ms of 44.1 kHz int16
// mono — generous headroom that also absorbs the slow drift between
// the mic's actual ~44.8 kHz output and the 44.1 kHz drain rate
// (the two rates differ by ~1.6 %, giving ~700 samples/sec of
// accumulation; this buffer takes ~23 s to fill at that drift).
#define MIC_STREAM_BYTES   32768u

// Digital gain applied after the 16-bit extraction. Normal speech at
// arm's length produces raw peaks around ±1M (≈±3900 after the >> 8
// extraction). 4× puts that in the 15k-ish range (≈50 % of int16
// scale), visible on the meter and audible on playback. Loud shouts
// (observed raw peak ±13M → ±51k before gain) already exceed int16
// scale, so they saturate cleanly — acceptable for a voice recorder.
// Exposed as a runtime setting via microphone_set_gain() so the user
// can tune it from the config menu / volume buttons.
#define MIC_GAIN_MIN       1
#define MIC_GAIN_MAX       8
#define MIC_GAIN_DEFAULT   4

// One-pole IIR low-pass filter coefficient in Q16.16. The INMP441
// produces a noticeable amount of high-frequency self-noise + digital
// pickup from the ESP32-P4 that our gain stage makes obvious; voice
// energy is concentrated below ~4 kHz and everything above is
// effectively noise for a handheld voice recorder. Cutoff of ~4 kHz
// at the mic's 44.8 kHz frame rate: α = 1 - exp(-2π × 4000 / 44800)
// ≈ 0.430, Q16 encoded = 28180. We cascade the filter twice
// (see mic_task_fn) for a -12 dB/oct rolloff, which knocks HF noise
// down by a further ~9 dB at 11 kHz vs a single pole while leaving
// voice formants (<4 kHz) essentially untouched.
#define MIC_LPF_ALPHA_Q16  28180u

static i2s_chan_handle_t    s_rx_handle  = NULL;
static TaskHandle_t         s_task       = NULL;
static StreamBufferHandle_t s_pcm_stream = NULL;
static volatile bool        s_running    = false;
static volatile uint16_t    s_peak_level = 0;
static volatile int         s_gain       = MIC_GAIN_DEFAULT;

// Debug raw-capture state. When s_dbg_active is true, the mic task
// appends each i2s_channel_read() into this buffer (up to the
// configured capacity) and clears the flag when full. The caller
// (microphone_debug_raw_dump) allocates the buffer in PSRAM, waits
// for the capture to finish, writes the WAV file, and frees.
static int32_t * volatile   s_dbg_buffer   = NULL;
static volatile size_t      s_dbg_capacity = 0;
static volatile size_t      s_dbg_write_ix = 0;
static volatile bool        s_dbg_active   = false;

// Background reader. Runs continuously between microphone_start()
// and microphone_stop(); blocking i2s_channel_read() naturally paces
// the loop to the DMA rate.
static void mic_task_fn(void *arg) {
    (void)arg;

    int32_t *raw = heap_caps_malloc(MIC_DMA_FRAMES * sizeof(int32_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    // We emit one sample per I2S frame (2 slots → 1 LEFT sample).
    int16_t *pcm = heap_caps_malloc((MIC_DMA_FRAMES / 2) * sizeof(int16_t),
                                    MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!raw || !pcm) {
        ESP_LOGE(TAG, "scratch alloc failed");
        if (raw) free(raw);
        if (pcm) free(pcm);
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    // Sample-rate telemetry. Log once every 5 s so we can verify
    // the producer / consumer rates are still matched after the
    // resampler (the AVI file claims 44.1 kHz and a mismatch here
    // would translate to pitch shift at playback).
    uint64_t total_raw_samples     = 0;
    uint64_t total_emitted_samples = 0;
    int64_t  rate_window_start_us  = esp_timer_get_time();

    // Resampler state — persists across I2S reads so the linear
    // interpolation works continuously across block boundaries.
    // phase_q16 is the sub-sample position inside the (prev_sample,
    // next_sample) interval in Q16.16; prev_sample holds the last
    // input so the interpolator can cross block seams without
    // hiccuping.
    uint32_t phase_q16   = 0;
    int16_t  prev_sample = 0;

    // Low-pass filter state — the previous filtered output, so the
    // one-pole IIR y[n] = α·x[n] + (1-α)·y[n-1] survives across
    // blocks. Stored as int32 to keep a little extra headroom for
    // the Q16 accumulator arithmetic. Two stages are cascaded for a
    // -12 dB/oct rolloff; lpf_prev1 is the first stage's output
    // (which also feeds the second stage), lpf_prev2 is the second
    // stage's output.
    int32_t  lpf_prev1   = 0;
    int32_t  lpf_prev2   = 0;

    while (s_running) {
        size_t got_bytes = 0;
        esp_err_t err = i2s_channel_read(s_rx_handle, raw,
                                         MIC_DMA_FRAMES * sizeof(int32_t),
                                         &got_bytes, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "i2s read: %d", err);
            }
            continue;
        }
        size_t n = got_bytes / sizeof(int32_t);
        if (n == 0) continue;

        // If a debug raw capture is in progress, copy this block into
        // the PSRAM buffer. We do this before gain/resample/filter so
        // the caller gets the unmodified DMA output.
        if (s_dbg_active && s_dbg_buffer) {
            size_t remaining = s_dbg_capacity - s_dbg_write_ix;
            size_t to_copy   = (n < remaining) ? n : remaining;
            if (to_copy > 0) {
                memcpy((int32_t *)s_dbg_buffer + s_dbg_write_ix,
                       raw, to_copy * sizeof(int32_t));
                s_dbg_write_ix += to_copy;
            }
            if (s_dbg_write_ix >= s_dbg_capacity) {
                s_dbg_active = false;
            }
        }

        // The DMA stream is I2S STEREO interleaved as L,R,L,R,... In
        // each pair the LEFT sample carries the mic audio; RIGHT is
        // the floating DIN pin during the unused slot (reads as
        // 0xFFFFFFFF since the mic tri-states when its L/R strap is
        // tied to GND). We iterate stride-2 and only process LEFT.
        //
        // Verified experimentally (microphone_debug_raw_dump + WAV
        // spectrum analysis): the ESP32-P4 driver in STEREO mode
        // delivers the audio in bits [23:8] of the 32-bit slot (top
        // byte is sign extension, bottom 5 bits are slot-boundary
        // bleed from the floating RIGHT slot and always read
        // `11111`). Shifting a signed int32 right by 8 therefore
        // discards the garbage bits cleanly and leaves the audio in
        // the low 16 bits of the result. Multiplying by MIC_GAIN
        // and clamping to int16 saturates loud transients while
        // lifting normal speech to usable levels.
        //
        // A streaming linear-interpolation resampler then converts
        // from the mic's actual ~44.8 kHz frame rate down to the
        // 44.1 kHz rate we advertise downstream. Per input sample
        // the while-loop emits 0 or 1 output samples depending on
        // where phase_q16 sits; over time the emit rate averages
        // out to src/dst = 44100/44800 ≈ 0.984 outputs per input.
        // That keeps producer and consumer rates matched so the
        // stream buffer never drifts full or starves.
        uint16_t peak   = 0;
        size_t   out_ix = 0;
        for (size_t i = 0; i < n; i += 2) {
            int32_t s32 = ((int32_t)(raw[i] >> 8)) * (int32_t)s_gain;
            if (s32 >  INT16_MAX) s32 =  INT16_MAX;
            if (s32 <  INT16_MIN) s32 =  INT16_MIN;

            // Two-stage cascaded one-pole IIR low-pass: each stage is
            // y[n] = α·x[n] + (1-α)·y[n-1], MIC_LPF_ALPHA_Q16 is α in
            // Q16. int64 intermediate avoids the sign-bit overflow
            // that would occur mixing two int32 products near full
            // scale. Cascading gives -12 dB/oct rolloff with the same
            // cutoff as a single pole.
            uint32_t one_minus_alpha = 65536u - MIC_LPF_ALPHA_Q16;
            int64_t  acc1 =
                (int64_t)s32       * (int64_t)MIC_LPF_ALPHA_Q16 +
                (int64_t)lpf_prev1 * (int64_t)one_minus_alpha;
            int32_t  y1 = (int32_t)(acc1 >> 16);
            lpf_prev1   = y1;
            int64_t  acc2 =
                (int64_t)y1        * (int64_t)MIC_LPF_ALPHA_Q16 +
                (int64_t)lpf_prev2 * (int64_t)one_minus_alpha;
            int32_t  y = (int32_t)(acc2 >> 16);
            if (y >  INT16_MAX) y =  INT16_MAX;
            if (y <  INT16_MIN) y =  INT16_MIN;
            lpf_prev2   = y;
            int16_t x   = (int16_t)y;

            int32_t v = (x < 0) ? -(int32_t)x : (int32_t)x;
            if ((uint32_t)v > peak) peak = (uint16_t)v;

            while (phase_q16 < 65536u) {
                int32_t diff   = (int32_t)x - (int32_t)prev_sample;
                // int64 to avoid overflow: diff ≤ 2^16, phase_q16 ≤ 2^16,
                // so the product is up to 2^32, outside int32 range.
                int32_t interp = (int32_t)prev_sample +
                                 (int32_t)(((int64_t)diff * (int64_t)phase_q16) >> 16);
                pcm[out_ix++] = (int16_t)interp;
                phase_q16    += RESAMPLE_STEP_Q16;
            }
            phase_q16   -= 65536u;
            prev_sample  = x;
        }
        s_peak_level = peak;

        if (s_pcm_stream && out_ix > 0) {
            // Non-blocking: if the recorder isn't consuming (not
            // recording) the buffer fills and further sends drop.
            // The recorder resets the buffer at record-start, so
            // stale data never reaches the AVI file.
            xStreamBufferSend(s_pcm_stream, pcm, out_ix * sizeof(int16_t), 0);
        }

        total_raw_samples     += n;
        total_emitted_samples += out_ix;
        int64_t now_us = esp_timer_get_time();
        int64_t win_us = now_us - rate_window_start_us;
        if (win_us >= 5000000) {
            uint32_t raw_hz = (uint32_t)(total_raw_samples     * 1000000ull / (uint64_t)win_us);
            uint32_t out_hz = (uint32_t)(total_emitted_samples * 1000000ull / (uint64_t)win_us);
            ESP_LOGI(TAG, "rate: raw=%u Hz emitted=%u Hz (cfg=%u Hz)",
                     (unsigned)raw_hz, (unsigned)out_hz,
                     (unsigned)MIC_I2S_RATE_NOM);
            total_raw_samples = total_emitted_samples = 0;
            rate_window_start_us = now_us;
        }
    }

    free(raw);
    free(pcm);
    s_peak_level = 0;
    ESP_LOGI(TAG, "task exit");
    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t microphone_start(void) {
    if (s_running) return ESP_OK;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(MIC_I2S_PORT, I2S_ROLE_MASTER);
    // 4 DMA descriptors × 512 frames = ~64 ms of DMA depth at
    // 32 kHz. The reader task pulls every 32 ms so we've got a full
    // period of slack before the DMA would overflow.
    // In STEREO + 32-bit slots each frame is 8 bytes, so a 504-frame
    // descriptor (the P4 driver's max per-descriptor size) holds
    // 4032 bytes. Pick a smaller frame_num so the driver doesn't
    // clamp and we span whole descriptors per read — and bump the
    // descriptor count up to keep ~64 ms of DMA depth.
    chan_cfg.dma_desc_num  = 8;
    chan_cfg.dma_frame_num = 256;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %d", err);
        return err;
    }

    // 32-bit data + 32-bit slot + STEREO → 64 BCLK per frame, BCLK
    // ≈ 3.072 MHz at 48 kHz (inside INMP441 spec). STEREO is
    // explicit here because the MONO slot mask doesn't actually
    // filter the unused slot on the P4 RX side — we pick LEFT in
    // software in mic_task_fn().
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MIC_I2S_RATE_NOM),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT,
                                                       I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_BCLK_GPIO,
            .ws   = MIC_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = MIC_DIN_GPIO,
            .invert_flags = { false, false, false },
        },
    };

    err = i2s_channel_init_std_mode(s_rx_handle, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode: %d", err);
        i2s_del_channel(s_rx_handle);
        s_rx_handle = NULL;
        return err;
    }

    // Trigger level = 1152 bytes = 576 int16 mono samples = one MPEG-II
    // Layer III frame at 22.05 kHz. With this, xStreamBufferReceive
    // blocks precisely until a full audio frame's worth is available,
    // which naturally paces the recorder's audio task to the mic's
    // production rate (no tick-rounded sleep needed on its side). For
    // other target rates/codecs we'd need to adjust — but right now
    // 22.05 kHz MPEG-II mono is baked into the whole audio chain.
    s_pcm_stream = xStreamBufferCreate(MIC_STREAM_BYTES, 1152);
    if (!s_pcm_stream) {
        ESP_LOGE(TAG, "stream buffer create failed");
        i2s_del_channel(s_rx_handle);
        s_rx_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    err = i2s_channel_enable(s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %d", err);
        vStreamBufferDelete(s_pcm_stream);
        s_pcm_stream = NULL;
        i2s_del_channel(s_rx_handle);
        s_rx_handle = NULL;
        return err;
    }

    s_peak_level = 0;
    s_running    = true;
    BaseType_t ok = xTaskCreate(mic_task_fn, "mic_rx", 4096, NULL, 5, &s_task);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        s_running = false;
        i2s_channel_disable(s_rx_handle);
        vStreamBufferDelete(s_pcm_stream);
        s_pcm_stream = NULL;
        i2s_del_channel(s_rx_handle);
        s_rx_handle = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "started (I2S%d, %u Hz nominal stereo → LEFT mono, SCK=%d WS=%d DIN=%d)",
             (int)MIC_I2S_PORT, (unsigned)MIC_I2S_RATE_NOM,
             (int)MIC_BCLK_GPIO, (int)MIC_WS_GPIO, (int)MIC_DIN_GPIO);
    return ESP_OK;
}

void microphone_stop(void) {
    if (!s_running) return;
    s_running = false;
    // Reader task observes s_running == false on its next i2s_read
    // timeout and exits cleanly. Wait up to 1 s before force-delete.
    for (int i = 0; i < 100; i++) {
        if (!s_task) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_task) {
        ESP_LOGW(TAG, "forcing task delete");
        vTaskDelete(s_task);
        s_task = NULL;
    }
    if (s_rx_handle) {
        i2s_channel_disable(s_rx_handle);
        i2s_del_channel(s_rx_handle);
        s_rx_handle = NULL;
    }
    if (s_pcm_stream) {
        vStreamBufferDelete(s_pcm_stream);
        s_pcm_stream = NULL;
    }
    s_peak_level = 0;
    ESP_LOGI(TAG, "stopped");
}

bool microphone_is_running(void) {
    return s_running;
}

uint16_t microphone_peak_level(void) {
    return s_running ? s_peak_level : (uint16_t)0;
}

void microphone_capture_reset(void) {
    if (s_pcm_stream) {
        xStreamBufferReset(s_pcm_stream);
    }
}

void microphone_set_gain(int gain) {
    if (gain < MIC_GAIN_MIN) gain = MIC_GAIN_MIN;
    if (gain > MIC_GAIN_MAX) gain = MIC_GAIN_MAX;
    s_gain = gain;
}

int microphone_get_gain(void) {
    return s_gain;
}

size_t microphone_read_pcm(int16_t *dst, size_t n_samples, TickType_t timeout) {
    if (!s_running || !s_pcm_stream || !dst || n_samples == 0) return 0;

    // FreeRTOS stream buffers only honour the trigger level when the
    // receiver is *blocked* — once the reader is scheduled it will
    // return whatever happens to be in the buffer, even if that's
    // far less than requested (the relevant check in stream_buffer.c
    // compares against xBytesToStoreMessageLength, which is 0 for a
    // plain stream buffer, NOT against the trigger level). That
    // meant the audio task was seeing partial frames nearly every
    // iteration; the caller zero-filled the missing half and we
    // encoded silence 40 % of the time. Loop here until we've
    // actually accumulated the full requested amount or the overall
    // timeout expires, so the caller gets contiguous live audio.
    size_t target_bytes = n_samples * sizeof(int16_t);
    size_t got_bytes    = 0;
    TickType_t start    = xTaskGetTickCount();
    while (got_bytes < target_bytes) {
        TickType_t elapsed   = xTaskGetTickCount() - start;
        TickType_t remaining = (elapsed >= timeout) ? 0 : (timeout - elapsed);
        size_t n = xStreamBufferReceive(s_pcm_stream,
                                        (uint8_t *)dst + got_bytes,
                                        target_bytes - got_bytes,
                                        remaining);
        if (n == 0) break; // timeout reached with nothing new
        got_bytes += n;
    }
    return got_bytes / sizeof(int16_t);
}

// --- Debug raw dump --------------------------------------------------------

// Minimal canonical 32-bit PCM stereo WAV header. We write it all at
// once so the file opens in any audio tool without needing special
// codecs. Everything is little-endian per the spec.
static void write_wav_header(FILE *f, uint32_t sample_rate, uint32_t num_frames) {
    const uint16_t channels        = 2;
    const uint16_t bits_per_sample = 32;
    const uint16_t block_align     = channels * (bits_per_sample / 8);
    const uint32_t byte_rate       = sample_rate * block_align;
    const uint32_t data_size       = num_frames * block_align;
    const uint32_t riff_size       = 36 + data_size;
    const uint32_t fmt_size        = 16;
    const uint16_t audio_format    = 1;  // PCM

    fwrite("RIFF", 1, 4, f);
    fwrite(&riff_size,       4, 1, f);
    fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f);
    fwrite(&fmt_size,        4, 1, f);
    fwrite(&audio_format,    2, 1, f);
    fwrite(&channels,        2, 1, f);
    fwrite(&sample_rate,     4, 1, f);
    fwrite(&byte_rate,       4, 1, f);
    fwrite(&block_align,     2, 1, f);
    fwrite(&bits_per_sample, 2, 1, f);
    fwrite("data", 1, 4, f);
    fwrite(&data_size,       4, 1, f);
}

esp_err_t microphone_debug_raw_dump(const char *path, int seconds) {
    if (!s_running) return ESP_ERR_INVALID_STATE;
    if (!path || seconds < 1 || seconds > 30) return ESP_ERR_INVALID_ARG;

    // ~89,600 int32 samples per second at the nominal 48 kHz stereo
    // config. Allocate with 12 % margin so we don't truncate if the
    // clock runs a bit fast.
    size_t capacity_samples = (size_t)seconds * 100000u;
    size_t capacity_bytes   = capacity_samples * sizeof(int32_t);

    int32_t *buf = heap_caps_malloc(capacity_bytes, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "dump: PSRAM alloc %u bytes failed", (unsigned)capacity_bytes);
        return ESP_ERR_NO_MEM;
    }

    s_dbg_buffer   = buf;
    s_dbg_capacity = capacity_samples;
    s_dbg_write_ix = 0;
    __sync_synchronize();
    s_dbg_active   = true;

    // Wait for the mic task to finish filling the buffer (active
    // clears when write_ix reaches capacity), or bail out after a
    // generous timeout in case something goes wrong.
    int timeout_ms = (seconds + 2) * 1000;
    while (s_dbg_active && timeout_ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
        timeout_ms -= 50;
    }
    s_dbg_active = false;  // belt-and-braces in case we timed out

    size_t written_samples = s_dbg_write_ix;
    size_t frames          = written_samples / 2;  // 2 int32 per stereo frame

    ESP_LOGI(TAG, "dump: captured %u samples (%u frames) → %s",
             (unsigned)written_samples, (unsigned)frames, path);

    esp_err_t result = ESP_OK;
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "dump: fopen %s failed (errno=%d)", path, errno);
        result = ESP_FAIL;
    } else {
        write_wav_header(f, MIC_I2S_RATE_NOM, (uint32_t)frames);
        size_t w = fwrite(buf, sizeof(int32_t), written_samples, f);
        fclose(f);
        if (w != written_samples) {
            ESP_LOGE(TAG, "dump: short write %u/%u", (unsigned)w, (unsigned)written_samples);
            result = ESP_FAIL;
        }
    }

    s_dbg_buffer   = NULL;
    s_dbg_capacity = 0;
    s_dbg_write_ix = 0;
    heap_caps_free(buf);
    return result;
}
