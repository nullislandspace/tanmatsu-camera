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
//     44.1 kHz with 32-bit slots gives BCLK = 44100 × 64 =
//     2.8224 MHz, comfortably in the mic's range.
//
//   * 44.1 kHz is exactly twice the 22.05 kHz rate Shine encodes
//     and the AVI file advertises, so the per-sample work below is
//     a 2:1 decimator instead of a phase-accumulator resampler.
//     No interpolation, no sub-sample state.
//
//   * The MONO slot mask on the P4 RX path does NOT filter the
//     unused slot — we confirmed experimentally that a MONO+LEFT
//     config still delivers both slots interleaved in DMA, with
//     the RIGHT slot showing the floating DIN pin (all 0xFFFFFFFF
//     because the INMP441 tri-states when L/R=GND makes it drive
//     LEFT only). So we configure STEREO explicitly and pick the
//     LEFT samples ourselves; at 44.1 kHz the stream is 88.2 kHz
//     of interleaved int32 samples, half of which carry the mic
//     audio.
//
//   * Trap to avoid: i2s_channel_read() returns ESP_ERR_TIMEOUT
//     with valid PARTIAL bytes already in the destination buffer
//     when its internal per-descriptor xQueueReceive loop times
//     out partway through filling a multi-descriptor request.
//     Treating timeout as a skip-this-iteration error silently
//     drops the partial data. We sidestep this entirely by reading
//     exactly one descriptor's worth per call (see MIC_DMA_FRAMES)
//     so each i2s_channel_read needs only one xQueueReceive — it
//     either succeeds atomically or returns 0 bytes. This was the
//     root cause of a long-standing 6.7-16 % rate-shaped phantom
//     undercount that looked like a clock-divider bug but wasn't.
#define MIC_I2S_RATE_NOM   44100u

// Pin assignments — see microphone.h header comment and
// microphone_inmp441.md for the wiring table.
#define MIC_BCLK_GPIO      GPIO_NUM_54
#define MIC_WS_GPIO        GPIO_NUM_49
#define MIC_DIN_GPIO       GPIO_NUM_53

// How many raw int32 samples we read per DMA pull. **Must equal one
// DMA descriptor's worth of int32 samples** (= dma_frame_num × 2 in
// stereo) — see the partial-read trap note in the file-level
// comment. At 44.1 kHz stereo, 256 frames × 2 int32/frame = 512,
// covering ~5.8 ms per pull.
#define MIC_DMA_FRAMES     512u

// Recorder-side ring buffer. 32 KB holds ~743 ms of 22.05 kHz int16
// mono — generous headroom for any consumer-side hiccups. Producer
// and consumer rates are exactly matched (44.1 kHz ÷ 2 = 22.05 kHz),
// so the buffer doesn't drift in either direction over time.
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
// at the 44.1 kHz frame rate: α = 1 - exp(-2π × 4000 / 44100)
// ≈ 0.434, Q16 encoded = 28464. We cascade the filter twice
// (see mic_task_fn) for a -12 dB/oct rolloff, which also serves as
// the anti-alias filter for the 2:1 decimation (Nyquist at 11.025
// kHz, well above the cutoff so no aliasing).
#define MIC_LPF_ALPHA_Q16  28464u

static i2s_chan_handle_t    s_rx_handle  = NULL;
static TaskHandle_t         s_task       = NULL;
static StreamBufferHandle_t s_pcm_stream = NULL;
static volatile bool        s_running    = false;
static volatile uint16_t    s_peak_level = 0;
static volatile int         s_gain       = MIC_GAIN_DEFAULT;

// Counts I2S RX descriptor overflows. The IDF's RX ISR silently
// discards the oldest queued descriptor (~5.8 ms of audio at 44.1
// kHz) whenever the message queue fills up because the reader
// couldn't drain it in time. Without this counter such losses are
// invisible — they just look like the mic running slow. Read by
// mic_task_fn's 5-second telemetry block.
static volatile uint32_t    s_dma_drops  = 0;

static bool IRAM_ATTR mic_on_q_ovf(i2s_chan_handle_t handle,
                                   i2s_event_data_t *event,
                                   void *user_ctx) {
    (void)handle; (void)event; (void)user_ctx;
    s_dma_drops++;
    return false;
}

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
    // the producer rate matches the I2S clock (~88200 Hz of int32
    // samples in stereo) and surface any DMA-overflow drops.
    uint64_t total_raw_samples     = 0;
    uint64_t total_emitted_samples = 0;
    uint32_t drops_at_window_start = s_dma_drops;
    int64_t  rate_window_start_us  = esp_timer_get_time();

    // Decimator phase: 0 emits the current sample, 1 skips it. With
    // 44.1 kHz in and 22.05 kHz out we toggle every input sample.
    // Stored across blocks so a block ending on an even count
    // doesn't reset the alignment.
    int      decimate_phase = 0;

    // Low-pass filter state — the previous filtered output, so the
    // one-pole IIR y[n] = α·x[n] + (1-α)·y[n-1] survives across
    // blocks. Stored as int32 to keep a little extra headroom for
    // the Q16 accumulator arithmetic. Two stages are cascaded for a
    // -12 dB/oct rolloff; lpf_prev1 is the first stage's output
    // (which also feeds the second stage), lpf_prev2 is the second
    // stage's output. The LPF runs at the full 44.1 kHz rate (i.e.
    // before decimation) so it also serves as the anti-alias.
    int32_t  lpf_prev1   = 0;
    int32_t  lpf_prev2   = 0;

    while (s_running) {
        size_t got_bytes = 0;
        esp_err_t err = i2s_channel_read(s_rx_handle, raw,
                                         MIC_DMA_FRAMES * sizeof(int32_t),
                                         &got_bytes, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            // ESP_ERR_TIMEOUT is benign here: with one descriptor
            // per call the IDF can't return partial bytes, so a
            // timeout just means "no data this round" and we loop.
            // Other errors are unexpected and worth logging.
            if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "i2s read: %d", err);
            }
            continue;
        }
        size_t n = got_bytes / sizeof(int32_t);
        if (n == 0) continue;

        // If a debug raw capture is in progress, copy this block into
        // the PSRAM buffer. We do this before gain/filter/decimate so
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
        // After the IIR low-pass (which doubles as the anti-alias
        // filter), a 2:1 decimator emits every other LEFT sample,
        // bringing the rate from 44.1 kHz down to the 22.05 kHz
        // that the AVI/Shine pipeline expects. Producer and
        // consumer rates are exactly matched, so no drift over time.
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

            // 2:1 decimator. Peak tracking and IIR run on every
            // sample (above) — only the emit step is gated.
            if (decimate_phase == 0) {
                pcm[out_ix++] = x;
                decimate_phase = 1;
            } else {
                decimate_phase = 0;
            }
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
            uint32_t drops  = s_dma_drops - drops_at_window_start;
            ESP_LOGI(TAG, "rate: raw=%u Hz emitted=%u Hz (cfg=%u Hz) drops=%u",
                     (unsigned)raw_hz, (unsigned)out_hz,
                     (unsigned)MIC_I2S_RATE_NOM, (unsigned)drops);
            total_raw_samples = total_emitted_samples = 0;
            drops_at_window_start = s_dma_drops;
            rate_window_start_us  = now_us;
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
    // 16 DMA descriptors × 256 frames at 44.1 kHz stereo = ~93 ms
    // of buffered audio. Heavy headroom: even if mic_task gets
    // preempted by the camera/encoder bursts for 80+ ms, the DMA
    // queue won't overflow and silently drop the oldest descriptor
    // (see mic_on_q_ovf for what happens when it does). In
    // STEREO + 32-bit slots each frame is 8 bytes, so 256 frames
    // per descriptor = 2048 bytes — under the P4 driver's per-
    // descriptor max so the driver doesn't clamp, and the read
    // size below spans whole descriptors per call.
    chan_cfg.dma_desc_num  = 16;
    chan_cfg.dma_frame_num = 256;

    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %d", err);
        return err;
    }

    // 32-bit data + 32-bit slot + STEREO → 64 BCLK per frame, BCLK
    // = 2.8224 MHz at 44.1 kHz (inside INMP441 spec). STEREO is
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

    // Register the queue-overflow callback so descriptor drops show
    // up in the 5-second telemetry instead of looking like the mic
    // running slow. Must happen while the channel is still in INIT
    // state (i.e. before i2s_channel_enable). The callback runs in
    // ISR context — keep it minimal.
    s_dma_drops = 0;
    i2s_event_callbacks_t cbs = { .on_recv_q_ovf = mic_on_q_ovf };
    err = i2s_channel_register_event_callback(s_rx_handle, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "register q_ovf cb: %d (drops will be invisible)", err);
        // Non-fatal: telemetry is degraded but capture still works.
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
    // Priority 7 — above vid_rec/aud_rec (both 6) so the I2S DMA
    // reader is never preempted by the encoders. The per-iteration
    // work (memcpy + IIR×2 + decimate + peak) is well under 1 % of
    // one core, so the encoders never starve.
    BaseType_t ok = xTaskCreate(mic_task_fn, "mic_rx", 4096, NULL, 7, &s_task);
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

    // 88,200 int32 samples per second at 44.1 kHz stereo. Allocate
    // with ~13 % margin so we don't truncate if the clock runs a
    // bit fast or descriptor boundaries push us slightly over.
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
