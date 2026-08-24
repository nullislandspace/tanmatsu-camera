#include "microphone.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
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
//   * We configure STEREO explicitly. Two mics share the bus (one
//     strapped L/R=GND, one L/R=VDD), so both slots carry audio and
//     the reader averages them to mono. At 44.1 kHz the DMA stream
//     is 88.2 kHz of interleaved int32 samples.
//
//     STEREO would be the right choice even with a single mic: the
//     MONO slot mask on the P4 RX path does NOT filter the unused
//     slot — we confirmed experimentally that a MONO+LEFT config
//     still delivers both slots interleaved in DMA, with the unused
//     one showing the floating DIN pin (all 0xFFFFFFFF).
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

// Digital gain. The user-facing setting (config menu, VOL+/-) is a
// *step* in [1..8], not a raw multiplier — MIC_GAIN_MULT maps it to
// the actual factor on a ~5 dB ladder spanning +6..+40 dB.
//
// Why a ladder and not a plain 1..8 multiplier: after MIC_SLOT_SHIFT
// (below) a sample is a *correctly scaled* int16, so unity gain is
// unity, and the useful range for a MEMS mic spans far more than 8:1.
// A linear 1..8 knob can't cover that; ~5 dB steps can.
//
// The ladder values are calibrated against a real raw capture rather
// than the datasheet, because the two disagree by ~30 dB. In a dump
// of *quiet* speech into a handheld device, the loudest syllable
// measured -29 dBFS RMS / -19.4 dBFS peak (300 ms long, 9.9 dB crest,
// so speech and not a handling transient) against a -65 dBFS room
// floor. Per the datasheet's -26 dBFS @ 94 dB SPL that would imply
// ~91 dB SPL, which soft speech is not — near-field pickup at a few
// cm accounts for the difference. Trust the measurement.
//
// So step 1 is unity (0 dB), for someone narrating close to the
// device: at unity that dump peaks at -19.4 dBFS, leaving ~19 dB for
// anyone louder than "trying not to annoy co-workers". The top of the
// ladder is +40 dB for a subject several metres away. Steps are
// ~5.7 dB apart. Re-measure with microphone_debug_raw_dump() if the
// mic hardware changes again.
//
// The step is what gets stored in camera.cfg, so the range is
// unchanged from the previous scheme and old config files still load.
#define MIC_GAIN_MIN       1
#define MIC_GAIN_MAX       8
// Step 3 = 4x: the measured quiet-speech dump lands at -7.4 dBFS
// there, which keeps headroom for a normal speaking voice at the
// same distance rather than optimising for the quietest case.
#define MIC_GAIN_DEFAULT   3

static const uint16_t MIC_GAIN_MULT[MIC_GAIN_MAX + 1] = {
    /* unused */ 0,
    /* 1 */   1, /* 2 */   2, /* 3 */   4, /* 4 */   7,
    /* 5 */  14, /* 6 */  27, /* 7 */  52, /* 8 */ 100,
};

// Right-shift that turns a full-scale signed 24-bit slot sample into a
// full-scale int16, i.e. 0 dB. Gain is applied as
// (sample24 * MIC_GAIN_MULT[step]) >> MIC_SLOT_SHIFT, so a step whose
// multiplier is 256 would be exactly the +48 dB that used to be baked
// into the extraction shift (see mic_task_fn) — nothing on the ladder
// goes anywhere near that, which is the whole point.
#define MIC_SLOT_SHIFT     8

// DC-blocking high-pass coefficient, Q30, cascaded twice in
// mic_task_fn for -12 dB/oct. y[n] = a·(y[n-1] + x[n] - x[n-1]) with
// a = exp(-2π × 60 / 44100) ≈ 0.991488, i.e. a 60 Hz corner.
//
// This is NOT optional polish — a measured raw capture from the
// stereo pair had 99.6 % of its total power below 20 Hz, sitting
// ~19 dB ABOVE the voice band, which ate all the headroom the speech
// needed. It is mechanical, not acoustic: the two mics' sub-20 Hz
// content is *anti*-correlated (-0.41) while their voice band
// correlates at +0.986 with a 2-sample inter-mic delay, which is what
// board flex / handling noise looks like when two bottom-port mics
// share a PCB. (The L+R average already cancels ~5 dB of it for free,
// but that is nowhere near enough.)
//
// The INMP441's own high-pass does not help: the datasheet puts its
// corner at -3 dB / 3.7 Hz at 48 kHz (≈3.4 Hz at our 44.1 kHz) and
// only -0.5 dB at 10.4 Hz, so it passes this range essentially
// untouched. 60 Hz is a free choice acoustically — it is exactly the
// -3 dB corner of the INMP441's own MEMS transducer, so nothing the
// mic reproduces faithfully is being discarded.
//
// Q30 rather than the LPF's Q16 because this pole sits very close to
// 1 and coefficient quantisation there shifts the corner a lot. The
// arithmetic shift floors rather than rounds, leaving a systematic
// DC bias below -100 dBFS — far under the part's -87 dBFS noise
// floor, so it does not undo the DC blocking.
#define MIC_HPF_A_Q30      ((int64_t)1064602009)

// One-pole IIR low-pass filter coefficient in Q16.16, cascaded twice
// in mic_task_fn for -12 dB/oct. Its primary job is anti-aliasing for
// the 2:1 decimation to 22.05 kHz (Nyquist 11.025 kHz); it also trims
// the mic's HF self-noise and digital pickup from the ESP32-P4.
//
// Cutoff of ~6 kHz at the 44.1 kHz frame rate:
//   α = 1 - exp(-2π × 6000 / 44100) ≈ 0.5747, Q16 encoded = 37661.
//
// This used to sit at 4 kHz, which was chosen back when the extraction
// stage applied +48 dB of unintended gain and the resulting hiss had
// to be filtered hard. With the gain staging fixed the noise floor is
// ~30 dB lower, so the filter no longer has to cost us the sibilance
// band — 6 kHz keeps speech crisp while still being ~0.9 octaves below
// Nyquist.
#define MIC_LPF_ALPHA_Q16  37661u

static i2s_chan_handle_t    s_rx_handle  = NULL;
static TaskHandle_t         s_task       = NULL;
static StreamBufferHandle_t s_pcm_stream = NULL;
static volatile bool        s_running    = false;
static volatile uint16_t    s_peak_level = 0;
static volatile int         s_gain       = MIC_GAIN_DEFAULT;
static mic_type_t           s_mic_type   = MIC_TYPE_NONE;

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
    // We emit one sample per I2S frame (2 slots → 1 mono sample).
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

    // DC-blocking high-pass state. Each stage needs both the previous
    // input and the previous output (y[n] = a·(y[n-1] + x[n] - x[n-1])),
    // and both must survive across DMA blocks or every block boundary
    // would re-introduce a step. Runs on the 24-bit sample BEFORE the
    // gain stage, so the rumble is removed while there is still
    // headroom to remove it in.
    int32_t  hpf_x1      = 0;
    int32_t  hpf_y1      = 0;
    int32_t  hpf_x2      = 0;
    int32_t  hpf_y2      = 0;

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

        // The DMA stream is I2S STEREO interleaved as L,R,L,R,...
        // Two mics share SCK/WS/SD; the one with L/R=GND drives the
        // LEFT slot, the one with L/R=VDD drives the RIGHT slot, each
        // tri-stating during the other's half-frame. Both slots
        // therefore carry real audio, and we average them into the
        // single mono channel the AVI/Shine pipeline expects. The
        // average is also worth ~3 dB of uncorrelated-noise rejection,
        // and it makes the recording independent of which mic the user
        // happens to be talking into.
        //
        // Slot layout: these are 24-bit MSB-first parts in a 32-bit
        // slot, so the I2S peripheral hands us the sample LEFT-JUSTIFIED
        // — bits [31:8] are the signed 24-bit value, bits [7:0] are the
        // trailing clocks after the mic stops driving. `>> 8` on the
        // signed int32 therefore recovers the true sample, spanning
        // ±2^23 at full scale.
        //
        // *** This is where the old prototype workaround lived. ***
        // The previous code did `(raw[i] >> 8) * gain` and then clamped
        // to int16 — that treats a ±2^23 value as if it were already
        // int16, i.e. it baked in a fixed 256× (+48 dB) before the
        // user's gain even applied. On the half-broken prototype mic,
        // which read roughly 17 dB low, that accidentally landed in a
        // usable range and looked correct. On healthy mics it clips
        // ordinary speech by ~11 dB at the lowest gain setting, which
        // is exactly the "overdriven at minimum gain" symptom. The
        // scaling is now explicit: multiply by the gain-ladder factor,
        // then >> MIC_SLOT_SHIFT to land in int16.
        //
        // After the IIR low-pass (which doubles as the anti-alias
        // filter), a 2:1 decimator emits every other frame, bringing
        // the rate from 44.1 kHz down to the 22.05 kHz that the
        // AVI/Shine pipeline expects. Producer and consumer rates are
        // exactly matched, so no drift over time.
        //
        // Both supported parts (INMP441, ICS43434) are 24-bit I2S MEMS
        // mics with identical Philips framing and sensitivity within a
        // dB of each other, so they share this decode path; s_mic_type
        // is kept for logging and for any future part that doesn't.
        const int32_t gain_mult = (int32_t)MIC_GAIN_MULT[s_gain];
        uint16_t peak   = 0;
        size_t   out_ix = 0;
        for (size_t i = 0; i + 1 < n; i += 2) {
            // Left-justified 24-bit slots → true signed 24-bit samples,
            // averaged down to mono. Sum of two ±2^23 values still fits
            // an int32 with 7 bits to spare.
            int32_t left  = raw[i]     >> 8;
            int32_t right = raw[i + 1] >> 8;
            int32_t mono  = (left + right) / 2;

            // Two cascaded DC blockers (see MIC_HPF_A_Q30). Placed
            // ahead of the gain stage on purpose: the sub-20 Hz
            // rumble is louder than the audio, so amplifying first
            // would clip on rumble long before speech got loud.
            int64_t hacc1 = MIC_HPF_A_Q30 *
                            (int64_t)(hpf_y1 + mono - hpf_x1);
            int32_t h1    = (int32_t)(hacc1 >> 30);
            hpf_x1        = mono;
            hpf_y1        = h1;
            int64_t hacc2 = MIC_HPF_A_Q30 *
                            (int64_t)(hpf_y2 + h1 - hpf_x2);
            int32_t h2    = (int32_t)(hacc2 >> 30);
            hpf_x2        = h1;
            hpf_y2        = h2;

            // Gain + scale to int16. The 64-bit product is deliberate:
            // |h2| can transiently exceed 2^23 on a high-pass
            // overshoot, and 2^24 × 100 would overflow int32.
            int32_t s32 = (int32_t)(((int64_t)h2 * gain_mult) >> MIC_SLOT_SHIFT);
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
            // frame (above) — only the emit step is gated.
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

esp_err_t microphone_start(mic_type_t type) {
    if (type == MIC_TYPE_NONE) return ESP_ERR_INVALID_ARG;
    if (s_running) return ESP_OK;
    s_mic_type = type;

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

    // The INMP441 tri-states SD immediately after its LSB, so the
    // last 8 bit-times of each 32-bit slot are undriven. The datasheet
    // asks for a 100 kOhm pull-DOWN on the shared SD trace to discharge
    // the line while every mic on the bus is tri-stated; without one,
    // those bit-times read back as junk. A raw capture from this board
    // shows exactly that (low byte reading 0x06/0x07/0x0e/0x0f instead
    // of zero), so the resistor is missing — enable the internal
    // pull-down as the software stand-in. Purely hygiene: the `>> 8`
    // in mic_task_fn discards those bits either way. If a real
    // pull-down is ever fitted, this line stays harmless.
    gpio_set_pull_mode(MIC_DIN_GPIO, GPIO_PULLDOWN_ONLY);

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

    const char *type_name = (s_mic_type == MIC_TYPE_INMP441)  ? "INMP441"  :
                            (s_mic_type == MIC_TYPE_ICS43434) ? "ICS43434" :
                                                                "?";
    ESP_LOGI(TAG, "started (%s, I2S%d, %u Hz nominal stereo → L+R mono, SCK=%d WS=%d DIN=%d)",
             type_name, (int)MIC_I2S_PORT, (unsigned)MIC_I2S_RATE_NOM,
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

int microphone_gain_multiplier(int gain) {
    if (gain < MIC_GAIN_MIN) gain = MIC_GAIN_MIN;
    if (gain > MIC_GAIN_MAX) gain = MIC_GAIN_MAX;
    return (int)MIC_GAIN_MULT[gain];
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
