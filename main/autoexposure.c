#include "autoexposure.h"

#include <inttypes.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/isp_ae.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "autoexposure";

// ---- AE controller + ISR-shared latest stats -------------------------

static isp_ae_ctlr_t s_ae = NULL;

// Updated from the AE ISR every frame, read from the main task in
// autoexposure_tick(). Same discipline as autofocus.c: volatile plus a
// sequence counter, the reader takes a memcpy snapshot and only acts on
// a bumped sequence. A torn read of a stale generation doesn't matter
// because the next frame replaces it wholesale.
static volatile isp_ae_result_t s_latest_stats;
static volatile uint32_t        s_stats_seq;

// ---- Tunables --------------------------------------------------------
//
// These are the numbers to reach for when this loop misbehaves on real
// hardware. Corrections log the measured luminance and the budget they
// moved to, so a few seconds of serial output in a scene you can judge
// by eye is enough to retune AE_TARGET_LUMA.

// Statistics frames between writing the exposure registers and seeing
// the result come back in the metering: one for the sensor to latch the
// new value at its next frame boundary, one for that frame to reach the
// ISP's statistics block.
//
// This is the number that lets the loop run every frame instead of once
// every half second. Each correction is computed against the budget the
// metered frame was actually shot under (see s_eg_pipe), so corrections
// already sent but not yet visible are never counted twice and the loop
// cannot chase its own output.
//
// The two directions of error are NOT symmetric, so this is deliberately
// set high. Over-estimating costs a frame or so of convergence speed and
// nothing else. Under-estimating makes the loop credit itself for a
// correction that has not landed yet, which is the one mistake that
// makes it ring: simulated against true latencies of one to four frames,
// a model of 2 oscillates hard when the truth is 3, while a model of 3
// stays monotone across the whole range.
#define AE_LATENCY_FRAMES   3

// Statistics frames to discard after a write whose effect must be seen
// before the next decision is taken — priming the pipe (enabling the
// loop, replaying onto the sensor after a format switch), and the large
// catch-up step below. Ordinary fine corrections discard nothing; the
// latency model is what covers those.
#define AE_SETTLE_FRAMES    AE_LATENCY_FRAMES

// Target mean luminance, 0..255. Statistics are sampled after demosaic
// and therefore linear (no gamma applied), where an 18% grey card sits
// at ~46. 64 aims about half a stop above that, which keeps typical
// scenes off the noise floor without riding the highlights. This is the
// single most likely thing to want changing after looking at real
// frames.
#define AE_TARGET_LUMA      64

// Fine tracking closes three eighths of the remaining error in *stops*
// per frame. Working in the log domain is what makes the motion look
// smooth: the step is proportional to the error the eye actually
// perceives, so the correction tapers off as it arrives instead of
// stopping dead. Both weights are dyadic so geo_step() can build them
// out of nothing but integer square roots.
//
// Beyond AE_FAST_RATIO the scene has changed outright (lights switched
// on, panning onto a window) and the frame is unusable during the
// transition anyway. There the loop closes three quarters of the error
// in one go and then waits AE_SETTLE_FRAMES to see where that landed,
// rather than stacking more large steps on top of a measurement that
// cannot yet reflect the first one.
#define AE_FAST_RATIO       3

// Convergence band, as a percentage of the *budget*, with hysteresis:
// the loop stops correcting inside AE_LOCK_PCT and does not start again
// until the error exceeds AE_WAKE_PCT. Narrower than the old luminance
// deadband could safely be, because damping — not a wide dead zone — is
// now what stops the picture dithering on sensor noise. The gap between
// the two keeps slow drift and HUD state from flickering.
#define AE_LOCK_PCT         6
#define AE_WAKE_PCT         12

// Don't bother the sensor over I2C for a correction this small; below
// about this the exposure register quantisation eats it anyway. Kept
// well under AE_LOCK_PCT so the loop can always still reach lock.
#define AE_MIN_WRITE_PCT    2

// Largest correction per step, as a shift. 3 = 8x = three stops. With
// log-domain damping the step is naturally bounded, so this is a sanity
// rail against a wild measurement rather than the main defence — set
// wide enough not to be what limits the catch-up step above.
#define AE_MAX_STEP_SHIFT   3

// A block at or above this is clipped, and its true luminance is
// unknowable — the mean it contributes to is an underestimate. When
// enough of the frame is clipped, meter down regardless of what the
// mean says, or a bright window in shot leaves the subject blown out.
#define AE_CLIP_LUMA        250
#define AE_CLIP_BLOCKS_MAX  2

// Serial output only. The loop now corrects at frame rate, so logging
// every step would drown the console.
#define AE_LOG_INTERVAL_MS  1000

// Centre-weighted metering over the 5x5 block grid. A flat average
// blows out any scene containing a window; weighting the middle is the
// cheapest defence that still behaves sensibly when the subject is not
// centred.
static const uint8_t AE_WEIGHTS[ISP_AE_BLOCK_X_NUM][ISP_AE_BLOCK_Y_NUM] = {
    { 1, 1, 1, 1, 1 },
    { 1, 2, 2, 2, 1 },
    { 1, 2, 4, 2, 1 },
    { 1, 2, 2, 2, 1 },
    { 1, 1, 1, 1, 1 },
};

// ---- Loop state ------------------------------------------------------

static bool     s_enabled       = false;
static uint32_t s_eg_us         = 0;   // current light budget (0 = not seeded)
static uint32_t s_last_seen_seq = 0;
static int      s_settle_frames = 0;
static int      s_last_luma     = 0;
static bool     s_at_limit      = false;
static bool     s_locked        = false;
static int64_t  s_next_log_us   = 0;
static camera_exposure_t s_last_exposure = {0};

// Budgets that have been programmed but whose effect has not yet come
// back through the statistics. Index 0 is the oldest — the budget the
// frame being metered right now was shot under.
static uint32_t s_eg_pipe[AE_LATENCY_FRAMES];

// ---- ISR callback ----------------------------------------------------

static bool IRAM_ATTR on_stats_done(isp_ae_ctlr_t ae,
                                    const esp_isp_ae_env_detector_evt_data_t *edata,
                                    void *user_data) {
    memcpy((void *)&s_latest_stats, &edata->ae_result, sizeof(isp_ae_result_t));
    s_stats_seq++;
    return false;
}

// ---- Helpers ---------------------------------------------------------

static void pipe_prime(uint32_t eg) {
    for (int i = 0; i < AE_LATENCY_FRAMES; i++) s_eg_pipe[i] = eg;
}

// Pop the budget the frame now being metered was shot under, push the
// budget currently programmed. Called once per statistics frame, which
// is what keeps the pipe exactly AE_LATENCY_FRAMES deep.
static uint32_t pipe_advance(uint32_t eg_now) {
    const uint32_t out = s_eg_pipe[0];
    for (int i = 0; i + 1 < AE_LATENCY_FRAMES; i++) s_eg_pipe[i] = s_eg_pipe[i + 1];
    s_eg_pipe[AE_LATENCY_FRAMES - 1] = eg_now;
    return out ? out : eg_now;
}

static uint32_t isqrt64(uint64_t v) {
    if (v == 0) return 0;
    // Start above the root so Newton descends monotonically.
    uint64_t x = 1ULL << ((64 - __builtin_clzll(v) + 1) / 2);
    for (int i = 0; i < 8; i++) {
        const uint64_t nx = (x + v / x) / 2;
        if (nx >= x) break;
        x = nx;
    }
    return (uint32_t)x;
}

// Blend the current budget toward the ideal one along the log axis, so
// a step of a given size looks the same whether the frame is dark or
// bright. Every weight here is a dyadic fraction, which means each one
// is just a nest of geometric means: sqrt(a*b) is the halfway point in
// stops, and feeding that back in halves the distance again.
static uint32_t geo_step(uint32_t cur, uint64_t ideal, bool fast) {
    const uint64_t half = isqrt64((uint64_t)cur * ideal);        // cur^1/2 ideal^1/2
    uint64_t m;
    if (fast) {
        m = isqrt64(half * ideal);                               // cur^1/4 ideal^3/4
    } else {
        const uint64_t qtr = isqrt64((uint64_t)cur * half);      // cur^3/4 ideal^1/4
        m = isqrt64(half * qtr);                                 // cur^5/8 ideal^3/8
    }
    if (m == 0) m = 1;
    return (uint32_t)m;
}

// ---- Public API ------------------------------------------------------

esp_err_t autoexposure_init(isp_proc_handle_t isp,
                            uint16_t input_w, uint16_t input_h) {
    if (s_ae) {
        ESP_LOGW(TAG, "already initialised");
        return ESP_OK;
    }

    // Sample the whole frame; the 5x5 block grid subdivides whatever
    // window it is given, and the weighting above is what decides where
    // the emphasis lands. Restricting the window here instead would
    // throw away the outer blocks entirely, leaving nothing to notice
    // a bright surround with.
    esp_isp_ae_config_t cfg = {
        // After demosaic rather than after gamma: the gamma block is
        // never configured in this pipeline, so its output is not
        // something to meter against.
        .sample_point           = ISP_AE_SAMPLE_POINT_AFTER_DEMOSAIC,
        .window.top_left.x      = 0,
        .window.top_left.y      = 0,
        .window.btm_right.x     = input_w - 1,
        .window.btm_right.y     = input_h - 1,
        .intr_priority          = 0,
    };

    esp_err_t err = esp_isp_new_ae_controller(isp, &cfg, &s_ae);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_isp_new_ae_controller: %d", err);
        s_ae = NULL;
        return err;
    }

    esp_isp_ae_env_detector_evt_cbs_t cbs = {
        .on_env_statistics_done = on_stats_done,
    };
    err = esp_isp_ae_env_detector_register_event_callbacks(s_ae, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register cb: %d", err);
        goto fail;
    }

    err = esp_isp_ae_controller_enable(s_ae);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "enable: %d", err);
        goto fail;
    }

    err = esp_isp_ae_controller_start_continuous_statistics(s_ae);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start_continuous: %d", err);
        esp_isp_ae_controller_disable(s_ae);
        goto fail;
    }

    // Re-arm after a pipeline rebuild (PHOTO<->VIDEO tears the ISP down
    // and back up). s_enabled carries the user's intent across; s_eg_us
    // carries the exposure the loop had settled on, so the new mode
    // starts where the old one left off instead of re-hunting.
    s_last_seen_seq = s_stats_seq;
    s_settle_frames = AE_SETTLE_FRAMES;
    pipe_prime(s_eg_us);

    ESP_LOGI(TAG, "ready, window=%ux%u, %dx%d blocks, enabled=%d",
             (unsigned)input_w, (unsigned)input_h,
             ISP_AE_BLOCK_X_NUM, ISP_AE_BLOCK_Y_NUM, (int)s_enabled);
    return ESP_OK;

fail:
    esp_isp_del_ae_controller(s_ae);
    s_ae = NULL;
    return err;
}

void autoexposure_shutdown(void) {
    if (!s_ae) return;
    esp_isp_ae_controller_stop_continuous_statistics(s_ae);
    esp_isp_ae_controller_disable(s_ae);
    esp_isp_del_ae_controller(s_ae);
    s_ae = NULL;
    // s_enabled and s_eg_us survive on purpose — see the re-arm comment
    // in init().
}

bool autoexposure_available(void) {
    return s_ae != NULL;
}

bool autoexposure_is_enabled(void) {
    return s_enabled;
}

void autoexposure_set_enabled(bool enabled) {
    if (s_enabled == enabled) return;
    s_enabled = enabled;
    if (enabled) {
        // Start measuring immediately rather than waiting out an
        // interval — switching to AUTO should visibly do something.
        s_settle_frames = AE_SETTLE_FRAMES;
        s_last_seen_seq = s_stats_seq;
        s_locked        = false;
        s_at_limit      = false;
        pipe_prime(s_eg_us);
    }
    ESP_LOGI(TAG, "%s", enabled ? "enabled" : "disabled");
}

uint32_t autoexposure_current_eg_us(void) {
    return s_eg_us;
}

void autoexposure_last_exposure(camera_exposure_t *out) {
    if (out) *out = s_last_exposure;
}

ae_hud_state_t autoexposure_hud_state(void) {
    if (!s_enabled || !s_ae) return AE_HUD_OFF;
    if (s_locked)             return AE_HUD_LOCKED;
    if (s_at_limit)           return AE_HUD_LIMIT;
    return AE_HUD_HUNTING;
}

int autoexposure_hud_luma(void) {
    return s_last_luma;
}

// Seed the loop's working budget from whatever is actually in the
// sensor's registers right now, so enabling AE (or coming back from a
// format switch) starts from the current picture rather than from a
// guess.
static void seed_from_sensor(camera_sensor_t *sensor) {
    camera_exposure_t cur = {0};
    if (camera_sensor_read_exposure(sensor, &cur) == ESP_OK && cur.exposure_us > 0) {
        uint32_t gain = cur.gain_q4 ? cur.gain_q4 : 16u;
        s_eg_us = (uint32_t)(((uint64_t)cur.exposure_us * gain) / 16ULL);
        // Also the HUD's starting readout, so a loop that lands inside
        // the deadband on its very first sample still has something
        // truthful to show instead of staying blank.
        s_last_exposure = cur;
    }
    if (s_eg_us == 0) {
        // Read-back failed. Start in the middle of the sensor's range
        // rather than at an extreme — from there the loop reaches
        // either end in a couple of corrections.
        uint32_t lo = camera_sensor_eg_min_us(sensor);
        uint32_t hi = camera_sensor_eg_max_us(sensor);
        s_eg_us = lo + (hi - lo) / 8u;
    }
    pipe_prime(s_eg_us);
}

void autoexposure_reapply(camera_sensor_t *sensor) {
    if (!sensor) return;
    if (s_eg_us == 0) seed_from_sensor(sensor);
    camera_sensor_set_exposure_eg(sensor, s_eg_us, &s_last_exposure);
    s_settle_frames = AE_SETTLE_FRAMES;
    s_last_seen_seq = s_stats_seq;
    pipe_prime(s_eg_us);
}

void autoexposure_tick(camera_sensor_t *sensor) {
    if (!s_ae || !s_enabled || sensor == NULL) return;

    // 1. Fresh statistics only. This is the loop's clock: one correction
    //    per statistics frame, no wall-clock rate limit. Stability comes
    //    from the latency model plus damping, not from running rarely.
    const uint32_t seq = s_stats_seq;
    if (seq == s_last_seen_seq) return;
    uint32_t missed = seq - s_last_seen_seq;
    s_last_seen_seq = seq;

    if (s_eg_us == 0) seed_from_sensor(sensor);

    // 2. Advance the in-flight pipe and recover the budget the frame we
    //    are about to meter was actually shot under. If the main loop
    //    fell behind and whole statistics frames went by unread, advance
    //    by that many so the pipe stays aligned with reality.
    if (missed > AE_LATENCY_FRAMES) missed = AE_LATENCY_FRAMES;
    uint32_t eg_meas = s_eg_us;
    for (uint32_t i = 0; i < missed; i++) eg_meas = pipe_advance(s_eg_us);

    // 3. Skip samples that cannot yet reflect a write we are waiting on:
    //    a priming write, or a large catch-up step.
    if (s_settle_frames > 0) {
        s_settle_frames--;
        return;
    }

    // 4. Snapshot and meter.
    isp_ae_result_t stats;
    memcpy(&stats, (const void *)&s_latest_stats, sizeof(stats));

    uint32_t acc = 0, wsum = 0;
    int clipped = 0;
    for (int x = 0; x < ISP_AE_BLOCK_X_NUM; x++) {
        for (int y = 0; y < ISP_AE_BLOCK_Y_NUM; y++) {
            int lum = stats.luminance[x][y];
            if (lum < 0)   lum = 0;
            if (lum > 255) lum = 255;
            if (lum >= AE_CLIP_LUMA) clipped++;
            const uint32_t w = AE_WEIGHTS[x][y];
            acc  += (uint32_t)lum * w;
            wsum += w;
        }
    }
    if (wsum == 0) return;

    uint32_t luma = acc / wsum;
    if (luma == 0) luma = 1;  // guard the divide below
    s_last_luma   = (int)luma;

    // 5. The budget that WOULD have produced the target luminance for
    //    the frame just metered. Luminance is linear in the budget while
    //    nothing is clipped, so this is just the ratio — and taking it
    //    against eg_meas rather than the current budget is precisely
    //    what lets the loop run every frame: corrections still in flight
    //    are already accounted for and never applied twice.
    uint64_t ideal = ((uint64_t)eg_meas * AE_TARGET_LUMA) / luma;
    if (ideal == 0) ideal = 1;

    // A frame whose mean is on target but which has several blown
    // blocks is not correctly exposed — the clipped blocks are capped at
    // 255 and drag the mean *down* relative to the light actually
    // arriving, so the mean alone cannot see the problem.
    const bool overexposed_by_clipping = (clipped > AE_CLIP_BLOCKS_MAX);
    if (overexposed_by_clipping && ideal >= s_eg_us) {
        ideal = (uint64_t)s_eg_us * 2 / 3;
    }

    // 6. Convergence test, in the budget domain rather than the
    //    luminance domain: with corrections in flight the picture can
    //    still be off target while the loop has already done everything
    //    it needs to, and locking on luminance would make it correct
    //    twice and then walk back.
    const uint64_t err_pct = (ideal > s_eg_us)
                           ? ((ideal - s_eg_us) * 100u) / s_eg_us
                           : ((s_eg_us - ideal) * 100u) / s_eg_us;
    const uint32_t band = s_locked ? AE_WAKE_PCT : AE_LOCK_PCT;

    if (err_pct <= band && !overexposed_by_clipping) {
        s_locked   = true;
        s_at_limit = false;
        return;
    }
    s_locked = false;

    // 7. Step toward it, damped in the log domain.
    const bool fast = (ideal > (uint64_t)s_eg_us * AE_FAST_RATIO) ||
                      (ideal * AE_FAST_RATIO < (uint64_t)s_eg_us);
    uint64_t want = geo_step(s_eg_us, ideal, fast);

    // Sanity rail against a wild measurement.
    const uint64_t ceil_step  = (uint64_t)s_eg_us << AE_MAX_STEP_SHIFT;
    const uint64_t floor_step = (uint64_t)s_eg_us >> AE_MAX_STEP_SHIFT;
    if (want > ceil_step)  want = ceil_step;
    if (want < floor_step) want = floor_step;

    const uint32_t eg_lo = camera_sensor_eg_min_us(sensor);
    const uint32_t eg_hi = camera_sensor_eg_max_us(sensor);
    if (want < eg_lo) want = eg_lo;
    if (want > eg_hi) want = eg_hi;

    // The scene is beyond what this sensor and frame rate can reach.
    // Say so in the HUD rather than sitting there looking converged.
    s_at_limit = (ideal > eg_hi) || (ideal < eg_lo);

    // Too small to be worth an I2C round trip, and below the exposure
    // register quantisation anyway. AE_MIN_WRITE_PCT sits well under
    // AE_LOCK_PCT, so this can never stall the loop short of lock.
    const uint64_t delta = (want > s_eg_us) ? (want - s_eg_us) : (s_eg_us - want);
    if (delta * 100u < (uint64_t)s_eg_us * AE_MIN_WRITE_PCT) return;

    const uint32_t prev = s_eg_us;
    s_eg_us = (uint32_t)want;

    camera_exposure_t got = {0};
    esp_err_t err = camera_sensor_set_exposure_eg(sensor, s_eg_us, &got);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "set_exposure_eg(%" PRIu32 "): %d", s_eg_us, err);
        s_eg_us = prev;
        return;
    }
    s_last_exposure = got;
    // The write lands in the pipe on the next statistics frame; the
    // entries already queued keep describing the frames still in flight.
    s_eg_pipe[AE_LATENCY_FRAMES - 1] = s_eg_us;
    // A catch-up step is far too large to stack another one on top of
    // before seeing where it landed. Fine corrections need no such gate
    // — that is what the latency model is for.
    if (fast) s_settle_frames = AE_SETTLE_FRAMES;

    const int64_t now = esp_timer_get_time();
    if (now >= s_next_log_us) {
        s_next_log_us = now + (int64_t)AE_LOG_INTERVAL_MS * 1000;
        ESP_LOGI(TAG, "luma %" PRIu32 " (target %d, %d clipped) -> eg %" PRIu32
                      " us = %" PRIu32 " us x %u.%02u",
                 luma, AE_TARGET_LUMA, clipped, s_eg_us, got.exposure_us,
                 (unsigned)(got.gain_q4 / 16u), (unsigned)((got.gain_q4 % 16u) * 100u / 16u));
    }
}
