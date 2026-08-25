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
// hardware. Every correction logs the measured luminance and the budget
// it moved to, so a few seconds of serial output in a scene you can
// judge by eye is enough to retune AE_TARGET_LUMA.

// One correction every half second. This is the whole stability
// argument: an exposure write lands in the statistics one to two frames
// later, which at 15 fps is 66-133 ms. Sampling four to eight times
// slower than the dead time means the loop never sees its own stale
// output, so it needs no damping term to stay stable. The cost is that
// it takes a couple of seconds to walk across a big lighting change,
// which is the trade this first stage deliberately accepts.
#define AE_INTERVAL_MS      500

// Discard this many frames of statistics after writing new registers,
// so a correction is never metered against the exposure it replaced.
// Belt-and-braces given the interval above already covers it.
#define AE_SETTLE_FRAMES    3

// Target mean luminance, 0..255. Statistics are sampled after demosaic
// and therefore linear (no gamma applied), where an 18% grey card sits
// at ~46. 64 aims about half a stop above that, which keeps typical
// scenes off the noise floor without riding the highlights. This is the
// single most likely thing to want changing after looking at real
// frames.
#define AE_TARGET_LUMA      64

// Deadband, as a percentage of the target, either side. Wide on
// purpose: below about a third of a stop the loop would be chasing
// sensor noise and the picture would visibly dither.
#define AE_DEADBAND_PCT     25

// Largest correction per step, as a shift. 2 = 4x = two stops. Bounds
// how far a single bad measurement (a hand passing the lens, a flash)
// can throw the exposure before the next sample corrects it.
#define AE_MAX_STEP_SHIFT   2

// A block at or above this is clipped, and its true luminance is
// unknowable — the mean it contributes to is an underestimate. When
// enough of the frame is clipped, meter down regardless of what the
// mean says, or a bright window in shot leaves the subject blown out.
#define AE_CLIP_LUMA        250
#define AE_CLIP_BLOCKS_MAX  2

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
static int64_t  s_next_run_us   = 0;
static int      s_last_luma     = 0;
static bool     s_at_limit      = false;
static bool     s_in_deadband   = false;
static camera_exposure_t s_last_exposure = {0};

// ---- ISR callback ----------------------------------------------------

static bool IRAM_ATTR on_stats_done(isp_ae_ctlr_t ae,
                                    const esp_isp_ae_env_detector_evt_data_t *edata,
                                    void *user_data) {
    memcpy((void *)&s_latest_stats, &edata->ae_result, sizeof(isp_ae_result_t));
    s_stats_seq++;
    return false;
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
    s_next_run_us   = 0;

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
        s_next_run_us   = 0;
        s_settle_frames = 0;
        s_last_seen_seq = s_stats_seq;
        s_in_deadband   = false;
        s_at_limit      = false;
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
    if (s_in_deadband)        return AE_HUD_LOCKED;
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
}

void autoexposure_reapply(camera_sensor_t *sensor) {
    if (!sensor) return;
    if (s_eg_us == 0) seed_from_sensor(sensor);
    camera_sensor_set_exposure_eg(sensor, s_eg_us, &s_last_exposure);
    s_settle_frames = AE_SETTLE_FRAMES;
    s_last_seen_seq = s_stats_seq;
}

void autoexposure_tick(camera_sensor_t *sensor) {
    if (!s_ae || !s_enabled || sensor == NULL) return;

    // 1. Rate limit. The interval, not a damping constant, is what
    //    keeps this loop stable.
    const int64_t now = esp_timer_get_time();
    if (now < s_next_run_us) return;

    // 2. Fresh statistics only.
    const uint32_t seq = s_stats_seq;
    if (seq == s_last_seen_seq) return;
    s_last_seen_seq = seq;

    // 3. Skip samples that may predate our last write.
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

    s_next_run_us = now + (int64_t)AE_INTERVAL_MS * 1000;

    if (s_eg_us == 0) seed_from_sensor(sensor);

    // 5. Deadband, with the clipping override. A frame whose mean is on
    //    target but which has several blown blocks is not correctly
    //    exposed — the clipped blocks are capped at 255 and are
    //    dragging the mean *down* relative to the light actually
    //    arriving, so the mean cannot see the problem.
    const uint32_t lo = (uint32_t)AE_TARGET_LUMA * (100 - AE_DEADBAND_PCT) / 100;
    const uint32_t hi = (uint32_t)AE_TARGET_LUMA * (100 + AE_DEADBAND_PCT) / 100;
    const bool overexposed_by_clipping = (clipped > AE_CLIP_BLOCKS_MAX);

    if (luma >= lo && luma <= hi && !overexposed_by_clipping) {
        s_in_deadband = true;
        s_at_limit    = false;
        return;
    }
    s_in_deadband = false;

    // 6. New budget. Luminance is linear in exposure while nothing is
    //    clipped, so the correction is just the ratio. When the frame
    //    IS clipped the measured luma understates the error and the
    //    ratio comes out too small — that is fine and in fact the safe
    //    direction, it simply converges from above over a few steps.
    uint64_t want = ((uint64_t)s_eg_us * (uint64_t)AE_TARGET_LUMA) / luma;
    if (overexposed_by_clipping && want >= s_eg_us) {
        // Mean says "fine", clipping says otherwise. Force a step down.
        want = (uint64_t)s_eg_us * 2 / 3;
    }

    // Bound a single correction, so one bad frame cannot swing the
    // exposure across the whole range.
    const uint64_t ceil_step = (uint64_t)s_eg_us << AE_MAX_STEP_SHIFT;
    const uint64_t floor_step = (uint64_t)s_eg_us >> AE_MAX_STEP_SHIFT;
    if (want > ceil_step)  want = ceil_step;
    if (want < floor_step) want = floor_step;

    const uint32_t eg_lo = camera_sensor_eg_min_us(sensor);
    const uint32_t eg_hi = camera_sensor_eg_max_us(sensor);
    if (want < eg_lo) want = eg_lo;
    if (want > eg_hi) want = eg_hi;

    // Ran out of sensor range: the scene is beyond what this exposure
    // can reach. Say so in the HUD rather than silently sitting there
    // looking like a converged loop.
    s_at_limit = (want == s_eg_us) ||
                 (want == eg_lo && luma > hi) ||
                 (want == eg_hi && luma < lo);
    if (want == s_eg_us) return;

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
    s_settle_frames = AE_SETTLE_FRAMES;

    ESP_LOGI(TAG, "luma %" PRIu32 " (target %d, %d clipped) -> eg %" PRIu32
                  " us = %" PRIu32 " us x %u.%02u",
             luma, AE_TARGET_LUMA, clipped, s_eg_us, got.exposure_us,
             (unsigned)(got.gain_q4 / 16u), (unsigned)((got.gain_q4 % 16u) * 100u / 16u));
}
