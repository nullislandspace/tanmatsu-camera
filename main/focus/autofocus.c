#include "autofocus.h"

#include <string.h>
#include "esp_log.h"
#include "driver/isp_af.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include "focus_driver.h"

static const char *TAG = "autofocus";

// ---- AF controller + ISR-shared latest stats -------------------------

static isp_af_ctlr_t s_af = NULL;

// Updated from the AF ISR every frame. Read from the main task in
// autofocus_tick(). Volatile + a sequence counter is enough: the
// reader uses memcpy snapshots and only acts on a bumped sequence,
// and we don't care about partial torn reads of a stale generation
// because the next frame replaces it.
static volatile isp_af_result_t s_latest_stats;
static volatile uint32_t        s_stats_seq;

// ---- Tunables --------------------------------------------------------

#define AF_SETTLE_FRAMES    3   // wait for VCM mechanical settle
#define AF_COARSE_STEP     64   // 16 samples across 0..1023
#define AF_FINE_STEP        8   // 16 samples across ±64 of best coarse pos
#define AF_FINE_SAMPLES     8   // total fine samples per pass
#define AF_LOCK_RECHECK    30   // ~2 s @ 15 fps before drift check
#define AF_LOCK_DROP_PCT   70   // re-acquire if def < 70 % of locked def

// ---- State machine ---------------------------------------------------

typedef enum {
    AF_IDLE,
    AF_COARSE,
    AF_FINE,
    AF_LOCKED,
    AF_OVERRIDE,
} af_state_t;

static af_state_t s_state           = AF_IDLE;
static bool       s_enabled         = false;
static bool       s_override        = false;

static int        s_settle_frames   = 0;
static uint32_t   s_last_seen_seq   = 0;

static int        s_best_def        = 0;
static uint16_t   s_best_pos        = 0;
static uint16_t   s_fine_origin     = 0;

static int        s_locked_def      = 0;
static int        s_lock_age_frames = 0;

static int        s_last_def        = 0;  // for HUD only

// ---- ISR callback ----------------------------------------------------

static bool IRAM_ATTR on_stats_done(isp_af_ctlr_t af, const esp_isp_af_env_detector_evt_data_t *edata, void *user_data) {
    memcpy((void *)&s_latest_stats, &edata->af_result, sizeof(isp_af_result_t));
    s_stats_seq++;
    return false;
}

// ---- Public API ------------------------------------------------------

esp_err_t autofocus_init(isp_proc_handle_t isp, uint16_t input_w, uint16_t input_h) {
    if (s_af) {
        ESP_LOGW(TAG, "already initialised");
        return ESP_OK;
    }

    // Center window 1/3 × 1/3 of the input frame. All three hardware
    // windows are populated with the same rect; we only consume
    // definition[0]. Future work: weighted multi-window.
    uint16_t w = input_w / 3;
    uint16_t h = input_h / 3;
    uint16_t x = (input_w - w) / 2;
    uint16_t y = (input_h - h) / 2;

    esp_isp_af_config_t cfg = {
        .edge_thresh = 128,
    };
    for (int i = 0; i < ISP_AF_WINDOW_NUM; i++) {
        cfg.window[i].top_left.x  = x;
        cfg.window[i].top_left.y  = y;
        cfg.window[i].btm_right.x = x + w - 1;
        cfg.window[i].btm_right.y = y + h - 1;
    }

    esp_err_t err = esp_isp_new_af_controller(isp, &cfg, &s_af);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_isp_new_af_controller: %d", err);
        s_af = NULL;
        return err;
    }

    esp_isp_af_env_detector_evt_cbs_t cbs = {
        .on_env_statistics_done = on_stats_done,
    };
    err = esp_isp_af_env_detector_register_event_callbacks(s_af, &cbs, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register cb: %d", err);
        goto fail;
    }

    err = esp_isp_af_controller_enable(s_af);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "enable: %d", err);
        goto fail;
    }

    err = esp_isp_af_controller_start_continuous_statistics(s_af);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start_continuous: %d", err);
        esp_isp_af_controller_disable(s_af);
        goto fail;
    }

    // Re-arm if the user had enabled AF before a previous teardown
    // (e.g. PHOTO↔VIDEO mode switch tears the ISP down + back up via
    // camera_preview_stop/start, but the user's intent persists).
    if (s_enabled) {
        s_state         = AF_COARSE;
        s_best_def      = 0;
        s_best_pos      = 0;
        s_settle_frames = AF_SETTLE_FRAMES;
    } else {
        s_state         = AF_IDLE;
        s_settle_frames = 0;
    }
    s_last_seen_seq = s_stats_seq;
    s_last_def      = 0;
    ESP_LOGI(TAG, "ready, window=(%u,%u)..(%u,%u), enabled=%d",
             (unsigned)x, (unsigned)y,
             (unsigned)(x + w - 1), (unsigned)(y + h - 1),
             (int)s_enabled);
    return ESP_OK;

fail:
    esp_isp_del_af_controller(s_af);
    s_af = NULL;
    return err;
}

void autofocus_shutdown(void) {
    if (!s_af) return;
    esp_isp_af_controller_stop_continuous_statistics(s_af);
    esp_isp_af_controller_disable(s_af);
    esp_isp_del_af_controller(s_af);
    s_af    = NULL;
    s_state = AF_IDLE;
    // Deliberately keep s_enabled across shutdown so a PHOTO↔VIDEO
    // mode switch (which recreates the ISP via camera_preview_stop +
    // start) doesn't silently drop AF. Re-init picks the user's
    // intent back up.
    s_override = false;
}

void autofocus_set_enabled(bool enabled) {
    if (s_enabled == enabled) return;
    s_enabled = enabled;
    if (enabled) {
        // Start a fresh search from wherever the lens currently is.
        s_state         = AF_COARSE;
        s_best_def      = 0;
        s_best_pos      = 0;
        s_settle_frames = AF_SETTLE_FRAMES;
        s_last_seen_seq = s_stats_seq;
        ESP_LOGI(TAG, "enabled");
    } else {
        s_state = AF_IDLE;
        ESP_LOGI(TAG, "disabled");
    }
}

void autofocus_set_manual_override(bool active) {
    if (s_override == active) return;
    s_override = active;
    if (active) {
        ESP_LOGD(TAG, "manual override on");
    } else {
        ESP_LOGD(TAG, "manual override off — re-converging");
    }
}

af_hud_state_t autofocus_hud_state(void) {
    if (!s_enabled) return AF_HUD_OFF;
    if (s_override) return AF_HUD_OVERRIDE;
    if (s_state == AF_LOCKED) return AF_HUD_LOCK;
    if (s_state == AF_IDLE)   return AF_HUD_OFF;
    return AF_HUD_SEARCH;
}

int autofocus_hud_sharpness(void) {
    return s_last_def;
}

// ---- Tick ------------------------------------------------------------

void autofocus_tick(uint16_t *focus_pos) {
    if (!s_af) return;

    const focus_driver_t *fd = focus_active();
    if (!fd) return;

    // 1. Override / disabled gating.
    if (!s_enabled) {
        s_state = AF_IDLE;
        return;
    }
    if (s_override) {
        // Note that we'll re-enter from AF_OVERRIDE on the next call
        // when override goes false (handled below).
        s_state = AF_OVERRIDE;
        return;
    }

    // 2. Just exiting OVERRIDE → kick a fine search around current pos.
    if (s_state == AF_OVERRIDE) {
        s_state         = AF_FINE;
        s_best_def      = 0;
        s_best_pos      = *focus_pos;
        s_fine_origin   = (*focus_pos > AF_FINE_STEP * (AF_FINE_SAMPLES / 2))
                          ? (uint16_t)(*focus_pos - AF_FINE_STEP * (AF_FINE_SAMPLES / 2))
                          : fd->pos_min;
        s_settle_frames = AF_SETTLE_FRAMES;
        s_last_seen_seq = s_stats_seq;
        // Park the lens at the start of the fine sweep.
        focus_set_position(s_fine_origin);
        *focus_pos = s_fine_origin;
        return;
    }

    // 3. New stats?
    uint32_t seq = s_stats_seq;
    if (seq == s_last_seen_seq) return;
    s_last_seen_seq = seq;

    // 4. VCM still settling — skip this sample.
    if (s_settle_frames > 0) {
        s_settle_frames--;
        return;
    }

    // 5. Read the latest sample (single window).
    int def = s_latest_stats.definition[0];
    s_last_def = def;

    // 6. Track best-so-far on the current sweep.
    if (def > s_best_def) {
        s_best_def = def;
        s_best_pos = *focus_pos;
    }

    // 7. State-specific transition.
    uint16_t cur  = *focus_pos;
    uint16_t next = cur;
    bool     move = false;

    switch (s_state) {
    case AF_COARSE: {
        int32_t np = (int32_t)cur + AF_COARSE_STEP;
        if (np > fd->pos_max) {
            // Sweep done → switch to fine around best, parked at start.
            int32_t origin = (int32_t)s_best_pos - AF_FINE_STEP * (AF_FINE_SAMPLES / 2);
            if (origin < fd->pos_min) origin = fd->pos_min;
            s_fine_origin   = (uint16_t)origin;
            next            = s_fine_origin;
            move            = true;
            s_state         = AF_FINE;
            s_best_def      = 0;
            s_best_pos      = s_fine_origin;
        } else {
            next = (uint16_t)np;
            move = true;
        }
        break;
    }
    case AF_FINE: {
        int32_t np  = (int32_t)cur + AF_FINE_STEP;
        int32_t end = (int32_t)s_fine_origin + AF_FINE_STEP * AF_FINE_SAMPLES;
        if (np > end || np > fd->pos_max) {
            // Done — park at peak and lock.
            next              = s_best_pos;
            move              = (next != cur);
            s_state           = AF_LOCKED;
            s_locked_def      = s_best_def;
            s_lock_age_frames = 0;
            ESP_LOGI(TAG, "locked at pos=%u def=%d", (unsigned)s_best_pos, s_best_def);
        } else {
            next = (uint16_t)np;
            move = true;
        }
        break;
    }
    case AF_LOCKED:
        s_lock_age_frames++;
        if (s_lock_age_frames >= AF_LOCK_RECHECK) {
            int threshold = (s_locked_def * AF_LOCK_DROP_PCT) / 100;
            if (def < threshold) {
                ESP_LOGI(TAG, "drift: def=%d < %d (%d%% of %d) → re-acquire",
                         def, threshold, AF_LOCK_DROP_PCT, s_locked_def);
                s_state         = AF_COARSE;
                s_best_def      = 0;
                s_best_pos      = cur;
                s_settle_frames = 0;
            } else {
                s_lock_age_frames = 0;  // reset window
            }
        }
        break;
    case AF_IDLE:
    case AF_OVERRIDE:
        // Handled above.
        break;
    }

    if (move && next != cur) {
        if (next < fd->pos_min) next = fd->pos_min;
        if (next > fd->pos_max) next = fd->pos_max;
        focus_set_position(next);
        *focus_pos      = next;
        s_settle_frames = AF_SETTLE_FRAMES;
    }
}
