#include "camera_pipeline.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/isp.h"
#include "driver/ppa.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr_types.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "hal/ppa_types.h"

static const char *TAG = "camera_pipeline";

// Fixed preview sensor format: the OV5647 highest-res MIPI mode. We
// use this for both preview and photo capture so WYSIWYG is automatic
// and no format switch is ever required.
#define PREVIEW_WIDTH       1920
#define PREVIEW_HEIGHT      1080
#define PREVIEW_LANE_COUNT  2
// 1920x1080 RAW10 @30 fps needs ~622 Mbps total, or ~311 Mbps/lane
// before MIPI overhead. 500 gives comfortable headroom and is inside
// the ESP32-P4 DPHY envelope.
#define PREVIEW_LANE_RATE   500

#define BYTES_PER_PIXEL     2     // RGB565 (ISP output, not the RAW10 CSI input)

// The preview pipeline bakes the display's 90° rotation into the PPA
// output so the main UI can skip per-pixel rotation in software. The
// three flags below describe the exact combination of rotation + mirrors
// that produces a user-correct landscape image when the panel-native
// RGB565 buffer is handed straight to bsp_display_blit.
//
// ANGLE_270 was picked after empirical testing — with ANGLE_90 the
// preview came out 180° rotated because PPA's "90°" turned out to
// rotate in the opposite sense to the fbdraw CCW3 panel transform
// derived from pax_orientation.c.
#define PREVIEW_PPA_ROTATION   PPA_SRM_ROTATION_ANGLE_270
#define PREVIEW_PPA_MIRROR_X   true
#define PREVIEW_PPA_MIRROR_Y   true

static esp_cam_ctlr_handle_t s_csi      = NULL;
static isp_proc_handle_t     s_isp      = NULL;
static ppa_client_handle_t   s_ppa      = NULL;

// DOUBLE-BUFFERED CSI TARGETS. The ESP-IDF CSI driver only lets us
// queue one trans at a time (queue_items=1) and the on_get_new_trans
// callback has to return a buffer every time it fires, so we flip
// between two buffers: the ISR always hands the "inactive" one to the
// next CSI frame, and the "active" one — which just finished writing —
// becomes the new stable read target for the render task and the
// photo snapshot path. No pause, no tearing.
static uint8_t *s_cam_buf[2]    = {NULL, NULL};
static size_t   s_cam_buf_sz    = 0;
static volatile int s_cam_active_idx = 0;              // ISR: next to fill
static volatile uint8_t *s_cam_stable = NULL;          // task-side: most-recently-completed
static volatile bool s_photo_lock     = false;         // when set, ISR stops toggling stable/active

static uint8_t *s_preview_buffer  = NULL;  // Scaled + rotated preview RGB565 (panel-native)
static size_t   s_preview_buffer_sz = 0;
static uint32_t s_preview_w       = 0;     // buffer width  (stride in pixels, panel-native)
static uint32_t s_preview_h       = 0;     // buffer height (row count,       panel-native)
static float    s_preview_scale   = 0.0f;  // uniform scale applied by the PPA SRM op

static SemaphoreHandle_t s_transfer_done = NULL; // given from CSI trans_finished ISR
static SemaphoreHandle_t s_srm_done     = NULL;  // given from PPA srm_done ISR
static SemaphoreHandle_t s_frame_ready  = NULL;  // given to the main loop after PPA
static SemaphoreHandle_t s_render_ready = NULL;  // given by the main loop after it has
                                                  // finished drawing; gates the next PPA
                                                  // transform so it doesn't overwrite the
                                                  // preview buffer mid-draw.
static SemaphoreHandle_t s_ppa_mutex    = NULL;  // serialises s_ppa access between the
                                                  // render task and photo snapshot path
                                                  // (only one SRM op can be pending on a
                                                  // single PPA client at a time).

static TaskHandle_t s_receive_task = NULL;
static TaskHandle_t s_render_task  = NULL;
static volatile bool s_running     = false;

// DRAM counters so we can see from task context whether the ISR callbacks
// are running at all.
static DRAM_ATTR volatile uint32_t s_cnt_get_new_trans  = 0;
static DRAM_ATTR volatile uint32_t s_cnt_trans_finished = 0;
static DRAM_ATTR volatile uint32_t s_cnt_srm_done       = 0;

// CSI and PPA ISR callbacks must live in IRAM when the CAM CSI ISR is
// configured cache-safe (CONFIG_CAM_CTLR_MIPI_CSI_ISR_CACHE_SAFE=y), because
// they may run while the flash cache is disabled.
//
// on_get_new_trans fires AFTER on_trans_finished — the buffer the
// trans we just finished was writing is at s_cam_active_idx. Promote
// it to the stable read target and flip active to the other buffer,
// then hand THAT to the CSI HW as the next fill target. If a photo
// capture is in progress (s_photo_lock), skip the toggle entirely
// so the stable pointer stays put while the PPA mirror pass is
// reading it — CSI keeps overwriting the active buffer, we just
// don't advance the pointers.
static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
    s_cnt_get_new_trans++;
    if (!s_photo_lock) {
        s_cam_stable     = s_cam_buf[s_cam_active_idx];
        s_cam_active_idx ^= 1;
    }
    trans->buffer = s_cam_buf[s_cam_active_idx];
    trans->buflen = s_cam_buf_sz;
    return false;
}

static bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
    s_cnt_trans_finished++;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_transfer_done, &hpw);
    return hpw == pdTRUE;
}

static bool IRAM_ATTR on_srm_done(ppa_client_handle_t ppa_handle, ppa_event_data_t *event, void *user_data) {
    s_cnt_srm_done++;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_srm_done, &hpw);
    return hpw == pdTRUE;
}

// receive_task is GONE. The original pattern was a tight loop calling
// esp_cam_ctlr_receive() to keep the CSI trans queue populated, but
// that prevents on_get_new_trans from ever being called — the queue
// always has a pending trans waiting to be dequeued, so the HW never
// needs to ask for a new one. That broke the double-buffer toggle
// which lives in on_get_new_trans. Instead we prime the queue once
// from camera_preview_start() and let on_get_new_trans handle every
// subsequent frame's buffer selection.

static void render_task(void *arg) {
    uint32_t n_frames = 0;
    uint32_t n_timeouts = 0;
    while (s_running) {
        if (xSemaphoreTake(s_transfer_done, pdMS_TO_TICKS(1000)) != pdTRUE) {
            n_timeouts++;
            if ((n_timeouts % 2) == 0) {
                ESP_LOGW(TAG, "no transfer_done: %" PRIu32 " timeouts, frames=%" PRIu32
                              "  ISR cnts get=%" PRIu32 " fin=%" PRIu32 " srm=%" PRIu32,
                         n_timeouts, n_frames,
                         s_cnt_get_new_trans, s_cnt_trans_finished, s_cnt_srm_done);
            }
            continue;
        }
        if (!s_running) break;

        // Wait until the UI has finished drawing the previous preview frame
        // before we overwrite the preview buffer with a new PPA transform.
        // If the UI never consumes, we just drop CSI frames — the CSI
        // keeps running into its double-buffered targets, we just don't
        // PPA-convert them.
        if (xSemaphoreTake(s_render_ready, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }
        if (!s_running) break;

        // Read the current stable camera buffer pointer. The CSI ISR
        // promotes buf[active_idx] to stable inside on_get_new_trans,
        // AFTER the frame was fully written but BEFORE the HW starts
        // writing the next one into the other buffer. So this pointer
        // is guaranteed not to be overwritten for the full duration
        // of the next frame period, i.e. ~66 ms at 15 fps — plenty
        // of time for a PPA pass that takes a couple of ms.
        const uint8_t *src = (const uint8_t *)s_cam_stable;
        if (src == NULL) {
            // CSI hasn't completed a frame yet — hand render_ready
            // back so the main loop doesn't deadlock.
            xSemaphoreGive(s_render_ready);
            continue;
        }

        // Serialise PPA client access against the photo snapshot path.
        // ppa_do_scale_rotate_mirror + srm_done wait must happen
        // atomically from the PPA's point of view — max_pending_trans_num
        // is 1 on this client.
        xSemaphoreTake(s_ppa_mutex, portMAX_DELAY);
        // Drain any stale srm_done left behind by a previous op.
        xSemaphoreTake(s_srm_done, 0);

        ppa_srm_oper_config_t srm = {
            .in = {
                .buffer         = (void *)src,
                .pic_w          = PREVIEW_WIDTH,
                .pic_h          = PREVIEW_HEIGHT,
                .block_w        = PREVIEW_WIDTH,
                .block_h        = PREVIEW_HEIGHT,
                .block_offset_x = 0,
                .block_offset_y = 0,
                .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
            },
            .out = {
                .buffer         = s_preview_buffer,
                .buffer_size    = s_preview_buffer_sz,
                // With rotation 90/270 the PPA produces an image whose
                // memory dimensions are input × scale with the axes
                // swapped — i.e. width = PREVIEW_HEIGHT*scale,
                // height = PREVIEW_WIDTH*scale. s_preview_w/h already
                // reflect that post-rotation layout.
                .pic_w          = s_preview_w,
                .pic_h          = s_preview_h,
                .block_offset_x = 0,
                .block_offset_y = 0,
                .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
            },
            .rotation_angle = PREVIEW_PPA_ROTATION,
            // scale_x and scale_y are uniform — aspect ratio preserved.
            .scale_x        = s_preview_scale,
            .scale_y        = s_preview_scale,
            // Sensor feed is vertically flipped AND left/right mirrored
            // relative to how the user expects to see it on the display.
            // Both flips are free on the PPA.
            .mirror_x       = PREVIEW_PPA_MIRROR_X,
            .mirror_y       = PREVIEW_PPA_MIRROR_Y,
            .rgb_swap       = false,
            .byte_swap      = false,
            .mode           = PPA_TRANS_MODE_NON_BLOCKING,
        };

        esp_err_t rerr = ppa_do_scale_rotate_mirror(s_ppa, &srm);
        if (rerr != ESP_OK) {
            ESP_LOGW(TAG, "ppa_do_scale_rotate_mirror: %d", rerr);
            xSemaphoreGive(s_ppa_mutex);
            continue;
        }

        if (xSemaphoreTake(s_srm_done, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_LOGW(TAG, "srm_done timeout");
            xSemaphoreGive(s_ppa_mutex);
            continue;
        }
        xSemaphoreGive(s_ppa_mutex);
        xSemaphoreGive(s_frame_ready);
        n_frames++;
        if (n_frames <= 3 || (n_frames % 30) == 0) {
            // ISR counters alongside n_frames — if get/fin run much
            // faster than n_frames, the main loop / PPA is the
            // bottleneck and frames are being dropped at render_ready.
            // If they track n_frames, the sensor rate is what it is.
            ESP_LOGI(TAG, "frame %" PRIu32 "  ISR get=%" PRIu32 " fin=%" PRIu32
                          " srm=%" PRIu32,
                     n_frames, s_cnt_get_new_trans, s_cnt_trans_finished, s_cnt_srm_done);
        }
    }
    vTaskDelete(NULL);
}

esp_err_t camera_preview_start(uint32_t req_w, uint32_t req_h) {
    if (s_running) return ESP_ERR_INVALID_STATE;
    if (req_w == 0 || req_h == 0) return ESP_ERR_INVALID_ARG;

    // The ESP32-P4 PPA scale register (PPA_SR_SCAL_X_FRAG_V) has only 4
    // fractional bits — scale factor resolution is 1/16. Asking for a scale
    // that doesn't land exactly on an n/16 boundary causes the PPA to
    // round it DOWN, producing an output image narrower/shorter than the
    // configured out.pic_w/pic_h, which leaves a strip of unwritten memory
    // (visible as noise) along the right/bottom edges of the preview buffer.
    //
    // Snap the scale factor to the largest n/16 that still fits inside the
    // requested box. Input is the fixed 800x640 sensor frame, so output is
    // always (n*50) x (n*40), with n in 1..16.
    // req_w / req_h are still expressed in user-landscape space: "fit
    // the 800x640 camera frame in this many landscape pixels, axis-
    // aligned, keeping aspect ratio". That's what the PPA scale factor
    // is computed against — the rotation that follows doesn't change
    // how many source pixels we consume, just how they're laid out.
    float scale_max_x = (float)req_w / (float)PREVIEW_WIDTH;
    float scale_max_y = (float)req_h / (float)PREVIEW_HEIGHT;
    float scale_max   = scale_max_x < scale_max_y ? scale_max_x : scale_max_y;
    uint32_t frag = (uint32_t)(scale_max * 16.0f);
    if (frag < 1)  frag = 1;
    if (frag > 16) frag = 16;

    s_preview_scale = (float)frag / 16.0f;

    // After PPA rotation 90°/270° the buffer dimensions swap relative
    // to the unrotated scaled image: the panel-native output is
    // (PREVIEW_HEIGHT * scale) wide × (PREVIEW_WIDTH * scale) tall, i.e.
    // 480 × 600 at the default 12/16 scale. The buffer is already laid
    // out in the order bsp_display_blit expects, so the main UI can
    // feed it straight in via fbdraw_blit_panel (memcpy per row).
    s_preview_w = (PREVIEW_HEIGHT * frag) / 16u;
    s_preview_h = (PREVIEW_WIDTH  * frag) / 16u;

    uint32_t cache_line = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
    ESP_LOGI(TAG, "preview buf %" PRIu32 "x%" PRIu32 " (panel-native, scale %" PRIu32 "/16) cache_line=%" PRIu32,
             s_preview_w, s_preview_h, frag, cache_line);

    // Allocate TWO camera buffers (double-buffering). The ISP writes
    // RGB565 into whichever is "active"; the other holds the
    // most-recently-completed frame and is the stable read target
    // for the render task and photo snapshots. 1920x1080 @2 B/px =
    // ~4.15 MB each, ~8.3 MB total. Comfortable in 32 MB PSRAM.
    s_cam_buf_sz = (size_t)PREVIEW_WIDTH * PREVIEW_HEIGHT * BYTES_PER_PIXEL;
    s_cam_buf_sz = (s_cam_buf_sz + cache_line - 1u) & ~((size_t)cache_line - 1u);
    for (int i = 0; i < 2; i++) {
        s_cam_buf[i] = heap_caps_aligned_calloc(cache_line, 1, s_cam_buf_sz,
                                                MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
        if (!s_cam_buf[i]) {
            ESP_LOGE(TAG, "cam_buf[%d] alloc failed (%zu bytes)", i, s_cam_buf_sz);
            goto fail;
        }
    }
    s_cam_active_idx = 0;
    s_cam_stable     = NULL;  // nothing stable yet; render_task bails until first frame
    s_photo_lock     = false;

    // buffer_size must be a multiple of the cache line size per
    // ppa_do_scale_rotate_mirror()'s alignment check; pad the allocation
    // up to the next cache line. PPA only writes s_preview_w * s_preview_h
    // pixels; the padding tail is never touched.
    s_preview_buffer_sz = (size_t)s_preview_w * s_preview_h * BYTES_PER_PIXEL;
    s_preview_buffer_sz = (s_preview_buffer_sz + cache_line - 1u) & ~((size_t)cache_line - 1u);
    s_preview_buffer    = heap_caps_aligned_calloc(cache_line, 1, s_preview_buffer_sz,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!s_preview_buffer) {
        ESP_LOGE(TAG, "preview_buffer alloc failed");
        goto fail;
    }

    s_transfer_done = xSemaphoreCreateBinary();
    s_srm_done      = xSemaphoreCreateBinary();
    s_frame_ready   = xSemaphoreCreateBinary();
    s_render_ready  = xSemaphoreCreateBinary();
    s_ppa_mutex     = xSemaphoreCreateMutex();
    if (!s_transfer_done || !s_srm_done || !s_frame_ready || !s_render_ready || !s_ppa_mutex) {
        ESP_LOGE(TAG, "semaphore alloc failed");
        goto fail;
    }
    // Prime: the first PPA transform may run before the main loop has
    // drawn anything.
    xSemaphoreGive(s_render_ready);

    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id                = 0,
        .h_res                  = PREVIEW_WIDTH,
        .v_res                  = PREVIEW_HEIGHT,
        .data_lane_num          = PREVIEW_LANE_COUNT,
        .lane_bit_rate_mbps     = PREVIEW_LANE_RATE,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW10,
        .output_data_color_type = CAM_CTLR_COLOR_RGB565,
        .queue_items            = 1,
        .byte_swap_en           = false,
        .bk_buffer_dis          = false,
    };
    esp_err_t err = esp_cam_new_csi_ctlr(&csi_cfg, &s_csi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_new_csi_ctlr: %d", err);
        goto fail;
    }

    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = on_get_new_trans,
        .on_trans_finished = on_trans_finished,
    };
    ESP_ERROR_CHECK(esp_cam_ctlr_register_event_callbacks(s_csi, &cbs, NULL));
    ESP_ERROR_CHECK(esp_cam_ctlr_enable(s_csi));

    // OV5647 sensor reports bayer_type=GBRG in its isp_info. Without this
    // the ISP defaults to BGGR (enum value 0), which demosaics the wrong
    // colour for every pixel and produces the classic green/magenta
    // speckle pattern on coloured regions.
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz                 = 80 * 1000 * 1000,
        .input_data_source      = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type  = ISP_COLOR_RAW10,
        .output_data_color_type = ISP_COLOR_RGB565,
        .has_line_start_packet  = false,
        .has_line_end_packet    = false,
        .h_res                  = PREVIEW_WIDTH,
        .v_res                  = PREVIEW_HEIGHT,
        .bayer_order            = COLOR_RAW_ELEMENT_ORDER_GBRG,
    };
    err = esp_isp_new_processor(&isp_cfg, &s_isp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_isp_new_processor: %d", err);
        goto fail;
    }
    ESP_ERROR_CHECK(esp_isp_enable(s_isp));

    ppa_client_config_t ppa_cfg = {
        .oper_type             = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
        .data_burst_length     = PPA_DATA_BURST_LENGTH_128,
    };
    err = ppa_register_client(&ppa_cfg, &s_ppa);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa_register_client: %d", err);
        goto fail;
    }
    ppa_event_callbacks_t ppa_cbs = {
        .on_trans_done = on_srm_done,
    };
    ESP_ERROR_CHECK(ppa_client_register_event_callbacks(s_ppa, &ppa_cbs));

    s_running = true;
    xTaskCreate(render_task,  "cam_render",  4096, NULL, 5, &s_render_task);

    ESP_ERROR_CHECK(esp_cam_ctlr_start(s_csi));

    // Prime the CSI trans queue exactly once. From here on,
    // on_get_new_trans is called from ISR context every time the HW
    // finishes a frame and needs a new buffer, and that's where the
    // double-buffer toggle happens. No receive_task required.
    {
        esp_cam_ctlr_trans_t trans = {
            .buffer = s_cam_buf[s_cam_active_idx],
            .buflen = s_cam_buf_sz,
        };
        err = esp_cam_ctlr_receive(s_csi, &trans, 1000);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "initial esp_cam_ctlr_receive: %d", err);
            goto fail;
        }
    }

    ESP_LOGI(TAG, "preview pipeline started");
    return ESP_OK;

fail:
    camera_preview_stop();
    return ESP_FAIL;
}

void camera_preview_stop(void) {
    s_running = false;

    // Stop CSI first so the hardware stops firing callbacks. The driver's
    // own per-ctlr state is still intact at this point — we're just
    // pausing the DMA / bridge.
    if (s_csi) {
        esp_cam_ctlr_stop(s_csi);
    }

    // The receive_task is blocked inside esp_cam_ctlr_receive() on an
    // internal CSI queue with ESP_CAM_CTLR_MAX_DELAY and will never
    // notice s_running=false on its own. The render_task might be
    // waiting on s_transfer_done or s_render_ready with timeouts up to
    // 1s. Force both to exit before we start deleting the CSI/PPA/ISP
    // whose objects they still reference — deleting a CSI controller
    // while a task is still blocked inside it double-acquires an
    // internal spinlock and panics.
    if (s_receive_task) {
        vTaskDelete(s_receive_task);
        s_receive_task = NULL;
    }
    if (s_render_task) {
        vTaskDelete(s_render_task);
        s_render_task = NULL;
    }
    // Let FreeRTOS run the idle task so the deleted tasks' TCBs are
    // actually reclaimed before we free anything else.
    vTaskDelay(pdMS_TO_TICKS(10));

    if (s_ppa) {
        ppa_unregister_client(s_ppa);
        s_ppa = NULL;
    }
    if (s_isp) {
        esp_isp_disable(s_isp);
        esp_isp_del_processor(s_isp);
        s_isp = NULL;
    }
    if (s_csi) {
        esp_cam_ctlr_disable(s_csi);
        esp_cam_ctlr_del(s_csi);
        s_csi = NULL;
    }

    if (s_transfer_done) { vSemaphoreDelete(s_transfer_done); s_transfer_done = NULL; }
    if (s_srm_done)      { vSemaphoreDelete(s_srm_done);      s_srm_done      = NULL; }
    if (s_frame_ready)   { vSemaphoreDelete(s_frame_ready);   s_frame_ready   = NULL; }
    if (s_render_ready)  { vSemaphoreDelete(s_render_ready);  s_render_ready  = NULL; }
    if (s_ppa_mutex)     { vSemaphoreDelete(s_ppa_mutex);     s_ppa_mutex     = NULL; }

    // Free both double-buffer slots. Clear the volatile pointers
    // first so any late ISR sees NULL rather than a dangling buffer.
    s_cam_stable     = NULL;
    s_cam_active_idx = 0;
    for (int i = 0; i < 2; i++) {
        if (s_cam_buf[i]) { free(s_cam_buf[i]); s_cam_buf[i] = NULL; }
    }
    s_cam_buf_sz = 0;
    if (s_preview_buffer) { free(s_preview_buffer); s_preview_buffer = NULL; s_preview_buffer_sz = 0; }
}

void camera_preview_give_render_ready(void) {
    if (s_render_ready) {
        xSemaphoreGive(s_render_ready);
    }
}

esp_err_t camera_preview_wait_frame(uint32_t timeout_ms) {
    if (!s_frame_ready) return ESP_ERR_INVALID_STATE;
    if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

const uint8_t *camera_preview_get_pixels(void) { return s_preview_buffer; }
uint32_t       camera_preview_get_width(void)  { return s_preview_w; }
uint32_t       camera_preview_get_height(void) { return s_preview_h; }

// Returns the current stable camera buffer — i.e. the one the CSI
// ISR has most recently promoted as "finished". May be NULL during
// the very first ISR cycle before any frame has completed.
const uint8_t *camera_preview_get_raw_pixels(void) { return (const uint8_t *)s_cam_stable; }
uint32_t       camera_preview_get_raw_width(void)  { return PREVIEW_WIDTH; }
uint32_t       camera_preview_get_raw_height(void) { return PREVIEW_HEIGHT; }

// Photo snapshot: take the current stable double-buffered CSI frame
// and run it through a PPA mirror-only pass (scale 1.0, rotation 0,
// mirror_x + mirror_y matching the preview) into a fresh RGB565
// buffer. Returns a standard 1920x1080 landscape image — no sensor
// pause, no format switch, no stream toggle.
//
// How the race is avoided:
//   1. Take s_ppa_mutex so the render task can't submit a concurrent
//      PPA op on the same client.
//   2. Set s_photo_lock = true so on_get_new_trans stops advancing
//      s_cam_stable and s_cam_active_idx. This freezes the stable
//      pointer for the duration of the PPA pass — the CSI keeps
//      overwriting the same active buffer, but we don't care because
//      we're reading the OTHER one.
//   3. Snapshot s_cam_stable into a local pointer.
//   4. Run the mirror PPA pass on that buffer.
//   5. Clear s_photo_lock and release s_ppa_mutex.
//
// On success *out_buf points to a cache-line-aligned PSRAM buffer the
// caller owns and must heap_caps_free(). *out_w / *out_h return
// 1920 / 1080.
esp_err_t camera_photo_snapshot(uint8_t **out_buf, uint32_t *out_w, uint32_t *out_h) {
    if (out_buf == NULL) return ESP_ERR_INVALID_ARG;
    if (!s_running || s_ppa == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    *out_buf = NULL;
    if (out_w) *out_w = 0;
    if (out_h) *out_h = 0;

    uint32_t cache_line = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
    size_t   snap_sz    = (size_t)PREVIEW_WIDTH * PREVIEW_HEIGHT * BYTES_PER_PIXEL;
    snap_sz = (snap_sz + cache_line - 1u) & ~((size_t)cache_line - 1u);

    uint8_t *snap = heap_caps_aligned_calloc(cache_line, 1, snap_sz,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (snap == NULL) {
        ESP_LOGE(TAG, "photo snapshot alloc failed (%zu bytes)", snap_sz);
        return ESP_ERR_NO_MEM;
    }

    // Serialise against the render task's PPA ops on the same client.
    xSemaphoreTake(s_ppa_mutex, portMAX_DELAY);
    // Drain any stale srm_done left by a previous op.
    xSemaphoreTake(s_srm_done, 0);

    // Freeze the stable pointer for the duration of the PPA pass.
    // From this point the ISR still fires on CSI frame completion
    // but on_get_new_trans keeps s_cam_stable / s_cam_active_idx
    // where they are.
    s_photo_lock = true;

    const uint8_t *src = (const uint8_t *)s_cam_stable;
    if (src == NULL) {
        ESP_LOGE(TAG, "photo snapshot: no stable frame yet");
        s_photo_lock = false;
        xSemaphoreGive(s_ppa_mutex);
        free(snap);
        return ESP_ERR_INVALID_STATE;
    }

    ppa_srm_oper_config_t srm = {
        .in = {
            .buffer         = (void *)src,
            .pic_w          = PREVIEW_WIDTH,
            .pic_h          = PREVIEW_HEIGHT,
            .block_w        = PREVIEW_WIDTH,
            .block_h        = PREVIEW_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer         = snap,
            .buffer_size    = snap_sz,
            .pic_w          = PREVIEW_WIDTH,
            .pic_h          = PREVIEW_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,   // landscape, don't rotate
        .scale_x        = 1.0f,                       // 1:1
        .scale_y        = 1.0f,
        .mirror_x       = PREVIEW_PPA_MIRROR_X,
        .mirror_y       = PREVIEW_PPA_MIRROR_Y,
        .rgb_swap       = false,
        .byte_swap      = false,
        .mode           = PPA_TRANS_MODE_NON_BLOCKING,
    };
    esp_err_t err = ppa_do_scale_rotate_mirror(s_ppa, &srm);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "photo snapshot PPA: %d", err);
        s_photo_lock = false;
        xSemaphoreGive(s_ppa_mutex);
        free(snap);
        return err;
    }
    if (xSemaphoreTake(s_srm_done, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGE(TAG, "photo snapshot PPA timeout");
        s_photo_lock = false;
        xSemaphoreGive(s_ppa_mutex);
        free(snap);
        return ESP_ERR_TIMEOUT;
    }

    // Release the ISR toggle lock and the PPA mutex. The next CSI
    // completion will advance s_cam_stable / s_cam_active_idx normally.
    s_photo_lock = false;
    xSemaphoreGive(s_ppa_mutex);

    *out_buf = snap;
    if (out_w) *out_w = PREVIEW_WIDTH;
    if (out_h) *out_h = PREVIEW_HEIGHT;
    return ESP_OK;
}
