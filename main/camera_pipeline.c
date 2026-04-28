#include "camera_pipeline.h"

#include "focus/autofocus.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>

#include "driver/isp.h"
#include "driver/ppa.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_timer.h"
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

// Fixed CSI lane count — 2 data lanes are physically wired on the
// Tanmatsu camera connector (see camera.md §2). Everything else is
// now runtime-configurable via camera_source_t.
#define PREVIEW_LANE_COUNT  2

#define BYTES_PER_PIXEL     2     // RGB565 (ISP output, not the RAW input)

// Current source descriptor, filled in by camera_preview_start.
static uint32_t              s_src_w            = 0;
static uint32_t              s_src_h            = 0;
static camera_input_format_t s_src_input_format = CAMERA_INPUT_RAW8;
static uint32_t              s_src_lane_rate    = 0;

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

// PPA mirror flags applied to preview, photo and video paths. Default
// (true, true) corrects the OV5647's natural orientation so the user
// sees a correct landscape image. Toggling both to (false, false)
// adds a 180° rotation — used when the sensor module is physically
// mounted upside down (settings menu "Rotate 180°"). Read every
// frame in the render task / capture path, so updates take effect
// on the next frame without a pipeline restart.
static bool s_mirror_x = true;
static bool s_mirror_y = true;

void camera_pipeline_set_rotate_180(bool on) {
    s_mirror_x = !on;
    s_mirror_y = !on;
}

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
    // Null-guard: a pending CSI transaction may still complete
    // during camera_preview_stop() teardown after the semaphore has
    // been deleted. Read the global into a local once and skip the
    // give if it's been cleared — better a silently-dropped ISR than
    // an xQueueGiveFromISR assert on a NULL pxQueue.
    SemaphoreHandle_t sem = s_transfer_done;
    if (sem == NULL) return false;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(sem, &hpw);
    return hpw == pdTRUE;
}

static bool IRAM_ATTR on_srm_done(ppa_client_handle_t ppa_handle, ppa_event_data_t *event, void *user_data) {
    s_cnt_srm_done++;
    // Same null-guard as on_trans_finished — a stale PPA trans can
    // complete deep into teardown if the previous mode's render_task
    // was killed mid-submit, and the completion ISR would otherwise
    // try to give a semaphore that camera_preview_stop already freed.
    SemaphoreHandle_t sem = s_srm_done;
    if (sem == NULL) return false;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(sem, &hpw);
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
        // Short timeout so the task wakes up every 100 ms and can
        // observe s_running being cleared by camera_preview_stop.
        // Originally 1 s but that made mode switches crash: the stop
        // path would vTaskDelete us mid-PPA-op, leaving a pending
        // transaction that the driver could not flush — the
        // completion ISR would then fire after the semaphore had
        // been freed. Exiting cleanly means the PPA client is
        // always idle by the time stop() tries to unregister it.
        if (xSemaphoreTake(s_transfer_done, pdMS_TO_TICKS(100)) != pdTRUE) {
            n_timeouts++;
            if ((n_timeouts % 20) == 0) {
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
        // Short timeout (100 ms) so the task can still exit quickly on
        // stop() even if the main loop is also stuck.
        if (xSemaphoreTake(s_render_ready, pdMS_TO_TICKS(100)) != pdTRUE) {
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
        int64_t t_mutex_in = esp_timer_get_time();
        xSemaphoreTake(s_ppa_mutex, portMAX_DELAY);
        int64_t t_mutex_out = esp_timer_get_time();
        // Drain any stale srm_done left behind by a previous op.
        xSemaphoreTake(s_srm_done, 0);

        ppa_srm_oper_config_t srm = {
            .in = {
                .buffer         = (void *)src,
                .pic_w          = s_src_w,
                .pic_h          = s_src_h,
                .block_w        = s_src_w,
                .block_h        = s_src_h,
                .block_offset_x = 0,
                .block_offset_y = 0,
                .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
            },
            .out = {
                .buffer         = s_preview_buffer,
                .buffer_size    = s_preview_buffer_sz,
                // With rotation 90/270 the PPA produces an image whose
                // memory dimensions are input × scale with the axes
                // swapped — i.e. width = s_src_h*scale,
                // height = s_src_w*scale. s_preview_w/h already
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
            .mirror_x       = s_mirror_x,
            .mirror_y       = s_mirror_y,
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
        int64_t t_ppa_done = esp_timer_get_time();
        xSemaphoreGive(s_ppa_mutex);
        xSemaphoreGive(s_frame_ready);
        n_frames++;
        /*
        if (n_frames <= 3 || (n_frames % 30) == 0) {
            ESP_LOGI(TAG, "frame %" PRIu32 "  ISR get=%" PRIu32 " fin=%" PRIu32
                          " srm=%" PRIu32 "  mutex_wait=%lld ppa_op=%lld us",
                     n_frames, s_cnt_get_new_trans, s_cnt_trans_finished, s_cnt_srm_done,
                     (long long)(t_mutex_out - t_mutex_in),
                     (long long)(t_ppa_done - t_mutex_out));
        }
        */
    }

    // Let camera_preview_stop know we've exited cleanly — it polls
    // this pointer so it can go straight to PPA/ISP/CSI teardown
    // without having to vTaskDelete us from the outside.
    s_render_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t camera_preview_start(const camera_source_t *src, uint32_t req_w, uint32_t req_h) {
    if (s_running) return ESP_ERR_INVALID_STATE;
    if (!src || src->width == 0 || src->height == 0 || src->lane_rate_mbps == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (req_w == 0 || req_h == 0) return ESP_ERR_INVALID_ARG;

    s_src_w            = src->width;
    s_src_h            = src->height;
    s_src_input_format = src->input_format;
    s_src_lane_rate    = src->lane_rate_mbps;

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
    float scale_max_x = (float)req_w / (float)s_src_w;
    float scale_max_y = (float)req_h / (float)s_src_h;
    float scale_max   = scale_max_x < scale_max_y ? scale_max_x : scale_max_y;
    uint32_t frag = (uint32_t)(scale_max * 16.0f);
    if (frag < 1)  frag = 1;
    if (frag > 16) frag = 16;

    s_preview_scale = (float)frag / 16.0f;

    // After PPA rotation 90°/270° the buffer dimensions swap relative
    // to the unrotated scaled image: the panel-native output is
    // (s_src_h * scale) wide × (s_src_w * scale) tall, i.e.
    // 480 × 600 at the default 12/16 scale. The buffer is already laid
    // out in the order bsp_display_blit expects, so the main UI can
    // feed it straight in via fbdraw_blit_panel (memcpy per row).
    s_preview_w = (s_src_h * frag) / 16u;
    s_preview_h = (s_src_w  * frag) / 16u;

    uint32_t cache_line = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
    ESP_LOGI(TAG, "preview buf %" PRIu32 "x%" PRIu32 " (panel-native, scale %" PRIu32 "/16) cache_line=%" PRIu32,
             s_preview_w, s_preview_h, frag, cache_line);

    // Allocate TWO camera buffers (double-buffering). The ISP writes
    // RGB565 into whichever is "active"; the other holds the
    // most-recently-completed frame and is the stable read target
    // for the render task and photo snapshots. 1920x1080 @2 B/px =
    // ~4.15 MB each, ~8.3 MB total. Comfortable in 32 MB PSRAM.
    s_cam_buf_sz = (size_t)s_src_w * s_src_h * BYTES_PER_PIXEL;
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

    // Map the source input format onto the CSI / ISP color enums and
    // decide whether the ISP runs as a Bayer demosaicer (OV5647: RAW
    // in, RGB565 out) or as a transparent pass-through (OV5640/OV5645:
    // sensor already delivers RGB565, ISP is in bypass mode so the
    // CSI bridge still routes data through it but no color processing
    // happens).
    bool        bayer_input    = false;
    cam_ctlr_color_t csi_in_ct = CAM_CTLR_COLOR_RGB565;
    isp_color_t isp_in_ct      = ISP_COLOR_RGB565;
    isp_color_t isp_out_ct     = ISP_COLOR_RGB565;
    switch (s_src_input_format) {
        case CAMERA_INPUT_RAW10:
            bayer_input = true;
            csi_in_ct   = CAM_CTLR_COLOR_RAW10;
            isp_in_ct   = ISP_COLOR_RAW10;
            isp_out_ct  = ISP_COLOR_RGB565;
            break;
        case CAMERA_INPUT_RAW8:
            bayer_input = true;
            csi_in_ct   = CAM_CTLR_COLOR_RAW8;
            isp_in_ct   = ISP_COLOR_RAW8;
            isp_out_ct  = ISP_COLOR_RGB565;
            break;
        case CAMERA_INPUT_RGB565:
        default:
            bayer_input = false;
            csi_in_ct   = CAM_CTLR_COLOR_RGB565;
            isp_in_ct   = ISP_COLOR_RGB565;
            isp_out_ct  = ISP_COLOR_RGB565;
            break;
    }

    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id                = 0,
        .h_res                  = s_src_w,
        .v_res                  = s_src_h,
        .data_lane_num          = PREVIEW_LANE_COUNT,
        .lane_bit_rate_mbps     = s_src_lane_rate,
        .input_data_color_type  = csi_in_ct,
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
    //
    // For RGB565-input sensors (OV5640/OV5645) we still need an ISP
    // processor — on the ESP32-P4 the CSI receiver shares a bridge
    // with the ISP and data has to flow through it to reach memory —
    // but we run it in `flags.bypass_isp` mode where input==output
    // and no color processing happens. The bayer_order field is
    // ignored in bypass.
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz                 = 80 * 1000 * 1000,
        .input_data_source      = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type  = isp_in_ct,
        .output_data_color_type = isp_out_ct,
        .has_line_start_packet  = false,
        .has_line_end_packet    = false,
        .h_res                  = s_src_w,
        .v_res                  = s_src_h,
        .bayer_order            = COLOR_RAW_ELEMENT_ORDER_GBRG,
        .flags = {
            .bypass_isp = !bayer_input,
        },
    };
    err = esp_isp_new_processor(&isp_cfg, &s_isp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_isp_new_processor: %d", err);
        goto fail;
    }
    ESP_ERROR_CHECK(esp_isp_enable(s_isp));

    // Hardware autofocus statistics tap. Only meaningful when the ISP
    // is running as a demosaicer — the AF block consumes ISP edge
    // statistics that don't exist in bypass mode. On RGB565 sensors
    // we treat the camera as fixed-focus.
    if (bayer_input) {
        if (autofocus_init(s_isp, s_src_w, s_src_h) != ESP_OK) {
            ESP_LOGW(TAG, "autofocus_init failed, AF disabled");
        }
    } else {
        ESP_LOGI(TAG, "AF disabled (RGB565 sensor, ISP in bypass)");
    }

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
    // Signal every task in the pipeline to exit on its next
    // cancellation point. The render task re-checks s_running
    // every 100 ms, so it will observe this within that window.
    s_running = false;

    // Stop the CSI hardware so no new on_trans_finished / on_get_new_trans
    // ISRs fire — render_task may still be mid-PPA-op, but at least we
    // won't accumulate additional DMA-completion events while we wait
    // for it to drain.
    if (s_csi) {
        esp_cam_ctlr_stop(s_csi);
    }

    // Wait up to 2 s for render_task to exit on its own. It clears
    // s_render_task just before calling vTaskDelete(NULL), so polling
    // the pointer is a safe and race-free join mechanism.
    //
    // This matters for mode switching: if we vTaskDelete'd
    // render_task mid-PPA-op like the old code did, the PPA client
    // would be left with an unprocessed transaction, ppa_unregister
    // would fail ("client still has unprocessed trans"), and the
    // eventual completion callback would fire after we'd already
    // freed s_srm_done — crashing with
    // "assert failed: xQueueGiveFromISR queue.c:1358 (pxQueue)".
    // Graceful exit means render_task always finishes its current
    // ppa op before breaking the loop.
    for (int i = 0; i < 200 && s_render_task; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_render_task) {
        ESP_LOGW(TAG, "render_task did not exit cleanly, forcing delete");
        vTaskDelete(s_render_task);
        s_render_task = NULL;
    }
    if (s_receive_task) {
        // Legacy variable — no task behind it in the current design,
        // but keep the hook in case future experiments put one back.
        vTaskDelete(s_receive_task);
        s_receive_task = NULL;
    }
    // Give the idle task a beat to reclaim the dead task's TCB.
    vTaskDelay(pdMS_TO_TICKS(20));

    // Belt-and-braces: drain any straggling PPA completion before we
    // unregister the client. If render_task exited gracefully there
    // won't be any, but if we had to force-delete it there might
    // still be a trans in flight. Wait up to 500 ms for it.
    if (s_ppa_mutex && s_srm_done) {
        if (xSemaphoreTake(s_ppa_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            xSemaphoreTake(s_srm_done, pdMS_TO_TICKS(500));
            xSemaphoreGive(s_ppa_mutex);
        }
    }

    if (s_ppa) {
        ppa_unregister_client(s_ppa);
        s_ppa = NULL;
    }
    if (s_isp) {
        autofocus_shutdown();
        esp_isp_disable(s_isp);
        esp_isp_del_processor(s_isp);
        s_isp = NULL;
    }
    if (s_csi) {
        esp_cam_ctlr_disable(s_csi);
        esp_cam_ctlr_del(s_csi);
        s_csi = NULL;
    }

    // Delete semaphores LAST. The ISR callbacks null-check the
    // globals, but the less racy the better: nuking the hardware
    // first guarantees no ISR can fire by the time we get here.
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

    s_src_w            = 0;
    s_src_h            = 0;
    s_src_input_format = CAMERA_INPUT_RAW8;
    s_src_lane_rate    = 0;
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
uint32_t       camera_preview_get_raw_width(void)  { return s_src_w; }
uint32_t       camera_preview_get_raw_height(void) { return s_src_h; }

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
    size_t   snap_sz    = (size_t)s_src_w * s_src_h * BYTES_PER_PIXEL;
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
            .pic_w          = s_src_w,
            .pic_h          = s_src_h,
            .block_w        = s_src_w,
            .block_h        = s_src_h,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer         = snap,
            .buffer_size    = snap_sz,
            .pic_w          = s_src_w,
            .pic_h          = s_src_h,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,   // landscape, don't rotate
        .scale_x        = 1.0f,                       // 1:1
        .scale_y        = 1.0f,
        .mirror_x       = s_mirror_x,
        .mirror_y       = s_mirror_y,
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
    if (out_w) *out_w = s_src_w;
    if (out_h) *out_h = s_src_h;
    return ESP_OK;
}

// Video recording snapshot: scale + colour-space convert the current
// stable camera frame into a caller-provided YUV420 packed buffer
// sized for the H.264 hardware encoder.
//
// See the header for the parameter contract. Key invariants:
//   - out_w/out_h must be reachable by a PPA scale factor whose
//     numerator in 1/16ths is an integer — i.e. out_w == 1920 * n/16
//     and out_h == 1080 * n/16 for the same integer n in 1..16.
//   - stride_w/stride_h are the MACROBLOCK-padded dimensions (next
//     multiple of 16) that the H.264 encoder expects its input
//     buffer to be sized for. The PPA writes the out_w x out_h
//     block at buffer offset (0, 0); rows out_h..stride_h-1 and
//     columns out_w..stride_w-1 are left at their previously
//     initialised value (caller should zero-init once on alloc).
esp_err_t camera_video_snapshot(uint8_t  *out_buf,
                                size_t    out_buf_sz,
                                uint32_t  out_w,
                                uint32_t  out_h,
                                uint32_t  stride_w,
                                uint32_t  stride_h) {
    if (out_buf == NULL || out_w == 0 || out_h == 0 ||
        stride_w < out_w || stride_h < out_h) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_running || s_ppa == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Verify the requested scale is actually representable by PPA's
    // 1/16 fixed-point scale register. We derive the integer
    // numerator n from out_w and double-check that out_h is produced
    // by the same factor — if the caller asks for a non-exact scale
    // we return INVALID_ARG rather than silently rounding.
    uint32_t n_from_w = (out_w * 16u) / s_src_w;
    if (n_from_w == 0 || n_from_w > 16) return ESP_ERR_INVALID_ARG;
    if ((s_src_w  * n_from_w) / 16u != out_w) return ESP_ERR_INVALID_ARG;
    if ((s_src_h * n_from_w) / 16u != out_h) return ESP_ERR_INVALID_ARG;
    float scale = (float)n_from_w / 16.0f;

    // YUV420 packed is 1.5 bytes per pixel at the stride dims.
    size_t min_sz = (size_t)stride_w * stride_h * 3u / 2u;
    if (out_buf_sz < min_sz) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Serialise against render_task / photo_snapshot on the same PPA
    // client. Split mutex wait from PPA submit+wait so the video
    // task's per-frame profiling can tell us whether the bottleneck
    // is contention (the mutex) or the PPA op itself.
    int64_t t_mutex_in = esp_timer_get_time();
    xSemaphoreTake(s_ppa_mutex, portMAX_DELAY);
    int64_t t_mutex_out = esp_timer_get_time();
    xSemaphoreTake(s_srm_done, 0);

    const uint8_t *src = (const uint8_t *)s_cam_stable;
    if (src == NULL) {
        xSemaphoreGive(s_ppa_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    ppa_srm_oper_config_t srm = {
        .in = {
            .buffer         = (void *)src,
            .pic_w          = s_src_w,
            .pic_h          = s_src_h,
            .block_w        = s_src_w,
            .block_h        = s_src_h,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer         = out_buf,
            .buffer_size    = out_buf_sz,
            // The output PICTURE dimensions are the padded mult-of-16
            // stride the H.264 encoder wants. The PPA writes the
            // scaled block into the top-left corner; the remaining
            // rows/columns are left alone (zero on first use).
            .pic_w          = stride_w,
            .pic_h          = stride_h,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm         = PPA_SRM_COLOR_MODE_YUV420,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x        = scale,
        .scale_y        = scale,
        .mirror_x       = s_mirror_x,
        .mirror_y       = s_mirror_y,
        .rgb_swap       = false,
        .byte_swap      = false,
        .mode           = PPA_TRANS_MODE_NON_BLOCKING,
    };
    esp_err_t err = ppa_do_scale_rotate_mirror(s_ppa, &srm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "video snapshot PPA: %d", err);
        xSemaphoreGive(s_ppa_mutex);
        return err;
    }
    if (xSemaphoreTake(s_srm_done, pdMS_TO_TICKS(500)) != pdTRUE) {
        ESP_LOGW(TAG, "video snapshot PPA timeout");
        xSemaphoreGive(s_ppa_mutex);
        return ESP_ERR_TIMEOUT;
    }
    int64_t t_ppa_done = esp_timer_get_time();
    xSemaphoreGive(s_ppa_mutex);

    // Dump a timing line every ~2 seconds (30 recording frames at
    // 15 fps) so we can compare mutex-wait vs PPA-op time.
    static int prof_n = 0;
    if (++prof_n >= 30) {
        prof_n = 0;
        ESP_LOGI(TAG, "vid snapshot: mutex_wait=%lld us, ppa_op=%lld us",
                 (long long)(t_mutex_out - t_mutex_in),
                 (long long)(t_ppa_done - t_mutex_out));
    }
    return ESP_OK;
}
