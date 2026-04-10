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

// Fixed preview sensor format (camera.md §8).
#define PREVIEW_WIDTH       800
#define PREVIEW_HEIGHT      640
#define PREVIEW_LANE_COUNT  2
#define PREVIEW_LANE_RATE   200   // Mbps per lane

#define BYTES_PER_PIXEL     2     // RGB565

static esp_cam_ctlr_handle_t s_csi      = NULL;
static isp_proc_handle_t     s_isp      = NULL;
static ppa_client_handle_t   s_ppa      = NULL;

static uint8_t *s_camera_buffer   = NULL;  // Full 800x640 RGB565 CSI output
static size_t   s_camera_buffer_sz = 0;

static uint8_t *s_preview_buffer  = NULL;  // Scaled preview (display-size) RGB565
static size_t   s_preview_buffer_sz = 0;
static uint32_t s_preview_w       = 0;
static uint32_t s_preview_h       = 0;

static SemaphoreHandle_t s_transfer_done = NULL; // given from CSI trans_finished ISR
static SemaphoreHandle_t s_srm_done     = NULL;  // given from PPA srm_done ISR
static SemaphoreHandle_t s_frame_ready  = NULL;  // given to the main loop after PPA
static SemaphoreHandle_t s_render_ready = NULL;  // given by the main loop after it has
                                                  // finished drawing; gates the next PPA
                                                  // transform so it doesn't overwrite the
                                                  // preview buffer mid-draw.

static esp_cam_ctlr_trans_t s_trans;

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
// Matches the ESP-IDF mipi_isp_dsi example: unconditionally provide the
// buffer. Back-pressure with the UI is handled task-side via a binary
// frame_ready semaphore, not in the ISR.
static bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
    s_cnt_get_new_trans++;
    trans->buffer = s_trans.buffer;
    trans->buflen = s_trans.buflen;
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

static void receive_task(void *arg) {
    while (s_running) {
        esp_cam_ctlr_receive(s_csi, &s_trans, ESP_CAM_CTLR_MAX_DELAY);
    }
    vTaskDelete(NULL);
}

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
        // If the UI never consumes, we just drop CSI frames (the CSI DMA
        // keeps running into s_camera_buffer but we don't PPA-convert them).
        if (xSemaphoreTake(s_render_ready, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }
        if (!s_running) break;

        float scale_x = (float)s_preview_w / (float)PREVIEW_WIDTH;
        float scale_y = (float)s_preview_h / (float)PREVIEW_HEIGHT;

        ppa_srm_oper_config_t srm = {
            .in = {
                .buffer         = s_camera_buffer,
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
                .pic_w          = s_preview_w,
                .pic_h          = s_preview_h,
                .block_offset_x = 0,
                .block_offset_y = 0,
                .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
            },
            .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
            .scale_x        = scale_x,
            .scale_y        = scale_y,
            // Sensor feed is vertically flipped AND left/right mirrored
            // relative to how the user expects to see it on the display.
            // Both flips are free on the PPA.
            .mirror_x       = true,
            .mirror_y       = true,
            .rgb_swap       = false,
            .byte_swap      = false,
            .mode           = PPA_TRANS_MODE_NON_BLOCKING,
        };

        esp_err_t rerr = ppa_do_scale_rotate_mirror(s_ppa, &srm);
        if (rerr != ESP_OK) {
            ESP_LOGW(TAG, "ppa_do_scale_rotate_mirror: %d", rerr);
            continue;
        }

        if (xSemaphoreTake(s_srm_done, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_LOGW(TAG, "srm_done timeout");
            continue;
        }
        xSemaphoreGive(s_frame_ready);
        n_frames++;
        if (n_frames <= 3 || (n_frames % 30) == 0) {
            ESP_LOGI(TAG, "frame %" PRIu32, n_frames);
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
    float scale_max_x = (float)req_w / (float)PREVIEW_WIDTH;
    float scale_max_y = (float)req_h / (float)PREVIEW_HEIGHT;
    float scale_max   = scale_max_x < scale_max_y ? scale_max_x : scale_max_y;
    uint32_t frag = (uint32_t)(scale_max * 16.0f);
    if (frag < 1)  frag = 1;
    if (frag > 16) frag = 16;

    s_preview_w = (PREVIEW_WIDTH  * frag) / 16u;
    s_preview_h = (PREVIEW_HEIGHT * frag) / 16u;

    uint32_t cache_line = cache_hal_get_cache_line_size(CACHE_LL_LEVEL_EXT_MEM, CACHE_TYPE_DATA);
    ESP_LOGI(TAG, "preview=%" PRIu32 "x%" PRIu32 " (scale %" PRIu32 "/16) cache_line=%" PRIu32,
             s_preview_w, s_preview_h, frag, cache_line);

    s_camera_buffer_sz = PREVIEW_WIDTH * PREVIEW_HEIGHT * BYTES_PER_PIXEL;
    s_camera_buffer    = heap_caps_aligned_calloc(cache_line, 1, s_camera_buffer_sz,
                                                  MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
    if (!s_camera_buffer) {
        ESP_LOGE(TAG, "camera_buffer alloc failed");
        goto fail;
    }

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
    if (!s_transfer_done || !s_srm_done || !s_frame_ready || !s_render_ready) {
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
        .input_data_color_type  = CAM_CTLR_COLOR_RAW8,
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
        .input_data_color_type  = ISP_COLOR_RAW8,
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

    s_trans.buffer = s_camera_buffer;
    s_trans.buflen = s_camera_buffer_sz;

    s_running = true;
    xTaskCreate(render_task,  "cam_render",  4096, NULL, 5, &s_render_task);
    xTaskCreate(receive_task, "cam_receive", 4096, NULL, 6, &s_receive_task);

    ESP_ERROR_CHECK(esp_cam_ctlr_start(s_csi));
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

    if (s_camera_buffer)  { free(s_camera_buffer);  s_camera_buffer  = NULL; s_camera_buffer_sz  = 0; }
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

const uint8_t *camera_preview_get_raw_pixels(void) { return s_camera_buffer; }
uint32_t       camera_preview_get_raw_width(void)  { return PREVIEW_WIDTH; }
uint32_t       camera_preview_get_raw_height(void) { return PREVIEW_HEIGHT; }
