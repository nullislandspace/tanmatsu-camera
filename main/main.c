#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include "bsp/device.h"
#include "bsp/display.h"
#include "bsp/input.h"
#include "bsp/led.h"
#include "bsp/power.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/lcd_types.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "bmp_writer.h"
#include "camera_pipeline.h"
#include "camera_sensor.h"
#include "fbdraw.h"
#include "photo.h"
#include "sdcard.h"
#include "usb_device.h"
#include "video.h"
#include "viewer.h"

#define DCIM_PATH "/sd/DCIM"

// Cap the sensor at 15 fps. At the native 50 fps the CSI DMA alone
// moves ~20 MB/s through PSRAM (10% of its total bandwidth) for frames
// we'd mostly throw away, and Step 7's H.264 recording target is also
// 15 fps — keeping the sensor locked to that rate means no frame
// dropping, no phase beating, and no wasted DMA.
#define PREVIEW_TARGET_FPS 15u

// Source descriptors for the two pipeline configurations the app
// swaps between. PHOTO uses the sensor's highest-res MIPI mode so
// preview and still capture can share a single WYSIWYG frame. VIDEO
// drops to 800x640 RAW8 so the PPA has enough headroom to do both
// the preview scale AND the YUV420 recording scale inside a 15 fps
// frame budget (1920x1080 blew past ~83 ms per PPA op, turning the
// whole pipeline into a 6-fps slideshow). See camera.md §9 for the
// bandwidth math.
static const camera_source_t SRC_PHOTO = {
    .width          = 1920,
    .height         = 1080,
    .is_raw10       = true,
    .lane_rate_mbps = 500,
};
static const camera_source_t SRC_VIDEO = {
    .width          = 800,
    .height         = 640,
    .is_raw10       = false,
    .lane_rate_mbps = 200,
};

typedef enum {
    MODE_PHOTO = 0,
    MODE_VIDEO,
    MODE_VIEW,
} app_mode_t;

static const char *mode_name(app_mode_t m) {
    switch (m) {
        case MODE_PHOTO: return "PHOTO";
        case MODE_VIDEO: return "VIDEO";
        case MODE_VIEW:  return "VIEW";
    }
    return "?";
}

static char const TAG[] = "main";

static size_t                       display_h_res        = 0;
static size_t                       display_v_res        = 0;
static lcd_color_rgb_pixel_format_t display_color_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
static lcd_rgb_data_endian_t        display_data_endian  = LCD_RGB_DATA_ENDIAN_LITTLE;
static fbdraw_t                     fb                   = {0};
static QueueHandle_t                input_event_queue    = NULL;

// RGB565 colours used by this TU. FBDRAW_BLACK/WHITE are already defined
// in fbdraw.h; add the rest as short names for readability.
#define BLACK       FBDRAW_BLACK
#define WHITE       FBDRAW_WHITE
#define RED         FBDRAW_RED
#define HUD_BG      fbdraw_rgb(32, 32, 32)    // dark grey strip behind HUD text
#define BANNER_BG   fbdraw_rgb(0, 0, 128)     // dark blue banner strip

static void blit(void) {
    bsp_display_blit(0, 0, display_h_res, display_v_res, fb.pixels);
}

static void splash(uint16_t bg, uint16_t fg, const char *line1, const char *line2) {
    fbdraw_clear(&fb, bg);
    fbdraw_hershey_string(&fb, fg, 16, 32, line1, 24);
    if (line2) {
        fbdraw_hershey_string(&fb, fg, 16, 64, line2, 18);
    }
    blit();
}

// Load the POSIX TZ string the launcher saved to NVS (namespace "system",
// key "tz") and apply it, so localtime_r() in photo.c produces wall-clock
// timestamps matching the timezone the user picked in tanmatsu-launcher's
// clock settings. Falls back to UTC if NVS has no entry.
static void apply_saved_timezone(void) {
    nvs_handle_t h;
    esp_err_t    err = nvs_open("system", NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no 'system' NVS namespace (%d), using UTC", err);
        setenv("TZ", "UTC0", 1);
        tzset();
        return;
    }
    char   tz[64] = {0};
    size_t tz_sz  = sizeof(tz);
    err = nvs_get_str(h, "tz", tz, &tz_sz);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no 'tz' key in NVS (%d), using UTC", err);
        setenv("TZ", "UTC0", 1);
    } else {
        ESP_LOGI(TAG, "applying saved TZ=\"%s\"", tz);
        setenv("TZ", tz, 1);
    }
    tzset();
}

// Tear the current preview pipeline down, reconfigure the sensor to
// the format associated with the target mode, and bring the pipeline
// back up. Used on F1/F2/F3 transitions — PHOTO and VIEW share the
// 1920x1080 RAW10 source because F1 (viewer) doesn't actually stream
// from the camera, it just reuses the preview pipeline as a backdrop.
//
// Returns ESP_OK if the transition completes; on failure the pipeline
// is left torn down and the caller is expected to surface an error
// screen / banner to the user.
static esp_err_t switch_pipeline_to_source(camera_sensor_t *sensor,
                                           const camera_source_t *src,
                                           bool is_video_mode,
                                           uint32_t preview_area_w,
                                           uint32_t preview_area_h) {
    // Stop sensor stream first so the CSI DMA stops firing while we
    // tear down the controller.
    camera_sensor_stream(sensor, false);
    camera_preview_stop();

    // Reconfigure the sensor. For PHOTO we use the 1920x1080 RAW10
    // preset and then override VTS to land at PREVIEW_TARGET_FPS;
    // for VIDEO we use the 800x640 RAW8 preset and leave the native
    // 50 fps alone (the video task only pulls at 15 Hz, the extra
    // frames are just wasted CSI DMA at <15 MB/s which is free).
    esp_err_t err;
    if (is_video_mode) {
        err = camera_sensor_set_format_video(sensor, NULL);
    } else {
        err = camera_sensor_set_format_preview(sensor, NULL);
        if (err == ESP_OK) {
            if (camera_sensor_set_preview_fps(sensor, PREVIEW_TARGET_FPS) != ESP_OK) {
                ESP_LOGW(TAG, "fps override failed, continuing at native rate");
            }
        }
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sensor format switch: %d", err);
        return err;
    }

    err = camera_preview_start(src, preview_area_w, preview_area_h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "preview start: %d", err);
        return err;
    }
    err = camera_sensor_stream(sensor, true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "stream start: %d", err);
        camera_preview_stop();
        return err;
    }
    return ESP_OK;
}

static void wait_for_esc(void) {
    while (1) {
        bsp_input_event_t event;
        if (xQueueReceive(input_event_queue, &event, portMAX_DELAY) == pdTRUE) {
            if (event.type == INPUT_EVENT_TYPE_NAVIGATION && event.args_navigation.state &&
                event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_ESC) {
                bsp_device_restart_to_launcher();
            }
        }
    }
}

void app_main(void) {
    // Switch the USB PHY out of badgelink mode back into flash/monitor mode.
    // Must run before bsp_device_initialize(). See videoplayer usb_device.c.
    usb_initialize();

    // ===== FOR DEVELOPMENT ONLY =====
    // Give the host-side serial monitor time to reconnect after the USB PHY
    // flip above — the Tanmatsu's monitor mode can take several seconds to
    // re-enumerate, and without this pause the first chunk of startup logs
    // is lost. Uncomment when you need to capture the very first boot logs.
    // vTaskDelay(pdMS_TO_TICKS(10000));
    // ===== END FOR DEVELOPMENT ONLY =====

    // Start the GPIO interrupt service
    gpio_install_isr_service(0);

    // Initialize the Non Volatile Storage partition
    esp_err_t res = nvs_flash_init();
    if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        res = nvs_flash_erase();
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "Failed to erase NVS flash: %d", res);
            return;
        }
        res = nvs_flash_init();
    }
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash: %d", res);
        return;
    }

    // The launcher stores the user's chosen timezone as a POSIX TZ string
    // in NVS. Apply it before any localtime_r() call so photo filenames
    // match wall-clock time.
    apply_saved_timezone();

    // Initialize the Board Support Package.
    // Display is configured as native RGB565 so the preview pipeline can
    // feed its PPA output straight into the framebuffer without any
    // software colour-space conversion. The ST7701 panel accepts RGB565
    // directly over MIPI DSI (COLMOD=0x55) so there is no penalty for
    // picking the lower bit depth here.
    const bsp_configuration_t bsp_configuration = {
        .display =
            {
                .requested_color_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
                .num_fbs                = 1,
            },
    };
    res = bsp_device_initialize(&bsp_configuration);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BSP: %d", res);
        return;
    }

    // Get display parameters and rotation.
    res = bsp_display_get_parameters(&display_h_res, &display_v_res, &display_color_format, &display_data_endian);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get display parameters: %d", res);
        return;
    }

    // Tanmatsu's ST7701 is a physical 480 x 800 portrait panel. The BSP
    // default orientation is BSP_DISPLAY_ROTATION_270 (landscape-right),
    // which fbdraw bakes into its user-coordinate transform. If the BSP
    // ever reports a different rotation or format we abort rather than
    // silently render wrong.
    if (display_color_format != LCD_COLOR_PIXEL_FORMAT_RGB565) {
        ESP_LOGE(TAG, "fbdraw expects RGB565 (got %d)", display_color_format);
        return;
    }
    bsp_display_rotation_t display_rotation = bsp_display_get_default_rotation();
    if (display_rotation != BSP_DISPLAY_ROTATION_270) {
        ESP_LOGE(TAG, "fbdraw only supports ROT_270 (got %d)", display_rotation);
        return;
    }

    if (fbdraw_init(&fb, display_h_res, display_v_res) != ESP_OK) {
        ESP_LOGE(TAG, "fbdraw_init failed");
        return;
    }

    // Get input event queue from BSP
    ESP_ERROR_CHECK(bsp_input_get_queue(&input_event_queue));

    // LEDs off
    for (int i = 0; i < 6; i++) {
        bsp_led_set_pixel(i, 0x000000);
    }
    bsp_led_send();
    bsp_led_set_mode(false);

    // Power on the camera (and, as a side effect, the C6 radio). The camera
    // enable line is shared with the C6 radio enable — see camera.md §3.
    splash(WHITE, BLACK, "Camera", "Enabling camera...");
    res = bsp_power_set_radio_state(BSP_POWER_RADIO_STATE_APPLICATION);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to assert camera/radio enable: %d", res);
        splash(RED, WHITE, "Camera error", "Cannot power camera");
        wait_for_esc();
        return;
    }

    // Mount the SD card and create the DCIM folder.
    splash(WHITE, BLACK, "Camera", "Mounting SD card...");
    res = sdcard_init();
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %d", res);
        splash(RED, WHITE, "No SD card", "Insert an SD card and reboot");
        wait_for_esc();
        return;
    }

    if (mkdir(DCIM_PATH, 0777) != 0 && errno != EEXIST) {
        ESP_LOGE(TAG, "Failed to create %s: %s", DCIM_PATH, strerror(errno));
        splash(RED, WHITE, "SD error", "Cannot create /sd/DCIM");
        wait_for_esc();
        return;
    }
    ESP_LOGI(TAG, "%s ready", DCIM_PATH);

    // Detect the OV5647 and configure it for the preview format.
    splash(WHITE, BLACK, "Camera", "Detecting sensor...");
    camera_sensor_t sensor = {0};
    if (camera_sensor_detect(&sensor) != ESP_OK) {
        splash(RED, WHITE, "Camera error", "No sensor detected");
        wait_for_esc();
        return;
    }
    if (camera_sensor_set_format_preview(&sensor, NULL) != ESP_OK) {
        splash(RED, WHITE, "Camera error", "Format not supported");
        wait_for_esc();
        return;
    }
    // set_format wrote the 50 fps default VTS; override it to our
    // target rate before starting the stream. This has to happen
    // AFTER set_format (which would otherwise overwrite it) and can
    // happen BEFORE the CSI pipeline comes up (SCCB writes are
    // independent of the stream).
    if (camera_sensor_set_preview_fps(&sensor, PREVIEW_TARGET_FPS) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to lower preview fps, continuing at native rate");
    }
    // NOTE: set_format leaves the sensor stream OFF. We bring up the CSI
    // pipeline FIRST so the CSI PHY is listening before the sensor starts
    // transmitting frames — starting the sensor while nothing is listening
    // was making the CSI PHY fail to lock on subsequent frames.

    // Layout (user landscape, 800 x 480):
    //
    //   +---------------------------------+------------+
    //   |                                 |            |
    //   |            BLACK BAR            |            |
    //   |  +---------------------------+  |    HUD     |
    //   |  |                           |  |  (600,0)-  |
    //   |  |     16:9 camera feed      |  |  (799,479) |
    //   |  |      (600 x 337)          |  |            |
    //   |  +---------------------------+  |            |
    //   |            BLACK BAR            |            |
    //   +---------------------------------+------------+
    //
    // Sensor is 1920 x 1080 (16:9). Preview area is 600 x 480 (5:4),
    // so we letterbox: pick the largest PPA 1/16 scale that fits
    // 1920 x 1080 inside 600 x 480. That's 5/16 (= 0.3125 exact),
    // which gives 600 x 337.5 → 600 x 337 content in user view, with
    // (480-337)/2 = 71 pixels of black bar above and below.
    //
    // The HUD strip on the right stays the same: 200 px wide, full
    // height.
    const uint32_t logical_w       = fb.user_w;                  // 800
    const uint32_t logical_h       = fb.user_h;                  // 480
    const uint32_t preview_area_w  = 600;
    const uint32_t preview_area_h  = logical_h;                  // 480
    const uint32_t hud_area_x      = preview_area_w;             // 600
    const uint32_t hud_area_w      = logical_w - preview_area_w; // 200

    if (camera_preview_start(&SRC_PHOTO, preview_area_w, preview_area_h) != ESP_OK) {
        splash(RED, WHITE, "Camera error", "Pipeline start failed");
        wait_for_esc();
        return;
    }

    // Now that the CSI PHY, ISP and DMA are all primed and waiting,
    // actually tell the sensor to start streaming.
    if (camera_sensor_stream(&sensor, true) != ESP_OK) {
        splash(RED, WHITE, "Camera error", "Stream start failed");
        wait_for_esc();
        return;
    }

    app_mode_t mode          = MODE_PHOTO;
    bool       space_pending = false;

    // Banner state for brief on-screen messages after a save.
    char       banner_text[64] = {0};
    TickType_t banner_until    = 0;

    // Helper lambda-ish: a transient banner with a default 2.5s duration.
    #define SHOW_BANNER(fmt, ...) do { \
        snprintf(banner_text, sizeof(banner_text), fmt, ##__VA_ARGS__); \
        banner_until = xTaskGetTickCount() + pdMS_TO_TICKS(2500); \
    } while (0)

    while (1) {
        // Drain any queued input events. Mode switches apply immediately;
        // space is latched so capture runs AFTER the blit of the current
        // frame (so the user gets visual feedback that the shutter fired).
        bsp_input_event_t event;
        while (xQueueReceive(input_event_queue, &event, 0) == pdTRUE) {
            if (event.type == INPUT_EVENT_TYPE_NAVIGATION && event.args_navigation.state) {
                switch (event.args_navigation.key) {
                    case BSP_INPUT_NAVIGATION_KEY_ESC:
                        if (video_is_recording()) {
                            video_record_stop();
                        }
                        viewer_close();
                        bsp_device_restart_to_launcher();
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_F1:
                        if (video_is_recording()) break;  // stop recording first
                        if (mode != MODE_VIEW) {
                            // Entering view mode: scan DCIM and decode the
                            // newest image. The live preview keeps running
                            // in the background so we can bounce back to
                            // photo/video mode instantly.
                            // Viewer fits inside the preview area only — HUD
                            // keeps its right-hand strip in view mode too.
                            esp_err_t verr = viewer_open(DCIM_PATH, preview_area_w, preview_area_h);
                            if (verr == ESP_OK) {
                                mode = MODE_VIEW;
                            } else if (verr == ESP_ERR_NOT_FOUND) {
                                SHOW_BANNER("No photos in /sd/DCIM");
                            } else {
                                SHOW_BANNER("Viewer open failed (%d)", verr);
                            }
                        }
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_F2:
                        if (video_is_recording()) break;  // finish recording first
                        if (mode == MODE_VIEW) viewer_close();
                        if (mode != MODE_PHOTO) {
                            // Coming back from VIDEO: re-bring the
                            // pipeline up on the 1920x1080 RAW10
                            // preset so subsequent photos are sharp.
                            esp_err_t serr = switch_pipeline_to_source(
                                &sensor, &SRC_PHOTO, false,
                                preview_area_w, preview_area_h);
                            if (serr != ESP_OK) {
                                SHOW_BANNER("Mode switch failed (%d)", serr);
                                break;
                            }
                        }
                        mode = MODE_PHOTO;
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_F3:
                        if (video_is_recording()) break;  // already in video mode
                        if (mode == MODE_VIEW) viewer_close();
                        if (mode != MODE_VIDEO) {
                            // Swap to 800x640 RAW8 so the PPA has
                            // enough headroom for preview + YUV420
                            // recording scale inside 15 fps budget.
                            esp_err_t serr = switch_pipeline_to_source(
                                &sensor, &SRC_VIDEO, true,
                                preview_area_w, preview_area_h);
                            if (serr != ESP_OK) {
                                SHOW_BANNER("Mode switch failed (%d)", serr);
                                break;
                            }
                        }
                        mode = MODE_VIDEO;
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_LEFT:
                        if (mode == MODE_VIEW) {
                            // ◄ = newer. Ends at index 0.
                            viewer_prev();
                        }
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_RIGHT:
                        if (mode == MODE_VIEW) {
                            // ► = older. Ends at last index.
                            viewer_next();
                        }
                        break;

                    default:
                        break;
                }
            }
            if (event.type == INPUT_EVENT_TYPE_KEYBOARD && event.args_keyboard.ascii == ' ') {
                space_pending = true;
            }
        }

        // Pull one frame from the preview pipeline in photo/video mode.
        // View mode ignores camera frames — it renders the decoded JPEG
        // instead — but we still need the preview pipeline to keep running
        // so entering photo/video mode is instant.
        bool got_preview_frame = false;
        int64_t t_wait_start = esp_timer_get_time();
        if (mode != MODE_VIEW) {
            if (camera_preview_wait_frame(100) == ESP_OK) {
                got_preview_frame = true;
            } else {
                // No fresh frame — skip this iteration so we don't redraw
                // an identical HUD over a stale preview 10 times a second.
                continue;
            }
        }
        int64_t t_after_wait = esp_timer_get_time();

        // Only clear the HUD strip on every frame — the preview area
        // is immediately overwritten by the blit and the HUD strip is
        // the only region that needs a black background beneath the
        // translucent overlays. Cuts ~8 ms off the old full clear.
        fbdraw_fill_rect(&fb, (int)hud_area_x, 0,
                         (int)hud_area_w, (int)logical_h, BLACK);
        int64_t t_after_bg = esp_timer_get_time();

        if (mode == MODE_VIEW && viewer_has_image()) {
            // Also clear the preview area in view mode so stale camera
            // pixels from before entering view mode don't show through
            // the letterbox margins around the decoded JPEG.
            fbdraw_fill_rect(&fb, 0, 0,
                             (int)preview_area_w, (int)preview_area_h, BLACK);
            int img_x = ((int)preview_area_w - (int)viewer_get_width())  / 2;
            int img_y = ((int)preview_area_h - (int)viewer_get_height()) / 2;
            // Viewer still emits landscape-oriented RGB565 via PAX —
            // fall back to the CPU rotation copy until viewer.c is
            // ported to produce panel-native output.
            fbdraw_blit_rotated(&fb, img_x, img_y,
                                (const uint16_t *)viewer_get_pixels(),
                                viewer_get_width(), viewer_get_height());
        } else if (mode != MODE_VIEW) {
            // camera_pipeline's PPA produced a 16:9 preview letterboxed
            // into the 5:4 (600x480) preview area. The PPA output is
            // stored panel-native: pw columns × ph rows, where pw is
            // the narrower dimension (337 at 5/16 scale) and ph covers
            // the full preview-area width in user space (600). Center
            // it horizontally in the 480-wide panel range, which
            // corresponds to vertical centring in the user's view.
            uint32_t pw = camera_preview_get_width();
            uint32_t ph = camera_preview_get_height();
            int panel_x_off = ((int)fb.panel_w - (int)pw) / 2;
            if (panel_x_off < 0) panel_x_off = 0;

            // Clear the top and bottom letterbox bars so stale content
            // from a previous frame or from view mode doesn't show.
            int top_bar_h = (int)fb.user_h - (panel_x_off + (int)pw);
            int bot_bar_y = (int)fb.user_h - panel_x_off;
            int bot_bar_h = panel_x_off;
            if (top_bar_h > 0) {
                fbdraw_fill_rect(&fb, 0, 0,
                                 (int)preview_area_w, top_bar_h, BLACK);
            }
            if (bot_bar_h > 0) {
                fbdraw_fill_rect(&fb, 0, bot_bar_y,
                                 (int)preview_area_w, bot_bar_h, BLACK);
            }

            fbdraw_blit_panel(&fb, panel_x_off, 0,
                              (const uint16_t *)camera_preview_get_pixels(),
                              pw, ph);
        }
        int64_t t_after_image = esp_timer_get_time();

        // HUD strip on the right. All coordinates are in the 200-pixel
        // wide user-space strip starting at hud_area_x.
        const int hud_pad_x = (int)hud_area_x + 10;
        const int hud_font  = 16;
        const int hud_line  = hud_font + 4;
        int hud_y = 14;

        // Mode header in a contrasting colour.
        fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y,
                              mode_name(mode), 20);
        hud_y += 28;

        // Key hint lines.
        fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "F1 view",  hud_font); hud_y += hud_line;
        fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "F2 photo", hud_font); hud_y += hud_line;
        fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "F3 video", hud_font); hud_y += hud_line;
        fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "Esc exit", hud_font); hud_y += hud_line + 6;

        // Mode-specific hint and state.
        switch (mode) {
            case MODE_PHOTO:
                fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "SPACE photo", hud_font);
                hud_y += hud_line;
                break;
            case MODE_VIDEO:
                if (video_is_recording()) {
                    uint32_t ms = video_recording_duration_ms();
                    uint32_t s  = ms / 1000u;
                    char rec_line[32];
                    snprintf(rec_line, sizeof(rec_line), "REC %02u:%02u",
                             (unsigned)(s / 60u), (unsigned)(s % 60u));
                    fbdraw_hershey_string(&fb, RED, hud_pad_x, hud_y, rec_line, hud_font);
                    hud_y += hud_line;
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "SPACE stop", hud_font);
                    hud_y += hud_line;
                } else {
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "SPACE rec", hud_font);
                    hud_y += hud_line;
                }
                break;
            case MODE_VIEW:
                if (viewer_has_image()) {
                    char line[32];
                    snprintf(line, sizeof(line), "%d / %d",
                             viewer_get_index() + 1, viewer_get_total());
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, line, hud_font);
                    hud_y += hud_line;
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "< newer", hud_font); hud_y += hud_line;
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "> older", hud_font); hud_y += hud_line;
                } else {
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "no pics", hud_font);
                    hud_y += hud_line;
                }
                break;
        }

        // Transient banner (e.g. "Saved IMG_xxx.jpg"). Draw it in the
        // bottom of the HUD strip so it doesn't cover the mode/key
        // hints.
        if (banner_text[0] && xTaskGetTickCount() < banner_until) {
            int banner_y = (int)logical_h - 60;
            fbdraw_fill_rect(&fb, (int)hud_area_x, banner_y,
                             (int)hud_area_w, 50, BANNER_BG);
            // Banner text may be too wide for the 200-px HUD strip, so
            // trim to the first 14 chars and hope for the best. GCC
            // warns on a naive snprintf("%s") here because the source
            // is 64 bytes wide, so truncate manually with %.14s.
            fbdraw_hershey_string(&fb, WHITE, hud_pad_x, banner_y + 16,
                                  banner_text, hud_font);
            (void)banner_y;
        } else {
            banner_text[0] = 0;
        }
        int64_t t_after_hud = esp_timer_get_time();

        blit();
        int64_t t_after_blit = esp_timer_get_time();

        // Dump per-stage timings every ~1 s so we can see which step
        // dominates. Remove once the main loop is fast enough.
        static int prof_n = 0;
        if (++prof_n >= 15) {
            prof_n = 0;
            ESP_LOGI(TAG, "loop: wait=%lld bg=%lld img=%lld hud=%lld blit=%lld (us)",
                     (long long)(t_after_wait  - t_wait_start),
                     (long long)(t_after_bg    - t_after_wait),
                     (long long)(t_after_image - t_after_bg),
                     (long long)(t_after_hud   - t_after_image),
                     (long long)(t_after_blit  - t_after_hud));
        }

        // Photo capture: the preview pipeline is torn down and rebuilt
        // at 1920x1080 inside photo_capture(), so the UI is frozen on
        // the last blitted frame for most of a second.
        if (space_pending) {
            space_pending = false;
            if (mode == MODE_PHOTO) {
                char saved_path[128] = {0};
                esp_err_t err = photo_capture(&sensor, preview_area_w, preview_area_h,
                                              PREVIEW_TARGET_FPS,
                                              DCIM_PATH,
                                              saved_path, sizeof(saved_path));
                if (err == ESP_OK && saved_path[0]) {
                    const char *basename = strrchr(saved_path, '/');
                    basename = basename ? basename + 1 : saved_path;
                    SHOW_BANNER("Saved %.48s", basename);
                } else {
                    SHOW_BANNER("Capture failed (%d)", err);
                }
            } else if (mode == MODE_VIDEO) {
                if (video_is_recording()) {
                    esp_err_t err = video_record_stop();
                    if (err == ESP_OK) {
                        SHOW_BANNER("Recording stopped");
                    } else {
                        SHOW_BANNER("Stop failed (%d)", err);
                    }
                } else {
                    char vid_path[128] = {0};
                    esp_err_t err = video_record_start(DCIM_PATH, vid_path, sizeof(vid_path));
                    if (err == ESP_OK && vid_path[0]) {
                        const char *basename = strrchr(vid_path, '/');
                        basename = basename ? basename + 1 : vid_path;
                        SHOW_BANNER("Rec %.48s", basename);
                    } else {
                        SHOW_BANNER("Rec start failed (%d)", err);
                    }
                }
            }
        }

        // The UI is done with s_preview_buffer (if we actually consumed
        // a frame this iteration). Releasing render_ready lets the render
        // task start the next PPA transform. In view mode we never took
        // a frame, so nothing to release — the render task will keep
        // dropping frames while we show the decoded JPEG.
        if (got_preview_frame) {
            camera_preview_give_render_ready();
        }

        // View mode doesn't block on camera frames, so throttle the loop
        // manually to ~30 Hz so we aren't spinning at 100% CPU while
        // showing a static image. Key events still arrive on the next
        // iteration within 33 ms which feels responsive.
        if (mode == MODE_VIEW) {
            vTaskDelay(pdMS_TO_TICKS(33));
        }
    }
}
