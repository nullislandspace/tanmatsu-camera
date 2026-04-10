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
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hal/lcd_types.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "pax_gfx.h"
#include "pax_shapes.h"
#include "hershey_font.h"
#include "bmp_writer.h"
#include "camera_pipeline.h"
#include "camera_sensor.h"
#include "photo.h"
#include "sdcard.h"
#include "usb_device.h"
#include "viewer.h"

#define DCIM_PATH "/sd/DCIM"

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
static pax_buf_t                    fb                   = {0};
static QueueHandle_t                input_event_queue    = NULL;

#define BLACK 0xFF000000
#define WHITE 0xFFFFFFFF
#define RED   0xFFFF0000

static void blit(void) {
    bsp_display_blit(0, 0, display_h_res, display_v_res, pax_buf_get_pixels(&fb));
}

static void splash(pax_col_t bg, pax_col_t fg, const char *line1, const char *line2) {
    pax_background(&fb, bg);
    hershey_draw_string(&fb, fg, 16, 16, line1, 24);
    if (line2) {
        hershey_draw_string(&fb, fg, 16, 48, line2, 18);
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
    // is lost. Remove (or shorten) this for release builds.
    vTaskDelay(pdMS_TO_TICKS(10000));
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

    // Get display parameters and rotation
    res = bsp_display_get_parameters(&display_h_res, &display_v_res, &display_color_format, &display_data_endian);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get display parameters: %d", res);
        return;
    }

    // Convert ESP-IDF color format into PAX buffer type
    pax_buf_type_t format = PAX_BUF_24_888RGB;
    switch (display_color_format) {
        case LCD_COLOR_PIXEL_FORMAT_RGB565:
            format = PAX_BUF_16_565RGB;
            break;
        case LCD_COLOR_PIXEL_FORMAT_RGB888:
            format = PAX_BUF_24_888RGB;
            break;
        default:
            break;
    }

    bsp_display_rotation_t display_rotation = bsp_display_get_default_rotation();
    pax_orientation_t      orientation      = PAX_O_UPRIGHT;
    switch (display_rotation) {
        case BSP_DISPLAY_ROTATION_90:
            orientation = PAX_O_ROT_CCW;
            break;
        case BSP_DISPLAY_ROTATION_180:
            orientation = PAX_O_ROT_HALF;
            break;
        case BSP_DISPLAY_ROTATION_270:
            orientation = PAX_O_ROT_CW;
            break;
        case BSP_DISPLAY_ROTATION_0:
        default:
            orientation = PAX_O_UPRIGHT;
            break;
    }

    pax_buf_init(&fb, NULL, display_h_res, display_v_res, format);
    pax_buf_reversed(&fb, display_data_endian == LCD_RGB_DATA_ENDIAN_BIG);
    pax_buf_set_orientation(&fb, orientation);

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
    // NOTE: set_format leaves the sensor stream OFF. We bring up the CSI
    // pipeline FIRST so the CSI PHY is listening before the sensor starts
    // transmitting frames — starting the sensor while nothing is listening
    // was making the CSI PHY fail to lock on subsequent frames.

    // Preview size: fit the 5:4 (800x640) camera frame into the rotated
    // display's LOGICAL coordinate space (after any PAX rotation), keeping
    // aspect ratio. Using pax_buf_get_width/height here — display_h_res and
    // display_v_res are the raw unrotated panel dimensions and are only
    // useful for the final bsp_display_blit() call.
    uint32_t logical_w = pax_buf_get_width(&fb);
    uint32_t logical_h = pax_buf_get_height(&fb);
    uint32_t preview_w = logical_w;
    uint32_t preview_h = (preview_w * 640u) / 800u;
    if (preview_h > logical_h) {
        preview_h = logical_h;
        preview_w = (preview_h * 800u) / 640u;
    }
    if (camera_preview_start(preview_w, preview_h) != ESP_OK) {
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

    // A PAX buffer that aliases the preview pipeline's RGB565 output — no
    // copy, just a different view over the same pixels. pax_draw_image()
    // handles the RGB565 → framebuffer format conversion.
    pax_buf_t preview_pax = {0};
    pax_buf_init(&preview_pax, (void *)camera_preview_get_pixels(),
                 camera_preview_get_width(), camera_preview_get_height(),
                 PAX_BUF_16_565RGB);

    int preview_x = ((int)logical_w - (int)camera_preview_get_width())  / 2;
    int preview_y = ((int)logical_h - (int)camera_preview_get_height()) / 2;

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
                        viewer_close();
                        bsp_device_restart_to_launcher();
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_F1:
                        if (mode != MODE_VIEW) {
                            // Entering view mode: scan DCIM and decode the
                            // newest image. The live preview keeps running
                            // in the background so we can bounce back to
                            // photo/video mode instantly.
                            esp_err_t verr = viewer_open(DCIM_PATH, logical_w, logical_h);
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
                        if (mode == MODE_VIEW) viewer_close();
                        mode = MODE_PHOTO;
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_F3:
                        if (mode == MODE_VIEW) viewer_close();
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
        if (mode != MODE_VIEW) {
            if (camera_preview_wait_frame(100) == ESP_OK) {
                got_preview_frame = true;
            } else {
                // No fresh frame — skip this iteration so we don't redraw
                // an identical HUD over a stale preview 10 times a second.
                continue;
            }
        }

        pax_background(&fb, BLACK);

        if (mode == MODE_VIEW && viewer_has_image()) {
            pax_buf_t img_pax = {0};
            pax_buf_init(&img_pax, (void *)viewer_get_pixels(),
                         viewer_get_width(), viewer_get_height(),
                         PAX_BUF_16_565RGB);
            int img_x = ((int)logical_w - (int)viewer_get_width())  / 2;
            int img_y = ((int)logical_h - (int)viewer_get_height()) / 2;
            pax_draw_image(&fb, &img_pax, img_x, img_y);
        } else if (mode != MODE_VIEW) {
            // preview_pax aliases s_preview_buffer (RGB565); PAX handles
            // the RGB565 → display format conversion into fb.
            pax_draw_image(&fb, &preview_pax, preview_x, preview_y);
        }

        // HUD: top bar with mode name and key hints.
        char hud[96];
        snprintf(hud, sizeof(hud), "%s   F1 view  F2 photo  F3 video  Esc exit",
                 mode_name(mode));
        pax_simple_rect(&fb, 0xC0000000, 0, 0, (float)logical_w, 22);
        hershey_draw_string(&fb, WHITE, 8, 3, hud, 18);

        // Context-specific bottom bar.
        char bottom_buf[96];
        const char *bottom_hint = "";
        switch (mode) {
            case MODE_PHOTO:
                bottom_hint = "SPACE = take photo";
                break;
            case MODE_VIDEO:
                bottom_hint = "SPACE = start/stop record  (not implemented)";
                break;
            case MODE_VIEW:
                if (viewer_has_image()) {
                    snprintf(bottom_buf, sizeof(bottom_buf),
                             "%d / %d  %.48s   <- newer  -> older",
                             viewer_get_index() + 1, viewer_get_total(),
                             viewer_get_filename());
                    bottom_hint = bottom_buf;
                } else {
                    bottom_hint = "(no images)";
                }
                break;
        }
        int bottom_y = (int)logical_h - 22;
        pax_simple_rect(&fb, 0xC0000000, 0, (float)bottom_y, (float)logical_w, 22);
        hershey_draw_string(&fb, WHITE, 8, (float)(bottom_y + 3), bottom_hint, 18);

        // Transient banner (e.g. "Saved IMG_xxx.jpg").
        if (banner_text[0] && xTaskGetTickCount() < banner_until) {
            pax_simple_rect(&fb, 0xC0000000, 0, 28, (float)logical_w, 22);
            hershey_draw_string(&fb, WHITE, 8, 31, banner_text, 18);
        } else {
            banner_text[0] = 0;
        }

        blit();

        // Photo capture: the preview pipeline is torn down and rebuilt
        // at 1920x1080 inside photo_capture(), so the UI is frozen on
        // the last blitted frame for most of a second.
        if (space_pending) {
            space_pending = false;
            if (mode == MODE_PHOTO) {
                char saved_path[128] = {0};
                esp_err_t err = photo_capture(&sensor, preview_w, preview_h,
                                              DCIM_PATH,
                                              saved_path, sizeof(saved_path));
                if (err == ESP_OK && saved_path[0]) {
                    const char *basename = strrchr(saved_path, '/');
                    basename = basename ? basename + 1 : saved_path;
                    SHOW_BANNER("Saved %.48s", basename);
                } else {
                    SHOW_BANNER("Capture failed (%d)", err);
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
