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
#include "nvs_flash.h"
#include "pax_fonts.h"
#include "pax_gfx.h"
#include "pax_text.h"
#include "pax_shapes.h"
#include "bmp_writer.h"
#include "camera_pipeline.h"
#include "camera_sensor.h"
#include "sdcard.h"
#include "usb_device.h"

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
    pax_draw_text(&fb, fg, pax_font_sky_mono, 20, 16, 16, line1);
    if (line2) {
        pax_draw_text(&fb, fg, pax_font_sky_mono, 16, 16, 48, line2);
    }
    blit();
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

    // Initialize the Board Support Package
    const bsp_configuration_t bsp_configuration = {
        .display =
            {
                .requested_color_format = LCD_COLOR_PIXEL_FORMAT_RGB888,
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

    while (1) {
        // Drain any queued input events. Mode switches apply on the next
        // frame; space is latched so the snapshot runs after the blit.
        bsp_input_event_t event;
        while (xQueueReceive(input_event_queue, &event, 0) == pdTRUE) {
            if (event.type == INPUT_EVENT_TYPE_NAVIGATION && event.args_navigation.state) {
                switch (event.args_navigation.key) {
                    case BSP_INPUT_NAVIGATION_KEY_ESC:
                        bsp_device_restart_to_launcher();
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_F1:
                        mode = MODE_VIEW;
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_F2:
                        mode = MODE_PHOTO;
                        break;
                    case BSP_INPUT_NAVIGATION_KEY_F3:
                        mode = MODE_VIDEO;
                        break;
                    default:
                        break;
                }
            }
            if (event.type == INPUT_EVENT_TYPE_KEYBOARD && event.args_keyboard.ascii == ' ') {
                space_pending = true;
            }
        }

        if (camera_preview_wait_frame(100) != ESP_OK) {
            continue;
        }

        // Draw the live preview into the rotated PAX framebuffer. The
        // preview_pax buffer aliases s_preview_buffer (RGB565); PAX handles
        // the RGB565 → RGB888 conversion into fb.
        pax_background(&fb, BLACK);
        pax_draw_image(&fb, &preview_pax, preview_x, preview_y);

        // HUD: top bar with mode name and key hints.
        char hud[96];
        snprintf(hud, sizeof(hud), "%s   F1 view  F2 photo  F3 video  Esc exit",
                 mode_name(mode));
        pax_simple_rect(&fb, 0xC0000000, 0, 0, (float)logical_w, 22);
        pax_draw_text(&fb, WHITE, pax_font_sky_mono, 16, 8, 3, hud);

        // Context-specific bottom bar.
        const char *bottom_hint = "";
        switch (mode) {
            case MODE_PHOTO: bottom_hint = "SPACE = snapshot (BMP debug dump)"; break;
            case MODE_VIDEO: bottom_hint = "SPACE = start/stop record  (not implemented)"; break;
            case MODE_VIEW:  bottom_hint = "<- / -> navigate  (not implemented)"; break;
        }
        int bottom_y = (int)logical_h - 22;
        pax_simple_rect(&fb, 0xC0000000, 0, (float)bottom_y, (float)logical_w, 22);
        pax_draw_text(&fb, WHITE, pax_font_sky_mono, 16, 8, (float)(bottom_y + 3), bottom_hint);

        // Transient banner (e.g. "Saved IMG_xxx.bmp").
        if (banner_text[0] && xTaskGetTickCount() < banner_until) {
            pax_simple_rect(&fb, 0xC0000000, 0, 28, (float)logical_w, 22);
            pax_draw_text(&fb, WHITE, pax_font_sky_mono, 16, 8, 31, banner_text);
        } else {
            banner_text[0] = 0;
        }

        blit();

        // The UI is done with s_preview_buffer. Release the back-pressure
        // gate so the render task can start the next PPA transform.
        camera_preview_give_render_ready();

        // Act on space AFTER the blit so the banner can show on the next
        // frame. snapshot_dbg uses the same s_preview_buffer we just drew,
        // plus a racy best-effort dump of s_camera_buffer (pre-PPA).
        if (space_pending) {
            space_pending = false;
            if (mode == MODE_PHOTO) {
                time_t    now     = time(NULL);
                struct tm tmv;
                localtime_r(&now, &tmv);
                char ts[32];
                strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tmv);

                char path[128];
                snprintf(path, sizeof(path), "%s/dbg_prev_%s.bmp", DCIM_PATH, ts);
                esp_err_t e1 = bmp_writer_save_rgb565(path,
                    (const uint16_t *)camera_preview_get_pixels(),
                    camera_preview_get_width(), camera_preview_get_height());

                snprintf(path, sizeof(path), "%s/dbg_full_%s.bmp", DCIM_PATH, ts);
                esp_err_t e2 = bmp_writer_save_rgb565(path,
                    (const uint16_t *)camera_preview_get_raw_pixels(),
                    camera_preview_get_raw_width(), camera_preview_get_raw_height());

                if (e1 == ESP_OK && e2 == ESP_OK) {
                    snprintf(banner_text, sizeof(banner_text), "Saved dbg_prev/full_%s.bmp", ts);
                } else {
                    snprintf(banner_text, sizeof(banner_text), "Save failed (%d / %d)", e1, e2);
                }
                banner_until = xTaskGetTickCount() + pdMS_TO_TICKS(2500);
            }
        }
    }
}
