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
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
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
#include "config.h"
#include "focus/autofocus.h"
#include "focus/focus_driver.h"
#include "fbdraw.h"
#include "photo.h"
#include "icons.h"
#include "intfs.h"
#include "microphone.h"
#include "sdcard.h"
#include "splash.h"
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

// Source descriptors for the OV5647 PHOTO ↔ VIDEO transitions. PHOTO
// uses the sensor's highest-res MIPI mode so preview and still capture
// can share a single WYSIWYG frame. VIDEO drops to 800x640 RAW8 so
// the PPA has enough headroom to do both the preview scale AND the
// YUV420 recording scale inside a 15 fps frame budget (1920x1080
// blew past ~83 ms per PPA op, turning the whole pipeline into a
// 6-fps slideshow). See camera.md §9 for the bandwidth math.
//
// OV5640/OV5645 use a single shared RGB565 source for all modes
// (built dynamically below from the active esp_cam_sensor_format_t)
// so the PHOTO↔VIDEO transition is a no-op for those sensors.
static const camera_source_t SRC_OV5647_PHOTO = {
    .width          = 1920,
    .height         = 1080,
    .input_format   = CAMERA_INPUT_RAW10,
    .lane_rate_mbps = 500,
};
static const camera_source_t SRC_OV5647_VIDEO = {
    .width          = 800,
    .height         = 640,
    .input_format   = CAMERA_INPUT_RAW8,
    .lane_rate_mbps = 200,
};

// Build a camera_source_t from the format the sensor driver just
// reported back via set_format_*. Used for OV5640/OV5645 where the
// dimensions and lane rate live entirely on the format struct so
// there's nothing to hardcode.
static camera_source_t source_from_format(const esp_cam_sensor_format_t *fmt,
                                          camera_input_format_t input_format) {
    camera_source_t src = {
        .width          = fmt->width,
        .height         = fmt->height,
        .input_format   = input_format,
        // mipi_clk in the format struct is the per-lane bit rate in
        // Hz; the CSI controller config wants the same value in Mbps.
        .lane_rate_mbps = fmt->mipi_info.mipi_clk / 1000000u,
    };
    return src;
}

// Pick the camera_source_t matching (sensor_kind, mode). For the
// OV5647 we return the static SRC_* descriptors; for everything else
// we synthesise it from the format the driver just bound to.
// Caller must have run camera_sensor_set_format_*() immediately
// before so `fmt` reflects the active format.
static camera_source_t pick_source(camera_sensor_kind_t kind,
                                   bool is_video_mode,
                                   const esp_cam_sensor_format_t *fmt) {
    switch (kind) {
        case CAMERA_SENSOR_OV5647:
            return is_video_mode ? SRC_OV5647_VIDEO : SRC_OV5647_PHOTO;
        case CAMERA_SENSOR_OV9281:
            // Monochrome RAW10 — reuse the RAW10 demosaic path. The
            // P4 ISP will produce roughly grayscale RGB565 because
            // every input pixel carries the same luminance value
            // regardless of which Bayer position the demosaicer
            // assumes, with mild interpolation artifacts at edges.
            return source_from_format(fmt, CAMERA_INPUT_RAW10);
        case CAMERA_SENSOR_OV5640:
        case CAMERA_SENSOR_OV5645:
        default:
            // YUV-class sensors deliver RGB565. Unknown sensors fall
            // through here too — if the detect loop bound to something
            // we don't recognise we assume RGB565 input rather than
            // RAW Bayer because the ISP-bypass path is the safer
            // default (no chance of demosaicing non-Bayer data into
            // garbage).
            return source_from_format(fmt, CAMERA_INPUT_RGB565);
    }
}

typedef enum {
    MODE_PHOTO = 0,
    MODE_VIDEO,
    MODE_VIEW,
    MODE_CONFIG,
} app_mode_t;

static const char *mode_name(app_mode_t m) {
    switch (m) {
        case MODE_PHOTO:  return "PHOTO";
        case MODE_VIDEO:  return "VIDEO";
        case MODE_VIEW:   return "VIEW";
        case MODE_CONFIG: return "CONFIG";
    }
    return "?";
}

// Focus subsystem runtime state. Position is session-only — each boot
// starts at mid-range so the user isn't fighting whatever the previous
// session left the VCM at (e.g. after a lens swap). The driver itself
// (real chip vs. simulator vs. off) is selected via the config menu;
// see focus_driver.h.
static camera_config_t g_cfg           = {0};
static bool            g_focus_present = false;  // active driver probed OK
static uint16_t        g_focus_pos     = 512;
static int             g_focus_dir     = 0;      // -1, 0, +1
static int64_t         g_focus_last_us = 0;

// Config menu items. Tagged struct so a single table can hold boolean
// checkboxes, a string-cycler for the focus driver name, and bounded
// integer cyclers (e.g. mic gain).
typedef enum {
    CFG_KIND_BOOL,
    CFG_KIND_DRIVER,
    CFG_KIND_INT,
} cfg_kind_t;

typedef struct {
    const char *label;
    cfg_kind_t  kind;
    void       *target;  // bool* (BOOL) / char[] (DRIVER) / int* (INT)
    int         imin;    // CFG_KIND_INT only — inclusive lower bound
    int         imax;    // CFG_KIND_INT only — inclusive upper bound
} cfg_item_t;

static cfg_item_t g_cfg_items[] = {
    { "Driver",     CFG_KIND_DRIVER, g_cfg.focus_driver,       0, 0 },
    { "Focus",      CFG_KIND_BOOL,   &g_cfg.focus_enabled,     0, 0 },
    { "Autofocus",  CFG_KIND_BOOL,   &g_cfg.autofocus_enabled, 0, 0 },
    { "Rotate 180", CFG_KIND_BOOL,   &g_cfg.rotate_180,        0, 0 },
    { "Microphone", CFG_KIND_BOOL,   &g_cfg.mic_enabled,       0, 0 },
    { "Mic Gain",   CFG_KIND_INT,    &g_cfg.mic_gain,
                    CONFIG_MIC_GAIN_MIN, CONFIG_MIC_GAIN_MAX },
};
#define CFG_ITEM_COUNT ((int)(sizeof(g_cfg_items) / sizeof(g_cfg_items[0])))
static int g_cfg_sel = 0;

// Item is selectable / can be toggled? Autofocus is greyed out unless
// a focus driver is currently active (Focus on + chip probed OK).
static bool cfg_item_enabled(const cfg_item_t *it) {
    if (it->target == &g_cfg.autofocus_enabled) return g_focus_present;
    return true;
}

typedef enum {
    CFG_ACT_NOOP,           // disabled row, nothing happened
    CFG_ACT_CHANGED,        // value changed; please save
    CFG_ACT_PROBE_FAILED,   // driver probe rejected; show banner
} cfg_act_result_t;

// RETURN/SPACE on a config row. Bool rows toggle, the driver row
// cycles to the next entry in focus_driver_registry[].
static cfg_act_result_t cfg_item_activate(int idx) {
    cfg_item_t *it = &g_cfg_items[idx];
    if (!cfg_item_enabled(it)) return CFG_ACT_NOOP;

    if (it->kind == CFG_KIND_DRIVER) {
        char *target = (char *)it->target;
        const focus_driver_t *cur = focus_driver_by_name(target);
        int n = focus_driver_count();
        int cur_idx = -1;
        for (int i = 0; i < n; i++) {
            if (focus_driver_by_index(i) == cur) { cur_idx = i; break; }
        }
        const focus_driver_t *next = focus_driver_by_index((cur_idx + 1) % n);

        // Cycling the driver always force-disables Focus + Autofocus
        // so the user has to explicitly opt in to the new chip. This
        // sidesteps the messy "what if the new driver fails to probe
        // mid-swap" path entirely — toggling Focus back on does a
        // clean probe and a clear banner if the new driver isn't
        // there.
        if (g_cfg.focus_enabled || g_focus_present) {
            focus_select("");
            g_cfg.focus_enabled     = false;
            g_focus_present         = false;
            g_focus_dir             = 0;
            g_cfg.autofocus_enabled = false;
            autofocus_set_enabled(false);
        }
        strncpy(target, next->name, CONFIG_FOCUS_DRIVER_MAXLEN - 1);
        target[CONFIG_FOCUS_DRIVER_MAXLEN - 1] = '\0';
        return CFG_ACT_CHANGED;
    }

    if (it->kind == CFG_KIND_INT) {
        int *iv   = (int *)it->target;
        int  prev = *iv;
        int  next = prev + 1;
        if (next > it->imax) next = it->imin;  // wrap
        *iv = next;
        if (iv == &g_cfg.mic_gain) {
            microphone_set_gain(*iv);
        }
        return (*iv != prev) ? CFG_ACT_CHANGED : CFG_ACT_NOOP;
    }

    // CFG_KIND_BOOL
    bool *v    = (bool *)it->target;
    bool  prev = *v;
    *v         = !prev;

    if (v == &g_cfg.focus_enabled) {
        if (*v) {
            if (focus_select(g_cfg.focus_driver) == ESP_OK) {
                g_focus_pos     = focus_active()->pos_default;
                g_focus_present = true;
            } else {
                *v = false;  // revert
                return CFG_ACT_PROBE_FAILED;
            }
        } else {
            focus_select("");
            g_focus_present         = false;
            g_focus_dir             = 0;
            // Force autofocus off too — keeps persisted state consistent.
            g_cfg.autofocus_enabled = false;
            autofocus_set_enabled(false);
        }
    } else if (v == &g_cfg.autofocus_enabled) {
        autofocus_set_enabled(g_focus_present && *v);
    } else if (v == &g_cfg.rotate_180) {
        camera_pipeline_set_rotate_180(*v);
    }

    return (*v != prev) ? CFG_ACT_CHANGED : CFG_ACT_NOOP;
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
#define YELLOW      fbdraw_rgb(255, 220, 0)
#define HUD_BG      fbdraw_rgb(32, 32, 32)    // dark grey strip behind HUD text
#define HUD_SEP     fbdraw_rgb(96, 96, 96)    // divider line in HUD
#define BANNER_BG   fbdraw_rgb(0, 0, 128)     // dark blue banner strip
#define MENU_HL_BG  fbdraw_rgb(64, 64, 0)     // dark yellow highlight in menu

// Non-blocking display blit. We bypass bsp_display_blit() and manage
// the DMA-done semaphore ourselves so we can skip frames when the
// display is still busy transferring the previous one — no blocking,
// no tearing (double-buffered fb), and the camera pipeline is never
// stalled by display DMA.
static esp_lcd_panel_handle_t s_panel    = NULL;
static SemaphoreHandle_t      s_blit_done = NULL;

static IRAM_ATTR bool on_blit_done(esp_lcd_panel_handle_t panel,
                                   esp_lcd_dpi_panel_event_data_t *edata,
                                   void *user_ctx) {
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(s_blit_done, &hpw);
    return hpw == pdTRUE;
}

// Blocking blit for one-shot screens (splash, errors) where we don't
// care about frame drops.
static void blit(void) {
    xSemaphoreTake(s_blit_done, pdMS_TO_TICKS(1000));
    esp_lcd_panel_draw_bitmap(s_panel, 0, 0,
                              display_h_res, display_v_res, fb.pixels);
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
// preview format because F1 (viewer) doesn't actually stream from
// the camera, it just reuses the preview pipeline as a backdrop.
//
// On OV5647 the PHOTO and VIDEO modes use distinct sensor presets
// (high-res RAW10 vs low-res RAW8); on OV5640/OV5645 there is a
// single shared RGB565 format and the rebuild is technically
// unnecessary, but we still go through the motions so the pipeline
// state-machine has one path through every transition.
//
// Returns ESP_OK if the transition completes; on failure the pipeline
// is left torn down and the caller is expected to surface an error
// screen / banner to the user.
static esp_err_t switch_pipeline_to_source(camera_sensor_t *sensor,
                                           bool is_video_mode,
                                           uint32_t preview_area_w,
                                           uint32_t preview_area_h) {
    // Stop sensor stream first so the CSI DMA stops firing while we
    // tear down the controller.
    camera_sensor_stream(sensor, false);
    camera_preview_stop();

    // Reconfigure the sensor. On OV5647: PHOTO uses 1920x1080 RAW10
    // capped to PREVIEW_TARGET_FPS via VTS; VIDEO uses 800x640 RAW8
    // and leaves the native 50 fps alone (the video task only pulls
    // at 15 Hz, the extra frames are just wasted CSI DMA at <15 MB/s
    // which is free). On OV5640/OV5645: both modes pick the single
    // RGB565 preset and we cap to PREVIEW_TARGET_FPS unconditionally
    // (a no-op when the native rate is already <= 15).
    esp_cam_sensor_format_t fmt = {0};
    esp_err_t err;
    if (is_video_mode) {
        err = camera_sensor_set_format_video(sensor, &fmt);
        // OV5640/OV5645: the shared format runs at native 14/30 fps;
        // 30 fps is way over our PSRAM bandwidth budget, so apply the
        // cap on the video preset too. OV5647 keeps its native 50 fps
        // for VIDEO mode (see comment above), so only apply the cap
        // on RGB565-class sensors.
        if (err == ESP_OK && sensor->kind != CAMERA_SENSOR_OV5647) {
            if (camera_sensor_set_preview_fps(sensor, PREVIEW_TARGET_FPS) != ESP_OK) {
                ESP_LOGW(TAG, "fps cap on video preset failed");
            }
        }
    } else {
        err = camera_sensor_set_format_preview(sensor, &fmt);
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

    camera_source_t src = pick_source(sensor->kind, is_video_mode, &fmt);
    err = camera_preview_start(&src, preview_area_w, preview_area_h);
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

    // Set up non-blocking display blit. Get the panel handle and
    // register our own DMA-completion callback, replacing the BSP's
    // default (which we no longer need since we call
    // esp_lcd_panel_draw_bitmap directly instead of bsp_display_blit).
    res = bsp_display_get_panel(&s_panel);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "bsp_display_get_panel: %d", res);
        return;
    }
    s_blit_done = xSemaphoreCreateBinary();
    xSemaphoreGive(s_blit_done);  // display starts idle
    esp_lcd_dpi_panel_event_callbacks_t blit_cbs = {
        .on_color_trans_done = on_blit_done,
    };
    res = esp_lcd_dpi_panel_register_event_callbacks(s_panel, &blit_cbs, NULL);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "register blit callbacks: %d", res);
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

    // Mount the wear-levelled internal FAT partition (/int) — the
    // launcher stores HUD icons and other shared assets there. Failure
    // is non-fatal; icons_load() falls back to text labels.
    intfs_init();

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

    // Load function-key icons. Both /sd and /int are now mounted, so
    // the ESC override under /sd/apps/at.cavac.tanmatype/ wins over
    // the launcher default at /int/icons/. Best effort — missing PNGs
    // are tolerated and the HUD draws "F1" / "Esc" text instead.
    icons_load();

    // Load /sd/camera.cfg. Missing file is non-fatal — config_load
    // seeds defaults and writes a template for the user. If the focus
    // subsystem is enabled, activate the configured driver now so we
    // can (a) apply the mid-range starting position and (b) warn via
    // banner if the module isn't actually attached.
    config_load(&g_cfg);
    camera_pipeline_set_rotate_180(g_cfg.rotate_180);
    microphone_set_gain(g_cfg.mic_gain);
    bool show_focus_missing_banner = false;
    if (g_cfg.focus_enabled) {
        if (focus_select(g_cfg.focus_driver) == ESP_OK) {
            g_focus_pos     = focus_active()->pos_default;
            g_focus_present = true;
            ESP_LOGI(TAG, "focus driver '%s' ready, pos=%u",
                     focus_active_name(), g_focus_pos);
        } else {
            ESP_LOGW(TAG, "focus driver '%s' enabled in config but not detected",
                     g_cfg.focus_driver);
            show_focus_missing_banner = true;
        }
    }

    // Show the branded splash screen for 2 seconds (non-fatal if missing).
    splash_show(&fb);

    // Detect the camera sensor and configure it for the preview format.
    // The JPEG splash screen stays visible during detection. The detect
    // loop walks every linker-registered esp_cam_sensor driver, so any
    // of OV5647/OV5640/OV5645 will bind here depending on what's
    // physically wired to the I2C bus.
    camera_sensor_t sensor = {0};
    if (camera_sensor_detect(&sensor) != ESP_OK) {
        splash(RED, WHITE, "Camera error", "No sensor detected");
        wait_for_esc();
        return;
    }
    esp_cam_sensor_format_t initial_fmt = {0};
    if (camera_sensor_set_format_preview(&sensor, &initial_fmt) != ESP_OK) {
        splash(RED, WHITE, "Camera error", "Format not supported");
        wait_for_esc();
        return;
    }
    // set_format wrote the format's default VTS; override it to our
    // target rate before starting the stream. This has to happen
    // AFTER set_format (which would otherwise overwrite it) and can
    // happen BEFORE the CSI pipeline comes up (SCCB writes are
    // independent of the stream). On sensors whose native rate is
    // already <= PREVIEW_TARGET_FPS the override is a no-op.
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

    camera_source_t initial_src = pick_source(sensor.kind, false, &initial_fmt);
    if (camera_preview_start(&initial_src, preview_area_w, preview_area_h) != ESP_OK) {
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

    if (show_focus_missing_banner) {
        snprintf(banner_text, sizeof(banner_text), "%s not detected",
                 g_cfg.focus_driver);
        banner_until = xTaskGetTickCount() + pdMS_TO_TICKS(3500);
    }

    // Apply autofocus enable to match the loaded config now that we
    // know whether the driver is actually present.
    autofocus_set_enabled(g_focus_present && g_cfg.autofocus_enabled);

    // Tracks which mode the user was in before opening the config
    // menu so F4/ESC returns them there. Only meaningful when
    // mode == MODE_CONFIG.
    app_mode_t prev_mode = MODE_PHOTO;

    // The mic runs whenever the user is in video mode (actual or
    // via the config menu entered from it) AND the mic_enabled config
    // bit is set. We reconcile declaratively once per loop iteration
    // so any combination of mode change + config toggle converges
    // without each call-site needing its own start/stop plumbing.
    #define MIC_SHOULD_RUN(m, pm) ( \
        g_cfg.mic_enabled && \
        ((m) == MODE_VIDEO || ((m) == MODE_CONFIG && (pm) == MODE_VIDEO)))

    // Helper lambda-ish: a transient banner with a default 2.5s duration.
    #define SHOW_BANNER(fmt, ...) do { \
        snprintf(banner_text, sizeof(banner_text), fmt, ##__VA_ARGS__); \
        banner_until = xTaskGetTickCount() + pdMS_TO_TICKS(2500); \
    } while (0)

    while (1) {
        // Bring the mic state in line with the current mode + config
        // before any other per-frame work. Start/stop failures are
        // logged by microphone.c; we don't surface them here because
        // the mic is optional hardware — the user can unplug it and
        // we keep falling back to silent audio tracks.
        {
            bool want = MIC_SHOULD_RUN(mode, prev_mode);
            if (want && !microphone_is_running()) {
                microphone_start();
            } else if (!want && microphone_is_running()) {
                microphone_stop();
            }
        }

        // Drain any queued input events. Mode switches apply immediately;
        // space is latched so capture runs AFTER the blit of the current
        // frame (so the user gets visual feedback that the shutter fired).
        bsp_input_event_t event;
        while (xQueueReceive(input_event_queue, &event, 0) == pdTRUE) {
            // UP/DOWN need both press and release so we can implement
            // hold-to-scan focus. Handle those first, before the
            // press-only switch below.
            if (event.type == INPUT_EVENT_TYPE_NAVIGATION &&
                (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_UP ||
                 event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_DOWN)) {
                if (mode == MODE_CONFIG) {
                    if (event.args_navigation.state) {
                        if (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_UP) {
                            g_cfg_sel = (g_cfg_sel - 1 + CFG_ITEM_COUNT) % CFG_ITEM_COUNT;
                        } else {
                            g_cfg_sel = (g_cfg_sel + 1) % CFG_ITEM_COUNT;
                        }
                    }
                } else if (g_focus_present && mode != MODE_VIEW) {
                    int dir = (event.args_navigation.key == BSP_INPUT_NAVIGATION_KEY_UP)
                                ? +1 : -1;
                    if (event.args_navigation.state) {
                        g_focus_dir     = dir;
                        g_focus_last_us = esp_timer_get_time();
                        if (g_cfg.autofocus_enabled) {
                            autofocus_set_manual_override(true);
                        }
                    } else if (g_focus_dir == dir) {
                        g_focus_dir = 0;
                        if (g_cfg.autofocus_enabled) {
                            autofocus_set_manual_override(false);
                        }
                    }
                }
                continue;
            }

            if (event.type == INPUT_EVENT_TYPE_NAVIGATION && event.args_navigation.state) {
                switch (event.args_navigation.key) {
                    case BSP_INPUT_NAVIGATION_KEY_ESC:
                        if (mode == MODE_CONFIG) {
                            mode = prev_mode;
                            break;
                        }
                        if (video_is_recording()) {
                            video_record_stop();
                        }
                        viewer_close();
                        bsp_device_restart_to_launcher();
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_F4:
                        if (video_is_recording()) break;
                        if (mode == MODE_CONFIG) {
                            mode = prev_mode;
                        } else {
                            prev_mode = mode;
                            mode      = MODE_CONFIG;
                            g_cfg_sel = 0;
                        }
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_RETURN:
                        if (mode == MODE_CONFIG && CFG_ITEM_COUNT > 0) {
                            cfg_act_result_t r = cfg_item_activate(g_cfg_sel);
                            if (r == CFG_ACT_CHANGED) {
                                config_save(&g_cfg);
                            } else if (r == CFG_ACT_PROBE_FAILED) {
                                SHOW_BANNER("%s not detected", g_cfg.focus_driver);
                            }
                        } else if (mode == MODE_VIDEO && !video_is_recording()) {
                            // Debug hook: dump 5 s of raw I2S samples
                            // to /sd/miccal.wav for offline analysis.
                            // The main loop is blocked during the
                            // dump — fine for a temporary diagnostic.
                            if (!microphone_is_running()) {
                                SHOW_BANNER("Enable mic in config first");
                            } else {
                                esp_err_t derr = microphone_debug_raw_dump(
                                    "/sd/miccal.wav", 5);
                                if (derr == ESP_OK) {
                                    SHOW_BANNER("Saved /sd/miccal.wav");
                                } else {
                                    SHOW_BANNER("Dump failed: %d", derr);
                                }
                            }
                        }
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
                            // pipeline up on the preview format. On
                            // OV5647 that's the 1920x1080 RAW10 preset
                            // so subsequent photos are sharp; on
                            // OV5640/OV5645 it's the same shared RGB565
                            // format we were already on, so this is a
                            // pipeline rebuild but no sensor-side
                            // format change.
                            esp_err_t serr = switch_pipeline_to_source(
                                &sensor, false,
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
                            // OV5647: swap to 800x640 RAW8 so the PPA
                            // has enough headroom for preview + YUV420
                            // recording scale inside 15 fps budget.
                            // OV5640/OV5645: same shared RGB565 format
                            // as PHOTO mode (those sensors don't have
                            // a separate low-res video preset).
                            esp_err_t serr = switch_pipeline_to_source(
                                &sensor, true,
                                preview_area_w, preview_area_h);
                            if (serr != ESP_OK) {
                                SHOW_BANNER("Mode switch failed (%d)", serr);
                                break;
                            }
                        }
                        mode = MODE_VIDEO;
                        break;

                    case BSP_INPUT_NAVIGATION_KEY_VOLUME_UP:
                    case BSP_INPUT_NAVIGATION_KEY_VOLUME_DOWN:
                        // Live mic-gain trim in video mode (preview +
                        // recording). Clamp to the configured range,
                        // push the new value into the mic task, and
                        // persist to /sd/camera.cfg so the next boot
                        // starts at the user's preferred level.
                        if (mode == MODE_VIDEO) {
                            int delta = (event.args_navigation.key ==
                                         BSP_INPUT_NAVIGATION_KEY_VOLUME_UP)
                                            ? +1 : -1;
                            int newg  = g_cfg.mic_gain + delta;
                            if (newg < CONFIG_MIC_GAIN_MIN) newg = CONFIG_MIC_GAIN_MIN;
                            if (newg > CONFIG_MIC_GAIN_MAX) newg = CONFIG_MIC_GAIN_MAX;
                            if (newg != g_cfg.mic_gain) {
                                g_cfg.mic_gain = newg;
                                microphone_set_gain(newg);
                                config_save(&g_cfg);
                            }
                        }
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
                if (mode == MODE_CONFIG) {
                    if (CFG_ITEM_COUNT > 0) {
                        cfg_act_result_t r = cfg_item_activate(g_cfg_sel);
                        if (r == CFG_ACT_CHANGED) {
                            config_save(&g_cfg);
                        } else if (r == CFG_ACT_PROBE_FAILED) {
                            SHOW_BANNER("%s not detected", g_cfg.focus_driver);
                        }
                    }
                } else {
                    space_pending = true;
                }
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

        // Skip drawing + blit entirely if the display is still DMA-ing
        // the previous frame. With double buffering this means we just
        // drop a display frame — the camera pipeline and input handling
        // are never stalled by display DMA.
        bool display_ready = (xSemaphoreTake(s_blit_done, 0) == pdTRUE);
        int64_t t_after_bg    = t_after_wait;
        int64_t t_after_image = t_after_wait;
        int64_t t_after_hud   = t_after_wait;
        int64_t t_after_blit  = t_after_wait;

        if (display_ready) {
            // Only clear the HUD strip on every frame — the preview area
            // is immediately overwritten by the blit and the HUD strip is
            // the only region that needs a black background beneath the
            // translucent overlays. Cuts ~8 ms off the old full clear.
            fbdraw_fill_rect(&fb, (int)hud_area_x, 0,
                             (int)hud_area_w, (int)logical_h, BLACK);
            t_after_bg = esp_timer_get_time();

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
            } else if (mode == MODE_CONFIG) {
                // Blank the preview area; the menu list is drawn below
                // the camera-preview branch so we skip that branch here.
                fbdraw_fill_rect(&fb, 0, 0,
                                 (int)preview_area_w, (int)preview_area_h, BLACK);
                int mx = 20;
                int my = 30;
                fbdraw_hershey_string(&fb, WHITE, mx, my, "Configuration", 22);
                my += 36;
                fbdraw_hershey_string(&fb, WHITE, mx, my,
                                      "UP/DN select  SPACE toggle/cycle", 14);
                my += 28;
                const uint16_t DIM     = fbdraw_rgb(96, 96, 96);
                const int      row_h   = 26;
                const int      row_pad = 4;
                for (int i = 0; i < CFG_ITEM_COUNT; ++i) {
                    const cfg_item_t *it    = &g_cfg_items[i];
                    bool              avail = cfg_item_enabled(it);
                    bool              sel   = (i == g_cfg_sel);
                    uint16_t          col   = sel  ? (avail ? YELLOW : DIM)
                                                   : (avail ? WHITE  : DIM);
                    if (sel) {
                        fbdraw_fill_rect(&fb, mx - row_pad, my - row_pad,
                                         (int)preview_area_w - 2 * (mx - row_pad),
                                         row_h, MENU_HL_BG);
                    }
                    char row[80];
                    if (it->kind == CFG_KIND_DRIVER) {
                        const focus_driver_t *drv =
                            focus_driver_by_name((const char *)it->target);
                        const char *shown = drv ? drv->display_name : "???";
                        snprintf(row, sizeof(row), "%-9s < %s >",
                                 it->label, shown);
                    } else if (it->kind == CFG_KIND_INT) {
                        int iv = *(int *)it->target;
                        snprintf(row, sizeof(row), "%-9s < %d >",
                                 it->label, iv);
                    } else {
                        bool v = *(bool *)it->target;
                        snprintf(row, sizeof(row), "[%c] %s%s",
                                 v ? 'x' : ' ', it->label,
                                 avail ? "" : " (needs Focus)");
                    }
                    fbdraw_hershey_string(&fb, col, mx, my, row, 18);
                    my += row_h;
                }
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
            t_after_image = esp_timer_get_time();

            // Manual focus stepping. A full sweep across the driver's
            // range takes ~4 s, i.e. 256 steps/s ≈ 3906 µs per step.
            // We scale steps by the elapsed wall time since the last
            // step so a slow frame loop doesn't make the scan feel
            // jittery.
            const focus_driver_t *fd = focus_active();
            if (g_focus_dir != 0 && g_focus_present && fd &&
                mode != MODE_VIEW && mode != MODE_CONFIG) {
                int64_t now  = esp_timer_get_time();
                int64_t dt   = now - g_focus_last_us;
                int     step = (int)((dt * 256) / 1000000);
                if (step > 0) {
                    int32_t np = (int32_t)g_focus_pos + g_focus_dir * step;
                    if (np < (int32_t)fd->pos_min) np = fd->pos_min;
                    if (np > (int32_t)fd->pos_max) np = fd->pos_max;
                    if ((uint16_t)np != g_focus_pos) {
                        g_focus_pos = (uint16_t)np;
                        focus_set_position(g_focus_pos);
                    }
                    g_focus_last_us = now;
                }
            }

            // Hardware autofocus tick. Skipped entirely when AF is
            // disabled (autofocus_tick is a no-op in that case, but
            // the explicit gate keeps the cost obvious here).
            if (g_focus_present && g_cfg.autofocus_enabled &&
                mode != MODE_VIEW && mode != MODE_CONFIG) {
                autofocus_tick(&g_focus_pos);
            }

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

            // Key hint lines: 32x32 colour icon + label, with the
            // text fallback ("F1", etc.) used when the launcher icon
            // PNGs aren't installed yet.
            #define KEY_ROW_H    32
            #define KEY_ICON_GAP 6
            #define KEY_TEXT_DY  ((KEY_ROW_H - hud_font) / 2)
            const struct {
                icon_key_t  icon;
                const char *fallback;
                const char *label;
            } krows[] = {
                { ICON_F1,  "F1",  "view"   },
                { ICON_F2,  "F2",  "photo"  },
                { ICON_F3,  "F3",  "video"  },
                { ICON_F4,  "F4",  "Config" },
                { ICON_ESC, "Esc", "exit"   },
            };
            for (size_t k = 0; k < sizeof(krows) / sizeof(krows[0]); k++) {
                int label_x = hud_pad_x;
                if (icons_get(krows[k].icon)) {
                    icons_blit(&fb, krows[k].icon, hud_pad_x, hud_y);
                    label_x = hud_pad_x + icons_width(krows[k].icon) + KEY_ICON_GAP;
                } else {
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y + KEY_TEXT_DY,
                                          krows[k].fallback, hud_font);
                    label_x = hud_pad_x + 36;
                }
                fbdraw_hershey_string(&fb, WHITE, label_x, hud_y + KEY_TEXT_DY,
                                      krows[k].label, hud_font);
                hud_y += KEY_ROW_H + 2;
            }
            hud_y += 4;

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
                    // Audio level meter. Shown whenever the mic is
                    // actually running so the user can verify at a
                    // glance that sound is reaching the chip — no
                    // need to record, transfer and play back a clip
                    // just to check wiring / enable. Peak value is
                    // [0..32767]; we map it to a bar that fills the
                    // HUD strip width minus the pad on either side.
                    if (microphone_is_running()) {
                        // Digital gain readout. Shown above the peak
                        // bar so the user knows what VOL+/- adjusts
                        // without having to enter the config menu.
                        char gain_line[24];
                        snprintf(gain_line, sizeof(gain_line),
                                 "Gain %dx", g_cfg.mic_gain);
                        fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y,
                                              gain_line, hud_font);
                        hud_y += hud_line;

                        uint16_t peak    = microphone_peak_level();
                        int      bar_x   = hud_pad_x;
                        int      bar_w   = (int)hud_area_w - 20;
                        int      bar_h   = 8;
                        int      fill_w  = (int)((uint32_t)peak * (uint32_t)bar_w / 32768u);
                        if (fill_w > bar_w) fill_w = bar_w;
                        // Colour shifts green → yellow → red as the
                        // signal approaches clipping, matching the
                        // convention every audio tool on the planet
                        // uses so the user reads it instantly.
                        uint16_t fill_col =
                            (peak > 26000) ? RED :
                            (peak > 16000) ? YELLOW :
                                             fbdraw_rgb(0, 200, 0);
                        fbdraw_fill_rect(&fb, bar_x, hud_y, bar_w, bar_h, HUD_BG);
                        if (fill_w > 0) {
                            fbdraw_fill_rect(&fb, bar_x, hud_y, fill_w, bar_h, fill_col);
                        }
                        hud_y += bar_h + 4;
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
                case MODE_CONFIG:
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "UP/DN sel",   hud_font); hud_y += hud_line;
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "SPACE tog",   hud_font); hud_y += hud_line;
                    fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, "F4/Esc back", hud_font); hud_y += hud_line;
                    break;
            }

            // Focus readout — visible whenever a focus driver is
            // active and we're not in the config menu (the menu shows
            // the state already) or the file viewer. Drawn as its own
            // block separated from the interaction hints above by a
            // gap and a thin divider line so it reads as a state
            // panel rather than another keybinding.
            if (g_focus_present &&
                mode != MODE_VIEW && mode != MODE_CONFIG) {
                hud_y += 14;  // gap above the focus block
                fbdraw_fill_rect(&fb, hud_pad_x, hud_y,
                                 (int)hud_area_w - 20, 1, HUD_SEP);
                hud_y += 8;

                char fl[16];
                snprintf(fl, sizeof(fl), "Focus %4u", (unsigned)g_focus_pos);
                fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, fl, hud_font);
                hud_y += hud_line;

                const char *af_msg = "AF: none";
                if (g_cfg.autofocus_enabled) {
                    switch (autofocus_hud_state()) {
                        case AF_HUD_SEARCH:   af_msg = "AF: searching"; break;
                        case AF_HUD_LOCK:     af_msg = "AF: locked";    break;
                        case AF_HUD_OVERRIDE: af_msg = "AF: manual";    break;
                        default:              af_msg = "AF: off";       break;
                    }
                }
                fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, af_msg, hud_font);
                hud_y += hud_line;

                char dl[24];
                snprintf(dl, sizeof(dl), "DRV: %s", focus_active_name());
                fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y, dl, hud_font);
                hud_y += hud_line;

                fbdraw_hershey_string(&fb, WHITE, hud_pad_x, hud_y,
                                      "UP/DN focus", hud_font);
                hud_y += hud_line;
            }

            // Sensor model footer at the bottom of the HUD strip.
            // Pinned to a fixed y rather than appended to hud_y so it
            // never collides with whatever the variable-height blocks
            // above it (audio meter, focus readout) push down to.
            fbdraw_hershey_string(&fb, WHITE, hud_pad_x, 458,
                                  camera_sensor_name(&sensor), hud_font);

            // Transient banner (e.g. "Saved IMG_xxx.jpg"). Draw it
            // left-aligned over the preview area so long filenames
            // aren't clipped by the narrow HUD strip.
            if (banner_text[0] && xTaskGetTickCount() < banner_until) {
                int banner_y = (int)logical_h - 50;
                fbdraw_fill_rect(&fb, 0, banner_y,
                                 (int)preview_area_w, 40, BANNER_BG);
                fbdraw_hershey_string(&fb, WHITE, 10, banner_y + 12,
                                      banner_text, hud_font);
            } else {
                banner_text[0] = 0;
            }
            t_after_hud = esp_timer_get_time();

            // Submit the completed frame to the display and swap to the
            // other buffer so the next iteration draws into a clean
            // buffer while DMA reads the one we just submitted.
            esp_lcd_panel_draw_bitmap(s_panel, 0, 0,
                                      display_h_res, display_v_res, fb.pixels);
            fbdraw_swap(&fb);
            t_after_blit = esp_timer_get_time();
        }

        // Dump per-stage timings every ~1 s so we can see which step
        // dominates. Remove once the main loop is fast enough.
        static int prof_n = 0;
        if (++prof_n >= 15) {
            prof_n = 0;
            /*
            ESP_LOGI(TAG, "loop: wait=%lld bg=%lld img=%lld hud=%lld blit=%lld ready=%d (us)",
                     (long long)(t_after_wait  - t_wait_start),
                     (long long)(t_after_bg    - t_after_wait),
                     (long long)(t_after_image - t_after_bg),
                     (long long)(t_after_hud   - t_after_image),
                     (long long)(t_after_blit  - t_after_hud),
                     display_ready);
             */
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
