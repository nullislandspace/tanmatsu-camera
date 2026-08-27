#include "config.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "esp_log.h"
#include "sdcard.h"

static const char *TAG = "config";

static const char CFG_HEADER[] =
    "# Tanmatsu camera configuration\n"
    "# Edit with any text editor. Reboot to apply changes made on a PC.\n"
    "# Lines starting with # are comments. Format: key=value\n"
    "#\n"
    "# focus_driver       which focus chip driver to use (e.g. simulator,\n"
    "#                     dw9714p). Inactive until focus_enabled=1.\n"
    "# focus_enabled      master enable for the focus subsystem.\n"
    "# autofocus_enabled  enable hardware autofocus (needs focus_enabled=1\n"
    "#                     and a working chip).\n"
    "# rotate_180         rotate the camera image 180° (preview + saved\n"
    "#                     files). For sensors mounted upside down.\n"
    "# mic_type           I2S microphone type: none, inmp441, ics43434. When\n"
    "#                     not 'none', video mode captures live audio and\n"
    "#                     shows a level meter. The legacy mic_enabled=0|1\n"
    "#                     key is still parsed (1 = inmp441, 0 = none).\n"
    "# mic_gain           digital gain step for mic samples (1..8). Each\n"
    "#                     step is ~5.7 dB: 1x (unity) up to 100x at step 8.\n"
    "# auto_exposure      choose the exposure automatically (the sensor's own\n"
    "#                     AE loop, or the software one where it has none).\n"
    "# cam_brightness     manual exposure step (1..16) used when auto_exposure=0.\n"
    "#                     One stop per step. Q/A adjust it live.\n"
    "# hdmi_probe         look for a TC358743 HDMI-to-CSI bridge when no\n"
    "#                     ordinary camera answered. Off by default: the\n"
    "#                     probe drives GPIO6, which is the camera LED line\n"
    "#                     and is also expansion pin E2 (see camera.md).\n"
    "# hdmi_color_path    TC358743 only. yuv420 (cheap, full colour) or\n"
    "#                     rgb565 (slower, coarser, but a better-trodden\n"
    "#                     path through the PPA).\n"
    "# hdmi_yuv_order     TC358743 only. Byte order in each 4-byte YUV422\n"
    "#                     group, 0..7. Change only if colours are wrong or\n"
    "#                     vertical edges show a one-pixel comb.\n";

const char *hdmi_color_path_config_name(hdmi_color_path_t p) {
    return (p == HDMI_COLOR_PATH_RGB565) ? "rgb565" : "yuv420";
}

const char *hdmi_color_path_display_name(hdmi_color_path_t p) {
    return (p == HDMI_COLOR_PATH_RGB565) ? "RGB565" : "YUV420";
}

static bool parse_hdmi_color_path(const char *v, hdmi_color_path_t *out) {
    if (strcasecmp(v, "yuv420") == 0 || strcmp(v, "0") == 0) {
        *out = HDMI_COLOR_PATH_YUV420;
        return true;
    }
    if (strcasecmp(v, "rgb565") == 0 || strcmp(v, "1") == 0) {
        *out = HDMI_COLOR_PATH_RGB565;
        return true;
    }
    return false;
}

static int clamp_hdmi_yuv_order(int v) {
    if (v < CONFIG_HDMI_YUV_ORDER_MIN) return CONFIG_HDMI_YUV_ORDER_DEFAULT;
    if (v > CONFIG_HDMI_YUV_ORDER_MAX) return CONFIG_HDMI_YUV_ORDER_DEFAULT;
    return v;
}

const char *mic_type_config_name(mic_type_t t) {
    switch (t) {
        case MIC_TYPE_INMP441:  return "inmp441";
        case MIC_TYPE_ICS43434: return "ics43434";
        case MIC_TYPE_NONE:
        default:                return "none";
    }
}

const char *mic_type_display_name(mic_type_t t) {
    switch (t) {
        case MIC_TYPE_INMP441:  return "INMP441";
        case MIC_TYPE_ICS43434: return "ICS43434";
        case MIC_TYPE_NONE:
        default:                return "None";
    }
}

static bool parse_mic_type(const char *v, mic_type_t *out) {
    if (strcasecmp(v, "none") == 0 || strcmp(v, "0") == 0 ||
        strcasecmp(v, "off") == 0  || strcasecmp(v, "false") == 0) {
        *out = MIC_TYPE_NONE;
        return true;
    }
    if (strcasecmp(v, "inmp441") == 0) {
        *out = MIC_TYPE_INMP441;
        return true;
    }
    if (strcasecmp(v, "ics43434") == 0) {
        *out = MIC_TYPE_ICS43434;
        return true;
    }
    return false;
}

static void defaults(camera_config_t *out) {
    strncpy(out->focus_driver, "simulator", CONFIG_FOCUS_DRIVER_MAXLEN - 1);
    out->focus_driver[CONFIG_FOCUS_DRIVER_MAXLEN - 1] = '\0';
    out->focus_enabled     = false;
    out->autofocus_enabled = false;
    out->rotate_180        = false;
    out->mic_type          = MIC_TYPE_NONE;
    out->mic_gain          = CONFIG_MIC_GAIN_DEFAULT;
    out->auto_exposure     = CONFIG_AUTO_EXPOSURE_DEFAULT;
    out->cam_brightness    = CONFIG_CAM_BRIGHTNESS_DEFAULT;
    out->hdmi_probe        = CONFIG_HDMI_PROBE_DEFAULT;
    out->hdmi_color_path   = HDMI_COLOR_PATH_YUV420;
    out->hdmi_yuv_order    = CONFIG_HDMI_YUV_ORDER_DEFAULT;
}

static int clamp_mic_gain(int v) {
    if (v < CONFIG_MIC_GAIN_MIN) return CONFIG_MIC_GAIN_MIN;
    if (v > CONFIG_MIC_GAIN_MAX) return CONFIG_MIC_GAIN_MAX;
    return v;
}

static int clamp_cam_brightness(int v) {
    if (v < CONFIG_CAM_BRIGHTNESS_MIN) return CONFIG_CAM_BRIGHTNESS_MIN;
    if (v > CONFIG_CAM_BRIGHTNESS_MAX) return CONFIG_CAM_BRIGHTNESS_MAX;
    return v;
}

// Trim leading/trailing whitespace in-place, return the (possibly
// shifted) start of the trimmed string.
static char *trim(char *s) {
    while (*s && isspace((unsigned char)*s)) s++;
    char *end = s + strlen(s);
    while (end > s && isspace((unsigned char)end[-1])) *--end = '\0';
    return s;
}

static bool parse_bool(const char *v, bool *out) {
    if (strcmp(v, "1") == 0 || strcasecmp(v, "true") == 0 ||
        strcasecmp(v, "yes") == 0 || strcasecmp(v, "on") == 0) {
        *out = true;
        return true;
    }
    if (strcmp(v, "0") == 0 || strcasecmp(v, "false") == 0 ||
        strcasecmp(v, "no") == 0 || strcasecmp(v, "off") == 0) {
        *out = false;
        return true;
    }
    return false;
}

esp_err_t config_save(const camera_config_t *cfg) {
    if (!sdcard_is_mounted()) return ESP_ERR_INVALID_STATE;
    FILE *f = fopen(CONFIG_PATH, "w");
    if (!f) {
        ESP_LOGE(TAG, "open %s for write: errno=%d", CONFIG_PATH, errno);
        return ESP_FAIL;
    }
    fputs(CFG_HEADER, f);
    fprintf(f, "focus_driver=%s\n",      cfg->focus_driver);
    fprintf(f, "focus_enabled=%d\n",     cfg->focus_enabled ? 1 : 0);
    fprintf(f, "autofocus_enabled=%d\n", cfg->autofocus_enabled ? 1 : 0);
    fprintf(f, "rotate_180=%d\n",        cfg->rotate_180 ? 1 : 0);
    fprintf(f, "mic_type=%s\n",          mic_type_config_name(cfg->mic_type));
    fprintf(f, "mic_gain=%d\n",          clamp_mic_gain(cfg->mic_gain));
    fprintf(f, "auto_exposure=%d\n",    cfg->auto_exposure ? 1 : 0);
    fprintf(f, "cam_brightness=%d\n",   clamp_cam_brightness(cfg->cam_brightness));
    fprintf(f, "hdmi_probe=%d\n",       cfg->hdmi_probe ? 1 : 0);
    fprintf(f, "hdmi_color_path=%s\n",  hdmi_color_path_config_name(cfg->hdmi_color_path));
    fprintf(f, "hdmi_yuv_order=%d\n",   clamp_hdmi_yuv_order(cfg->hdmi_yuv_order));
    fclose(f);
    ESP_LOGI(TAG, "saved %s (driver=%s focus=%d af=%d rot180=%d mic=%s gain=%d ae=%d bright=%d)",
             CONFIG_PATH, cfg->focus_driver,
             cfg->focus_enabled, cfg->autofocus_enabled,
             cfg->rotate_180, mic_type_config_name(cfg->mic_type), cfg->mic_gain,
             cfg->auto_exposure, cfg->cam_brightness);
    return ESP_OK;
}

esp_err_t config_load(camera_config_t *out) {
    defaults(out);
    if (!sdcard_is_mounted()) {
        ESP_LOGW(TAG, "SD not mounted, using defaults");
        return ESP_OK;
    }

    FILE *f = fopen(CONFIG_PATH, "r");
    if (!f) {
        ESP_LOGI(TAG, "%s not found, seeding with defaults", CONFIG_PATH);
        config_save(out);  // best-effort
        return ESP_OK;
    }

    char line[128];
    while (fgets(line, sizeof(line), f)) {
        char *s = trim(line);
        if (*s == '\0' || *s == '#') continue;
        char *eq = strchr(s, '=');
        if (!eq) continue;
        *eq = '\0';
        char *key = trim(s);
        char *val = trim(eq + 1);

        if (strcmp(key, "focus_driver") == 0) {
            if (strlen(val) >= CONFIG_FOCUS_DRIVER_MAXLEN) {
                ESP_LOGW(TAG, "focus_driver value too long, keeping default '%s'",
                         out->focus_driver);
            } else {
                strncpy(out->focus_driver, val, CONFIG_FOCUS_DRIVER_MAXLEN - 1);
                out->focus_driver[CONFIG_FOCUS_DRIVER_MAXLEN - 1] = '\0';
            }
        } else if (strcmp(key, "focus_enabled") == 0) {
            if (!parse_bool(val, &out->focus_enabled)) {
                ESP_LOGW(TAG, "bad value for focus_enabled: '%s'", val);
            }
        } else if (strcmp(key, "autofocus_enabled") == 0) {
            if (!parse_bool(val, &out->autofocus_enabled)) {
                ESP_LOGW(TAG, "bad value for autofocus_enabled: '%s'", val);
            }
        } else if (strcmp(key, "rotate_180") == 0) {
            if (!parse_bool(val, &out->rotate_180)) {
                ESP_LOGW(TAG, "bad value for rotate_180: '%s'", val);
            }
        } else if (strcmp(key, "mic_type") == 0) {
            if (!parse_mic_type(val, &out->mic_type)) {
                ESP_LOGW(TAG, "bad value for mic_type: '%s'", val);
            }
        } else if (strcmp(key, "mic_enabled") == 0) {
            // Legacy key. Map 1 → INMP441 (the only mic the old
            // schema knew about), 0 → none. Newer mic_type lines
            // win if both appear in the file (parsed in source
            // order).
            bool legacy = false;
            if (parse_bool(val, &legacy)) {
                out->mic_type = legacy ? MIC_TYPE_INMP441 : MIC_TYPE_NONE;
            } else {
                ESP_LOGW(TAG, "bad value for mic_enabled: '%s'", val);
            }
        } else if (strcmp(key, "mic_gain") == 0) {
            char *endp = NULL;
            long  n    = strtol(val, &endp, 10);
            if (endp && endp != val && *endp == '\0') {
                out->mic_gain = clamp_mic_gain((int)n);
            } else {
                ESP_LOGW(TAG, "bad value for mic_gain: '%s'", val);
            }
        } else if (strcmp(key, "auto_exposure") == 0) {
            if (!parse_bool(val, &out->auto_exposure)) {
                ESP_LOGW(TAG, "bad value for auto_exposure: '%s'", val);
            }
        } else if (strcmp(key, "cam_brightness") == 0) {
            // 0 used to mean "auto" before auto_exposure existed; accept
            // it (and the word) so an older config file still lands in
            // automatic rather than at the darkest step.
            if (strcasecmp(val, "auto") == 0 || strcmp(val, "0") == 0) {
                out->auto_exposure  = true;
                out->cam_brightness = CONFIG_CAM_BRIGHTNESS_DEFAULT;
            } else {
                char *endp = NULL;
                long  n    = strtol(val, &endp, 10);
                if (endp && endp != val && *endp == '\0') {
                    out->cam_brightness = clamp_cam_brightness((int)n);
                } else {
                    ESP_LOGW(TAG, "bad value for cam_brightness: '%s'", val);
                }
            }
        } else if (strcmp(key, "hdmi_probe") == 0) {
            if (!parse_bool(val, &out->hdmi_probe)) {
                ESP_LOGW(TAG, "bad value for hdmi_probe: '%s'", val);
            }
        } else if (strcmp(key, "hdmi_color_path") == 0) {
            if (!parse_hdmi_color_path(val, &out->hdmi_color_path)) {
                ESP_LOGW(TAG, "bad value for hdmi_color_path: '%s'", val);
            }
        } else if (strcmp(key, "hdmi_yuv_order") == 0) {
            char *endp = NULL;
            long  n    = strtol(val, &endp, 10);
            if (endp && endp != val && *endp == '\0') {
                out->hdmi_yuv_order = clamp_hdmi_yuv_order((int)n);
            } else {
                ESP_LOGW(TAG, "bad value for hdmi_yuv_order: '%s'", val);
            }
        } else {
            ESP_LOGW(TAG, "unknown key '%s'", key);
        }
    }
    fclose(f);
    ESP_LOGI(TAG, "loaded %s (driver=%s focus=%d af=%d rot180=%d mic=%s gain=%d ae=%d bright=%d)",
             CONFIG_PATH, out->focus_driver,
             out->focus_enabled, out->autofocus_enabled,
             out->rotate_180, mic_type_config_name(out->mic_type), out->mic_gain,
             out->auto_exposure, out->cam_brightness);
    return ESP_OK;
}
