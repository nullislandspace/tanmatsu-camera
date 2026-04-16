#include "config.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
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
    "#                     and a working chip).\n";

static void defaults(camera_config_t *out) {
    strncpy(out->focus_driver, "simulator", CONFIG_FOCUS_DRIVER_MAXLEN - 1);
    out->focus_driver[CONFIG_FOCUS_DRIVER_MAXLEN - 1] = '\0';
    out->focus_enabled     = false;
    out->autofocus_enabled = false;
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
    fclose(f);
    ESP_LOGI(TAG, "saved %s (driver=%s focus=%d af=%d)",
             CONFIG_PATH, cfg->focus_driver,
             cfg->focus_enabled, cfg->autofocus_enabled);
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
        } else {
            ESP_LOGW(TAG, "unknown key '%s'", key);
        }
    }
    fclose(f);
    ESP_LOGI(TAG, "loaded %s (driver=%s focus=%d af=%d)",
             CONFIG_PATH, out->focus_driver,
             out->focus_enabled, out->autofocus_enabled);
    return ESP_OK;
}
