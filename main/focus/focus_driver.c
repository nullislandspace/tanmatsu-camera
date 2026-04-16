#include "focus_driver.h"

#include <string.h>
#include "esp_log.h"

#include "dw9714p.h"

static const char *TAG = "focus";

// ---- DW9714P driver ---------------------------------------------------

static const focus_driver_t drv_dw9714p = {
    .name         = "dw9714p",
    .display_name = "DW9714P",
    .probe        = dw9714p_probe,
    .init         = dw9714p_init,
    .set_position = dw9714p_set_position,
    .pos_min      = DW9714P_POS_MIN,
    .pos_max      = DW9714P_POS_MAX,
    .pos_default  = DW9714P_POS_MID,
};

// Defined in focus_simulator.c.
extern const focus_driver_t focus_drv_simulator;

// Order matters — first entry is the menu/cycler default.
const focus_driver_t *const focus_driver_registry[] = {
    &focus_drv_simulator,
    &drv_dw9714p,
    NULL,
};

// ---- Active-driver state ---------------------------------------------

static const focus_driver_t *s_active = NULL;

// ---- Lookup helpers --------------------------------------------------

int focus_driver_count(void) {
    int n = 0;
    while (focus_driver_registry[n]) n++;
    return n;
}

const focus_driver_t *focus_driver_by_name(const char *name) {
    if (!name || !*name) return NULL;
    for (int i = 0; focus_driver_registry[i]; i++) {
        if (strcmp(focus_driver_registry[i]->name, name) == 0) {
            return focus_driver_registry[i];
        }
    }
    return NULL;
}

const focus_driver_t *focus_driver_by_index(int idx) {
    if (idx < 0) return NULL;
    int n = focus_driver_count();
    if (idx >= n) return NULL;
    return focus_driver_registry[idx];
}

// ---- Activation ------------------------------------------------------

esp_err_t focus_select(const char *name) {
    // Empty/NULL name means deactivate. VCMs have no shutdown opcode
    // and freewheel safely; we just drop the active pointer.
    if (!name || !*name) {
        if (s_active) ESP_LOGI(TAG, "deactivating '%s'", s_active->name);
        s_active = NULL;
        return ESP_OK;
    }

    const focus_driver_t *drv = focus_driver_by_name(name);
    if (!drv) {
        ESP_LOGW(TAG, "unknown driver '%s'", name);
        return ESP_ERR_NOT_FOUND;
    }

    // Atomic switch: probe + init the new driver BEFORE dropping the
    // old one. This way a failed switch leaves the old driver active
    // so the user-visible state stays consistent.
    esp_err_t err = drv->probe();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "driver '%s' probe failed: %d", name, err);
        return err;
    }
    if (drv->init) {
        err = drv->init();
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "driver '%s' init failed: %d", name, err);
            return err;
        }
    }
    err = drv->set_position(drv->pos_default);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "driver '%s' set_position(default) failed: %d", name, err);
        return err;
    }

    if (s_active && s_active != drv) {
        ESP_LOGI(TAG, "switching '%s' -> '%s'", s_active->name, drv->name);
    } else {
        ESP_LOGI(TAG, "activated '%s'", drv->name);
    }
    s_active = drv;
    return ESP_OK;
}

const focus_driver_t *focus_active(void) {
    return s_active;
}

const char *focus_active_name(void) {
    return s_active ? s_active->name : "off";
}

esp_err_t focus_set_position(uint16_t pos) {
    if (!s_active) return ESP_ERR_INVALID_STATE;
    return s_active->set_position(pos);
}
