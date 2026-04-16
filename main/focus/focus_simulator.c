#include "focus_driver.h"

#include "esp_log.h"

// Pure software focus driver — pretends a VCM is always present so the
// rest of the focus + autofocus pipeline (menu, manual scan, AF state
// machine, HUD) can be exercised on stock hardware with no chip
// attached. Commanded positions are stored but have no physical effect;
// the AF hill-climb on real ISP sharpness will therefore not converge
// in any meaningful way (sharpness stays roughly flat with position).
// That is expected — the simulator is a code-path test, not an AF
// quality test.

static const char *TAG = "focus_sim";

static uint16_t s_pos = 512;

static esp_err_t sim_probe(void) { return ESP_OK; }
static esp_err_t sim_init (void) { return ESP_OK; }

static esp_err_t sim_set_position(uint16_t pos) {
    if (pos > 1023) pos = 1023;
    s_pos = pos;
    ESP_LOGD(TAG, "pos=%u", (unsigned)pos);
    return ESP_OK;
}

const focus_driver_t focus_drv_simulator = {
    .name         = "simulator",
    .display_name = "Simulator",
    .probe        = sim_probe,
    .init         = sim_init,
    .set_position = sim_set_position,
    .pos_min      = 0,
    .pos_max      = 1023,
    .pos_default  = 512,
};
