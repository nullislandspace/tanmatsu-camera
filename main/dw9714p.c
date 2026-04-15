#include "dw9714p.h"

#include "bsp/i2c.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Bus runs at 100 kHz to match the OV5647 SCCB speed — they share
// wires, so the slower of the two devices sets the bus speed.
#define DW9714P_SCL_HZ       100000u
#define DW9714P_XFER_TIMEOUT 50  // ms

// Unlock / DLC / T_SRC / re-lock words used by Linux DW9714 BSP drivers.
// Values verified against:
//   - ASUS Zenfone5 kernel imx219/dw9714.{c,h}
//   - Intel AtomISP ov5693/ov5693.h (VCM_PROTECTION_OFF/ON macros)
// The mainline kernel driver (drivers/media/i2c/dw9714.c) relies on
// power-on defaults and does NOT send any of these.
#define DW9714P_CMD_PROTECT_OFF 0xECA3u
#define DW9714P_CMD_DLC_MCLK    0xA10Cu  // vcm_dlc_mclk(DLC=1, mclk=0)
#define DW9714P_CMD_TSRC        0xF288u  // vcm_tsrc(0x11)
#define DW9714P_CMD_PROTECT_ON  0xDC51u

static const char *TAG = "dw9714p";

static i2c_master_dev_handle_t s_dev = NULL;

// Attach the chip to the primary I2C bus if we don't have a handle
// yet. Caller must hold the bus mutex. Returns ESP_OK if s_dev is
// ready after the call.
static esp_err_t ensure_dev_locked(void) {
    if (s_dev != NULL) return ESP_OK;

    i2c_master_bus_handle_t bus = NULL;
    esp_err_t               err = bsp_i2c_primary_bus_get_handle(&bus);
    if (err != ESP_OK) return err;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DW9714P_I2C_ADDR,
        .scl_speed_hz    = DW9714P_SCL_HZ,
    };
    return i2c_master_bus_add_device(bus, &cfg, &s_dev);
}

// Send a single 16-bit big-endian word. Caller must hold the bus mutex.
static esp_err_t write_word_locked(uint16_t word) {
    uint8_t buf[2] = { (uint8_t)(word >> 8), (uint8_t)(word & 0xFF) };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), DW9714P_XFER_TIMEOUT);
}

esp_err_t dw9714p_probe(void) {
    bsp_i2c_primary_bus_claim();
    esp_err_t err = ensure_dev_locked();
    if (err == ESP_OK) {
        // A 16-bit write of 0x0000 is a legal direct-mode position=0
        // command. If the chip is present it ACKs; if not, we get
        // ESP_ERR_TIMEOUT from the I2C master (NACK on address phase).
        err = write_word_locked(0x0000);
    }
    bsp_i2c_primary_bus_release();

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "DW9714P detected at 0x%02X", DW9714P_I2C_ADDR);
    } else {
        ESP_LOGW(TAG, "DW9714P probe failed: %d", err);
        if (err == ESP_ERR_TIMEOUT) err = ESP_ERR_NOT_FOUND;
    }
    return err;
}

esp_err_t dw9714p_init(void) {
    // Datasheet / recent kernel fix: wait ≥12 ms after power-on for
    // the internal amplifier offset calibration to settle before any
    // I2C transaction. We pad to 15 ms. Safe to run repeatedly — if
    // the chip was already powered, this is just a harmless delay.
    vTaskDelay(pdMS_TO_TICKS(15));

    bsp_i2c_primary_bus_claim();
    esp_err_t err = ensure_dev_locked();
    if (err == ESP_OK) err = write_word_locked(DW9714P_CMD_PROTECT_OFF);
    if (err == ESP_OK) err = write_word_locked(DW9714P_CMD_DLC_MCLK);
    if (err == ESP_OK) err = write_word_locked(DW9714P_CMD_TSRC);
    if (err == ESP_OK) err = write_word_locked(DW9714P_CMD_PROTECT_ON);
    bsp_i2c_primary_bus_release();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init sequence failed: %d", err);
    }
    return err;
}

esp_err_t dw9714p_set_position(uint16_t pos) {
    if (pos > DW9714P_POS_MAX) pos = DW9714P_POS_MAX;

    // Direct mode (s = 0): 16-bit word = (pos << 4). Top 6 bits of
    // pos land in the high byte, low 4 bits in the upper nibble of
    // the low byte.
    uint16_t word = (uint16_t)(pos << 4);

    bsp_i2c_primary_bus_claim();
    esp_err_t err = ensure_dev_locked();
    if (err == ESP_OK) err = write_word_locked(word);
    bsp_i2c_primary_bus_release();
    return err;
}
