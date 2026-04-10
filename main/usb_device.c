#include "usb_device.h"
#include "sdkconfig.h"

#ifdef CONFIG_IDF_TARGET_ESP32P4

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"

static const char* TAG = "usb";

void usb_initialize(void) {
    ESP_LOGI(TAG, "USB debug initialization");

    // Pull override values to temporarily disconnect from USB bus
    const usb_serial_jtag_pull_override_vals_t override_disable_usb = {
        .dm_pd = true, .dm_pu = false, .dp_pd = true, .dp_pu = false};

    // Pull override values to re-enable USB (D+ pull-up for device detection)
    const usb_serial_jtag_pull_override_vals_t override_enable_usb = {
        .dm_pd = false, .dm_pu = false, .dp_pd = false, .dp_pu = true};

    // Drop off the bus by removing the pull-up on USB DP
    usb_serial_jtag_ll_phy_enable_pull_override(&override_disable_usb);

    // Select USB Serial JTAG PHY (debug mode = PHY 0)
    usb_serial_jtag_ll_phy_select(0);

    // Wait for disconnect to settle (same as launcher)
    vTaskDelay(pdMS_TO_TICKS(500));

    // Put the device back onto the bus by re-enabling the pull-up on USB DP
    usb_serial_jtag_ll_phy_enable_pull_override(&override_enable_usb);
    usb_serial_jtag_ll_phy_disable_pull_override();

    ESP_LOGI(TAG, "USB debug mode enabled");
}

#else

// Stub for non-ESP32P4 targets
void usb_initialize(void) {
}

#endif
