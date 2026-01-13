#include <unistd.h>
#include "esp_log.h"
#include "vigilant.h"
#include "status_led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app_main";

void app_main(void)
{
    VigilantConfig VgConfig = {
        .unique_component_name = "Vigliant ESP Test",
        .network_mode = NW_MODE_APSTA
    };
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    status_led_config_t led_cfg = {
        .gpio = CONFIG_VE_STATUS_LED_GPIO,
        .resolution_hz = 10 * 1000 * 1000,
        .max_leds = 1,
        .invert_out = false,
        .with_dma = false,
        .mem_block_symbols = 64,
    };
    ESP_ERROR_CHECK(status_led_init(&led_cfg));


    while (1) {
        ESP_ERROR_CHECK(status_led_set_rgb(255, 0, 0));
        vTaskDelay(pdMS_TO_TICKS(300));

        ESP_ERROR_CHECK(status_led_set_rgb(0, 255, 0));
        vTaskDelay(pdMS_TO_TICKS(300));

        ESP_ERROR_CHECK(status_led_set_rgb(0, 0, 255));
        vTaskDelay(pdMS_TO_TICKS(300));

        ESP_ERROR_CHECK(status_led_set_rgb(255, 255, 255));
        vTaskDelay(pdMS_TO_TICKS(300));

        ESP_ERROR_CHECK(status_led_off());
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
