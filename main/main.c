#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_led.h"
#include "vigilant.h"
#include "fan.h"

// static const char *TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    fan_init();

    while (1) {
        set_fan_speed(100);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        set_fan_speed(20);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        set_fan_speed(5);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        set_fan_speed(1);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}
