#include <unistd.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_led.h"
#include "vigilant.h"
#include "BQ769x2.h"

// static const char *TAG = "app_main";

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Mainboard",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));
}
