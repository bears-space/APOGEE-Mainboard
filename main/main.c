#include <unistd.h>
#include "esp_log.h"
#include "vigilant.h"

static const char *TAG = "app_main";

void app_main(void)
{
    VigilantConfig VgConfig = {
        .unique_component_name = "Vigliant ESP Test",
        .network_mode = NW_MODE_APSTA
    };

    ESP_ERROR_CHECK(vigilant_init(VgConfig));

    while (1) {
        sleep(1);
    }
}
