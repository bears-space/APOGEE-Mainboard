#include <unistd.h>
#include "esp_log.h"
#include "vigilant.h"

static const char *TAG = "app_main";

void app_main(void)
{
    vigilant_init();

    while (1) {
        sleep(5);
    }
}
