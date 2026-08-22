#include <stdio.h>
#include "fan.h"
#include "esp_log.h"

static const char *TAG = "fan";

void set_fan_speed(int speed)
{
    ESP_LOGI(TAG, "Setting fan speed to %d", speed);
}

void stop_fan(void)
{
    ESP_LOGI(TAG, "Stopping fan");
}

void start_fan(void)
{
    ESP_LOGI(TAG, "Starting fan");
}