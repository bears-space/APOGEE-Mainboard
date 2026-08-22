#include <stdio.h>
#include "fan.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

static const char *TAG = "fan";


#define PWM_GPIO       CONFIG_FAN_PIN
#define PWM_FREQUENCY  30000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_TIMER      LEDC_TIMER_0
#define PWM_CHANNEL    LEDC_CHANNEL_0
#define PWM_MODE       LEDC_LOW_SPEED_MODE

void pwm_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode       = PWM_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num       = PWM_GPIO,
        .speed_mode     = PWM_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .flags.output_invert = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

void fan_init(void)
{
    pwm_init();
    set_fan_speed(0);
    start_fan();
}

void set_fan_speed(int speed)
{
    // int speed: 0-100% speed

    if (speed < 0) {
        speed = 0;
    } else if (speed > 100) {
        speed = 100;
    }

    ESP_LOGI(TAG, "Setting fan speed to %d", speed);

    // 8-bit resolution: values 0–255
    const uint32_t max_duty = (1U << PWM_RESOLUTION) - 1;
    const uint32_t duty = (max_duty * speed) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, PWM_CHANNEL));
}

void stop_fan(void)
{
    set_fan_speed(0);
    ESP_LOGI(TAG, "Stopping fan");
}

void start_fan(void)
{
    set_fan_speed(100);
    ESP_LOGI(TAG, "Starting fan");
}