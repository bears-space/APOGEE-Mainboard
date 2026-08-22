#include "fan.h"

#include <stdio.h>

#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char* TAG = "fan";

#define PWM_GPIO CONFIG_FAN_PIN
#define PWM_FREQUENCY 30000
#define PWM_RESOLUTION LEDC_TIMER_8_BIT
#define PWM_TIMER LEDC_TIMER_0
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_MODE LEDC_LOW_SPEED_MODE
#define TACH_PULSES_PER_REVOLUTION 2
#define MICROSECONDS_PER_MINUTE 60000000LL

static pcnt_unit_handle_t pcnt_unit;
static pcnt_channel_handle_t pcnt_channel;
static int64_t pcnt_sample_start_us;

void pwm_init(void) {
    ledc_timer_config_t timer_config = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num = PWM_GPIO,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
}

void pulse_counter_init(void) {
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000,
        .low_limit = -1,
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_chan_config_t channel_config = {
        .edge_gpio_num = CONFIG_PCNT_PIN,
        .level_gpio_num = -1,
    };

    ESP_ERROR_CHECK(
        pcnt_new_channel(pcnt_unit, &channel_config, &pcnt_channel));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_channel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,  // rising edge
        PCNT_CHANNEL_EDGE_ACTION_HOLD       // falling edge
        ));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    pcnt_sample_start_us = esp_timer_get_time();
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void fan_init(void) {
    pulse_counter_init();
    pwm_init();
    set_fan_speed(0);
    start_fan();
}

void set_fan_speed(int speed) {
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

void stop_fan(void) {
    set_fan_speed(0);
    ESP_LOGI(TAG, "Stopping fan");
}

void start_fan(void) {
    set_fan_speed(100);
    ESP_LOGI(TAG, "Starting fan");
}

int fan_get_rpm(void) {
    int pulse_count = 0;

    ESP_ERROR_CHECK(pcnt_unit_stop(pcnt_unit));
    const int64_t sample_end_us = esp_timer_get_time();
    ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

    const int64_t elapsed_us = sample_end_us - pcnt_sample_start_us;
    pcnt_sample_start_us = esp_timer_get_time();
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    if (elapsed_us <= 0) {
        return 0;
    }

    return (int)(((int64_t)pulse_count * MICROSECONDS_PER_MINUTE) /
                 (elapsed_us * TACH_PULSES_PER_REVOLUTION));
}
