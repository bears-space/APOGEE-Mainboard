# I2C Interface

The Fan component uses a PWM signal to generate the speed for a fan. Furthermore, it uses PCNT (Pulse Counter peripheral) as a tachometer.

## Setting speed
In percent 0-100%

```c
set_fan_speed(100); // 100%
set_fan_speed(0);   // 0%
```

## Starting and stopping the fan

```c
start_fan() // Sets the speed to 100%
stop_fan()  // Sets the speed to 0% 
```

## Reading RPM

```c
set_fan_speed(100);
while (1) {
    int rpm = fan_get_rpm();
    ESP_LOGI(TAG, "Fan RPM: %d", rpm);
    vTaskDelay(pdMS_TO_TICKS(500));
}
```