#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

esp_err_t bq76942_read_cell_voltage(i2c_master_dev_handle_t port, uint8_t addr, float *voltage);
esp_err_t bq76942_read_pack_voltage(i2c_master_dev_handle_t port, uint8_t addr, float *voltage);
esp_err_t bq76942_read_current(i2c_master_dev_handle_t port, uint8_t addr, float *current);
esp_err_t bq76942_read_temperatures(i2c_master_dev_handle_t port, uint8_t addr, float *temperatures);
esp_err_t bq76942_read_safety_status(i2c_master_dev_handle_t port, uint8_t addr, uint8_t *status);
esp_err_t bq76942_data_memory_read(i2c_master_dev_handle_t port, uint8_t addr, uint16_t *data);
esp_err_t bq76942_data_memory_write(i2c_master_dev_handle_t port, uint8_t addr, uint16_t *data);
esp_err_t bq76942_enter_config_update(i2c_master_dev_handle_t port, uint8_t addr);
esp_err_t bq76942_exit_config_update(i2c_master_dev_handle_t port, uint8_t addr);