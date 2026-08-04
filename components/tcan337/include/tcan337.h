#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** TCAN337 transceiver and ESP32-S3 TWAI controller configuration. */
typedef struct {
    gpio_num_t tx_io;
    gpio_num_t rx_io;
    /** Optional TCAN337 pin 5 input. Use GPIO_NUM_NC when not connected. */
    gpio_num_t fault_io;
    /** Optional TCAN337 pin 8 output. Use GPIO_NUM_NC when strapped low. */
    gpio_num_t silent_io;
    /** Classic CAN arbitration bitrate, up to 1 Mbit/s for TCAN337. */
    uint32_t bitrate;
} tcan337_config_t;

/** A Classic CAN frame received or transmitted by the ESP32 TWAI controller. */
typedef struct {
    uint32_t id;
    bool extended;
    bool remote;
    uint8_t data_length;
    uint64_t timestamp_us;
    uint8_t data[8];
} tcan337_frame_t;

typedef enum {
    TCAN337_ERROR_ACTIVE,
    TCAN337_ERROR_WARNING,
    TCAN337_ERROR_PASSIVE,
    TCAN337_BUS_OFF,
} tcan337_bus_state_t;

enum {
    TCAN337_ERROR_ARBITRATION_LOST = 1UL << 0,
    TCAN337_ERROR_BIT = 1UL << 1,
    TCAN337_ERROR_FORM = 1UL << 2,
    TCAN337_ERROR_STUFF = 1UL << 3,
    TCAN337_ERROR_ACK = 1UL << 4,
};

typedef struct {
    tcan337_bus_state_t state;
    uint16_t tx_error_count;
    uint16_t rx_error_count;
    uint32_t bus_error_count;
    uint32_t last_error_flags;
    uint32_t received_frames;
    uint32_t dropped_frames;
    uint32_t transmitted_frames;
    uint32_t failed_transmissions;
    bool fault_monitoring_available;
    /** True when the optional FAULT input reports a transceiver fault. */
    bool transceiver_fault;
} tcan337_diagnostics_t;

/** Initialize the ESP32-S3 TWAI controller for a boot-time self-test. */
esp_err_t tcan337_init(const tcan337_config_t* config);

/**
 * Send and receive a known eight-byte Classic CAN beacon in self-test mode.
 * The beacon ID may be an 11-bit standard ID or a 29-bit extended ID.
 */
esp_err_t tcan337_run_loopback_test(uint32_t beacon_id);

/**
 * Leave self-test mode and start normal CAN reception.
 *
 * listen_only=false participates normally and acknowledges valid frames.
 * listen_only=true never transmits dominant bits, including acknowledgements.
 */
esp_err_t tcan337_start(bool listen_only);

/** Transmit one Classic CAN frame and wait for the attempt to finish. */
esp_err_t tcan337_transmit(const tcan337_frame_t* frame, uint32_t timeout_ms);

/** Receive the oldest frame, waiting for up to timeout_ms. */
esp_err_t tcan337_receive(tcan337_frame_t* frame, uint32_t timeout_ms);

/** Read controller counters and the optional TCAN337 FAULT input. */
esp_err_t tcan337_get_diagnostics(tcan337_diagnostics_t* diagnostics);

/** Begin recovery when the controller is bus-off. */
esp_err_t tcan337_recover(void);

#ifdef __cplusplus
}
#endif
