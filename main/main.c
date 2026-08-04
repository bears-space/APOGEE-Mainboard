#include <inttypes.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_led.h"
#include "tcan337.h"
#include "tcan4550.h"
#include "vigilant.h"

/* Hardware connections supplied for the ESP32-S3. */
#define TCAN4550_CS_IO GPIO_NUM_10
#define TCAN4550_MOSI_IO GPIO_NUM_11
#define TCAN4550_SCLK_IO GPIO_NUM_12
#define TCAN4550_MISO_IO GPIO_NUM_13

/* TCAN337 uses the ESP32-S3's integrated Classic CAN/TWAI controller. */
#define TCAN337_RX_IO GPIO_NUM_15
#define TCAN337_TX_IO GPIO_NUM_16
#define TCAN337_FAULT_IO GPIO_NUM_NC
#define TCAN337_SILENT_IO GPIO_NUM_NC

/*
 * This is the CAN identifier used by both loopback verification frames.
 * 0x000..0x7FF selects an 11-bit ID; a larger value selects a 29-bit ID.
 */
#define TCAN4550_BEACON_ADDRESS 0x123U

/* Independent self-test identifier for the TCAN337/TWAI interface. */
#define TCAN337_BEACON_ADDRESS 0x321U

/* Change this to 20000000 if the board uses a 20 MHz crystal/clock. */
#define TCAN4550_OSCILLATOR_HZ 40000000U
#define TCAN4550_NOMINAL_BITRATE 500000U
#define TCAN4550_DATA_BITRATE 2000000U
#define TCAN337_BITRATE 500000U

/* External loopback exercises the transceiver and CANH/CANL path. */
#define TCAN4550_RUN_EXTERNAL_LOOPBACK_TEST 1

#define TCAN_MCAN_IR_EXPECTED \
    ((1UL << 16) | (1UL << 9) | (1UL << 2) | (1UL << 0))
#define TCAN_MCAN_IR_ERRORS 0x3FEF00F8UL
#define TCAN_SPI_STATUS_ERRORS 0x3F3F0000UL
#define TCAN_DEVICE_IR_FAULTS 0x00ED2109UL
#define TCAN_DEVICE_IR_CAN_SILENT (1UL << 10)

static const char* TAG = "app_main";

static const char* tcan337_bus_state_name(tcan337_bus_state_t state) {
    switch (state) {
        case TCAN337_ERROR_WARNING:
            return "warning";
        case TCAN337_ERROR_PASSIVE:
            return "passive";
        case TCAN337_BUS_OFF:
            return "bus-off";
        case TCAN337_ERROR_ACTIVE:
        default:
            return "active";
    }
}

static void log_tcan337_frame(const tcan337_frame_t* frame, uint32_t sequence) {
    char payload[8U * 3U + 1U] = {0};
    size_t offset = 0;
    for (uint8_t index = 0; index < frame->data_length; ++index) {
        const int written =
            snprintf(&payload[offset], sizeof(payload) - offset, "%s%02X",
                     index == 0 ? "" : " ", frame->data[index]);
        if (written < 0 || (size_t)written >= sizeof(payload) - offset) {
            break;
        }
        offset += (size_t)written;
    }

    ESP_LOGI("can_rx",
             "TCAN337 #%06" PRIu32 " ID=0x%08" PRIX32
             " %s CAN%s len=%u timestamp=%" PRIu64 " us data=[%s]",
             sequence, frame->id, frame->extended ? "EXT" : "STD",
             frame->remote ? " RTR" : "", frame->data_length,
             frame->timestamp_us, payload);
}

static void log_tcan337_health(void) {
    static uint32_t previous_bus_errors = 0;
    static uint32_t previous_dropped_frames = 0;
    static uint32_t previous_failed_transmissions = 0;
    static bool recovery_requested = false;

    tcan337_diagnostics_t diagnostics = {0};
    esp_err_t err = tcan337_get_diagnostics(&diagnostics);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read TCAN337/TWAI diagnostics: %s",
                 esp_err_to_name(err));
        return;
    }

    const bool new_fault =
        diagnostics.last_error_flags != 0 ||
        diagnostics.bus_error_count != previous_bus_errors ||
        diagnostics.dropped_frames != previous_dropped_frames ||
        diagnostics.failed_transmissions != previous_failed_transmissions;
    if (diagnostics.state != TCAN337_ERROR_ACTIVE || new_fault ||
        diagnostics.transceiver_fault) {
        ESP_LOGW("can_health",
                 "TCAN337 state=%s TEC=%u REC=%u bus_err=%" PRIu32
                 " error=0x%02" PRIX32 " rx=%" PRIu32 " dropped=%" PRIu32
                 " tx=%" PRIu32 " tx_failed=%" PRIu32 "%s",
                 tcan337_bus_state_name(diagnostics.state),
                 diagnostics.tx_error_count, diagnostics.rx_error_count,
                 diagnostics.bus_error_count, diagnostics.last_error_flags,
                 diagnostics.received_frames, diagnostics.dropped_frames,
                 diagnostics.transmitted_frames,
                 diagnostics.failed_transmissions,
                 diagnostics.transceiver_fault ? " FAULT=ACTIVE" : "");
    }

    if (diagnostics.state == TCAN337_BUS_OFF && !recovery_requested) {
        err = tcan337_recover();
        if (err == ESP_OK) {
            ESP_LOGW("can_health", "TCAN337 bus-off recovery started");
            recovery_requested = true;
        } else {
            ESP_LOGE(TAG, "Could not start TCAN337 recovery: %s",
                     esp_err_to_name(err));
        }
    } else if (diagnostics.state == TCAN337_ERROR_ACTIVE) {
        recovery_requested = false;
    }

    previous_bus_errors = diagnostics.bus_error_count;
    previous_dropped_frames = diagnostics.dropped_frames;
    previous_failed_transmissions = diagnostics.failed_transmissions;
}

static void tcan337_logger_task(void* argument) {
    (void)argument;
    uint32_t received_count = 0;

    while (true) {
        tcan337_frame_t frame = {0};
        const esp_err_t err = tcan337_receive(&frame, 1000);
        if (err == ESP_OK) {
            log_tcan337_frame(&frame, ++received_count);
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "TCAN337 receive failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        log_tcan337_health();
    }
}

static void start_tcan337_verification(void) {
    const tcan337_config_t config = {
        .tx_io = TCAN337_TX_IO,
        .rx_io = TCAN337_RX_IO,
        .fault_io = TCAN337_FAULT_IO,
        .silent_io = TCAN337_SILENT_IO,
        .bitrate = TCAN337_BITRATE,
    };

    esp_err_t err = tcan337_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCAN337/TWAI initialization FAILED: %s",
                 esp_err_to_name(err));
        return;
    }

    err = tcan337_run_loopback_test(TCAN337_BEACON_ADDRESS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "TCAN337 self-reception FAILED (%s). Check TXD/RXD, S=low, "
                 "CANH/CANL, power, and termination.",
                 esp_err_to_name(err));
    }

    err = tcan337_start(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start TCAN337 receiver: %s",
                 esp_err_to_name(err));
        return;
    }

    const BaseType_t task_created =
        xTaskCreate(tcan337_logger_task, "tcan337_logger", 4096, NULL, 5, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Could not create TCAN337 logging task");
    }
}

static void log_can_frame(const tcan4550_frame_t* frame, uint32_t sequence) {
    char payload[64U * 3U + 1U] = {0};
    size_t offset = 0;
    for (uint8_t index = 0; index < frame->data_length; ++index) {
        const int written =
            snprintf(&payload[offset], sizeof(payload) - offset, "%s%02X",
                     index == 0 ? "" : " ", frame->data[index]);
        if (written < 0 || (size_t)written >= sizeof(payload) - offset) {
            break;
        }
        offset += (size_t)written;
    }

    ESP_LOGI("can_rx",
             "#%06" PRIu32 " ID=0x%08" PRIX32
             " %s %s%s%s%s%s DLC=%u len=%u timestamp=0x%04X "
             "filter=%s%u data=[%s]",
             sequence, frame->id, frame->extended ? "EXT" : "STD",
             frame->fd_format ? "CAN-FD" : "CAN",
             frame->bit_rate_switch ? " BRS" : "", frame->remote ? " RTR" : "",
             frame->error_state_indicator ? " ESI" : "",
             frame->rx_fifo_message_lost ? " FIFO_LOSS" : "",
             frame->data_length_code, frame->data_length, frame->timestamp,
             frame->accepted_non_matching ? "non-match/" : "",
             frame->filter_index, payload);
}

static void log_can_health(void) {
    static bool bus_silent_reported = false;
    tcan4550_diagnostics_t diagnostics = {0};
    esp_err_t err = tcan4550_get_diagnostics(&diagnostics);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not read TCAN4550 diagnostics: %s",
                 esp_err_to_name(err));
        return;
    }

    const uint32_t unexpected_mcan =
        diagnostics.mcan_interrupts & ~TCAN_MCAN_IR_EXPECTED;
    const uint32_t last_error = diagnostics.protocol_status & 0x7U;
    const uint32_t data_last_error = (diagnostics.protocol_status >> 8) & 0x7U;
    const bool protocol_error = (diagnostics.protocol_status &
                                 ((1UL << 7) | (1UL << 6) | (1UL << 5))) != 0 ||
                                (last_error != 0 && last_error != 7) ||
                                (data_last_error != 0 && data_last_error != 7);

    if ((diagnostics.device_interrupts & TCAN_DEVICE_IR_CAN_SILENT) != 0 &&
        !bus_silent_reported) {
        ESP_LOGI("can_health",
                 "CAN bus is silent; this is expected until another CAN node "
                 "transmits");
        bus_silent_reported = true;
    }

    if ((diagnostics.spi_status & TCAN_SPI_STATUS_ERRORS) != 0 ||
        (diagnostics.device_interrupts & TCAN_DEVICE_IR_FAULTS) != 0 ||
        (unexpected_mcan & TCAN_MCAN_IR_ERRORS) != 0 || protocol_error) {
        ESP_LOGW("can_health",
                 "SPI=0x%08" PRIX32 " DEV_IR=0x%08" PRIX32
                 " MCAN_IR=0x%08" PRIX32 " PSR=0x%08" PRIX32
                 " ECR=0x%08" PRIX32,
                 diagnostics.spi_status, diagnostics.device_interrupts,
                 diagnostics.mcan_interrupts, diagnostics.protocol_status,
                 diagnostics.error_counters);
    }

    if (diagnostics.mcan_interrupts != 0) {
        err = tcan4550_clear_mcan_interrupts(diagnostics.mcan_interrupts);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Could not clear M_CAN interrupts: %s",
                     esp_err_to_name(err));
        }
    }
    if (diagnostics.device_interrupts != 0) {
        err = tcan4550_clear_device_interrupts(diagnostics.device_interrupts);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Could not clear TCAN4550 interrupts: %s",
                     esp_err_to_name(err));
        }
    }
}

static void can_logger_task(void* argument) {
    (void)argument;
    uint32_t received_count = 0;

    while (true) {
        tcan4550_frame_t frame = {0};
        esp_err_t err = tcan4550_receive(&frame, 1000);
        if (err == ESP_OK) {
            log_can_frame(&frame, ++received_count);
        } else if (err != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "CAN receive failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        log_can_health();
    }
}

static void start_tcan4550_verification(void) {
    const tcan4550_config_t config = {
        .spi_host = SPI2_HOST,
        .cs_io = TCAN4550_CS_IO,
        .mosi_io = TCAN4550_MOSI_IO,
        .miso_io = TCAN4550_MISO_IO,
        .sclk_io = TCAN4550_SCLK_IO,
        .spi_clock_hz = 2000000U,
        .oscillator_hz = TCAN4550_OSCILLATOR_HZ,
        .nominal_bitrate = TCAN4550_NOMINAL_BITRATE,
        .data_bitrate = TCAN4550_DATA_BITRATE,
    };

    esp_err_t err = tcan4550_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCAN4550 SPI/initialization test FAILED: %s",
                 esp_err_to_name(err));
        return;
    }

    err = tcan4550_run_loopback_test(TCAN4550_LOOPBACK_INTERNAL,
                                     TCAN4550_BEACON_ADDRESS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TCAN4550 internal loopback FAILED: %s",
                 esp_err_to_name(err));
        return;
    }

#if TCAN4550_RUN_EXTERNAL_LOOPBACK_TEST
    err = tcan4550_run_loopback_test(TCAN4550_LOOPBACK_EXTERNAL,
                                     TCAN4550_BEACON_ADDRESS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG,
                 "TCAN4550 external loopback FAILED (%s). SPI/controller are "
                 "working; check CANH/CANL and termination.",
                 esp_err_to_name(err));
    }
#endif

    err = tcan4550_start(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start TCAN4550 receiver: %s",
                 esp_err_to_name(err));
        return;
    }

    const BaseType_t task_created =
        xTaskCreate(can_logger_task, "can_logger", 4096, NULL, 5, NULL);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Could not create CAN logging task");
    }
}

void app_main(void) {
    VigilantConfig VgConfig = {.unique_component_name = "Vigilant ESP Test",
                               .network_mode = NW_MODE_APSTA};
    ESP_ERROR_CHECK(vigilant_init(VgConfig));
    start_tcan4550_verification();
    start_tcan337_verification();
}
