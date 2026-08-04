#include <inttypes.h>
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "status_led.h"
#include "tcan4550.h"
#include "vigilant.h"

/* Hardware connections supplied for the ESP32-S3. */
#define TCAN4550_CS_IO GPIO_NUM_10
#define TCAN4550_MOSI_IO GPIO_NUM_11
#define TCAN4550_SCLK_IO GPIO_NUM_12
#define TCAN4550_MISO_IO GPIO_NUM_13

/*
 * This is the CAN identifier used by both loopback verification frames.
 * 0x000..0x7FF selects an 11-bit ID; a larger value selects a 29-bit ID.
 */
#define TCAN4550_BEACON_ADDRESS 0x123U

/* Change this to 20000000 if the board uses a 20 MHz crystal/clock. */
#define TCAN4550_OSCILLATOR_HZ 40000000U
#define TCAN4550_NOMINAL_BITRATE 500000U
#define TCAN4550_DATA_BITRATE 2000000U

/* External loopback exercises the transceiver and CANH/CANL path. */
#define TCAN4550_RUN_EXTERNAL_LOOPBACK_TEST 1

#define TCAN_MCAN_IR_EXPECTED \
    ((1UL << 16) | (1UL << 9) | (1UL << 2) | (1UL << 0))
#define TCAN_MCAN_IR_ERRORS 0x3FEF00F8UL
#define TCAN_SPI_STATUS_ERRORS 0x3F3F0000UL
#define TCAN_DEVICE_IR_FAULTS 0x00ED2109UL
#define TCAN_DEVICE_IR_CAN_SILENT (1UL << 10)

static const char* TAG = "app_main";

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
}
