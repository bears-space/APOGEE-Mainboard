#include "tcan337.h"

#include <inttypes.h>
#include <stdatomic.h>
#include <stddef.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define TCAN337_RX_QUEUE_DEPTH 32U
#define TCAN337_TX_QUEUE_DEPTH 4U
#define TCAN337_MAX_BITRATE 1000000U
#define TCAN337_TIMESTAMP_HZ 1000000U

typedef enum {
    TCAN337_STATE_UNINITIALIZED,
    TCAN337_STATE_READY,
    TCAN337_STATE_SELF_TEST,
    TCAN337_STATE_RUNNING,
} tcan337_driver_state_t;

typedef struct {
    tcan337_config_t config;
    twai_node_handle_t node;
    QueueHandle_t rx_queue;
    SemaphoreHandle_t mutex;
    tcan337_driver_state_t driver_state;
    bool node_enabled;
    bool listen_only;
    atomic_uint_least32_t received_frames;
    atomic_uint_least32_t dropped_frames;
    atomic_uint_least32_t transmitted_frames;
    atomic_uint_least32_t failed_transmissions;
    atomic_uint_least32_t last_error_flags;
} tcan337_context_t;

static const char* TAG = "tcan337";
static tcan337_context_t s_tcan337;

static void reset_runtime_statistics(void) {
    atomic_store_explicit(&s_tcan337.received_frames, 0, memory_order_relaxed);
    atomic_store_explicit(&s_tcan337.dropped_frames, 0, memory_order_relaxed);
    atomic_store_explicit(&s_tcan337.transmitted_frames, 0,
                          memory_order_relaxed);
    atomic_store_explicit(&s_tcan337.failed_transmissions, 0,
                          memory_order_relaxed);
    atomic_store_explicit(&s_tcan337.last_error_flags, 0, memory_order_relaxed);
}

static bool gpio_is_optional_input(gpio_num_t gpio) {
    return gpio == GPIO_NUM_NC || GPIO_IS_VALID_GPIO(gpio);
}

static bool gpio_is_optional_output(gpio_num_t gpio) {
    return gpio == GPIO_NUM_NC || GPIO_IS_VALID_OUTPUT_GPIO(gpio);
}

static tcan337_bus_state_t map_bus_state(twai_error_state_t state) {
    switch (state) {
        case TWAI_ERROR_WARNING:
            return TCAN337_ERROR_WARNING;
        case TWAI_ERROR_PASSIVE:
            return TCAN337_ERROR_PASSIVE;
        case TWAI_ERROR_BUS_OFF:
            return TCAN337_BUS_OFF;
        case TWAI_ERROR_ACTIVE:
        default:
            return TCAN337_ERROR_ACTIVE;
    }
}

static bool IRAM_ATTR on_rx_done(twai_node_handle_t node,
                                 const twai_rx_done_event_data_t* event,
                                 void* user_context) {
    (void)event;
    tcan337_context_t* context = user_context;
    tcan337_frame_t received = {0};
    twai_frame_t frame = {
        .buffer = received.data,
        .buffer_len = sizeof(received.data),
    };

    if (twai_node_receive_from_isr(node, &frame) != ESP_OK) {
        atomic_fetch_add_explicit(&context->dropped_frames, 1,
                                  memory_order_relaxed);
        return false;
    }

    received.id = frame.header.id;
    received.extended = frame.header.ide;
    received.remote = frame.header.rtr;
    received.data_length = frame.header.dlc <= sizeof(received.data)
                               ? (uint8_t)frame.header.dlc
                               : sizeof(received.data);
    received.timestamp_us = frame.header.timestamp;

    BaseType_t task_woken = pdFALSE;
    if (xQueueSendFromISR(context->rx_queue, &received, &task_woken) ==
        pdTRUE) {
        atomic_fetch_add_explicit(&context->received_frames, 1,
                                  memory_order_relaxed);
    } else {
        atomic_fetch_add_explicit(&context->dropped_frames, 1,
                                  memory_order_relaxed);
    }
    return task_woken == pdTRUE;
}

static bool IRAM_ATTR on_tx_done(twai_node_handle_t node,
                                 const twai_tx_done_event_data_t* event,
                                 void* user_context) {
    (void)node;
    tcan337_context_t* context = user_context;
    if (event->is_tx_success) {
        atomic_fetch_add_explicit(&context->transmitted_frames, 1,
                                  memory_order_relaxed);
    } else {
        atomic_fetch_add_explicit(&context->failed_transmissions, 1,
                                  memory_order_relaxed);
    }
    return false;
}

static bool IRAM_ATTR on_error(twai_node_handle_t node,
                               const twai_error_event_data_t* event,
                               void* user_context) {
    (void)node;
    tcan337_context_t* context = user_context;
    atomic_fetch_or_explicit(&context->last_error_flags, event->err_flags.val,
                             memory_order_relaxed);
    return false;
}

static esp_err_t configure_optional_pins(const tcan337_config_t* config) {
    if (config->silent_io != GPIO_NUM_NC) {
        const gpio_config_t silent_config = {
            .pin_bit_mask = 1ULL << config->silent_io,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&silent_config);
        if (err != ESP_OK) {
            return err;
        }
        err = gpio_set_level(config->silent_io, 0);
        if (err != ESP_OK) {
            return err;
        }
    }

    if (config->fault_io != GPIO_NUM_NC) {
        const gpio_config_t fault_config = {
            .pin_bit_mask = 1ULL << config->fault_io,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        return gpio_config(&fault_config);
    }
    return ESP_OK;
}

static esp_err_t create_node(bool self_test, bool loopback, bool listen_only) {
    const twai_onchip_node_config_t node_config = {
        .io_cfg =
            {
                .tx = s_tcan337.config.tx_io,
                .rx = s_tcan337.config.rx_io,
                .quanta_clk_out = GPIO_NUM_NC,
                .bus_off_indicator = GPIO_NUM_NC,
            },
        .bit_timing =
            {
                .bitrate = s_tcan337.config.bitrate,
            },
        .timestamp_resolution_hz = TCAN337_TIMESTAMP_HZ,
        /* Single-shot keeps an unacknowledged frame from retaining its pointer.
         */
        .fail_retry_cnt = 0,
        .tx_queue_depth = TCAN337_TX_QUEUE_DEPTH,
        .flags =
            {
                .enable_self_test = self_test,
                .enable_loopback = loopback,
                .enable_listen_only = listen_only,
            },
    };

    esp_err_t err = twai_new_node_onchip(&node_config, &s_tcan337.node);
    if (err != ESP_OK) {
        s_tcan337.node = NULL;
        return err;
    }

    const twai_event_callbacks_t callbacks = {
        .on_tx_done = on_tx_done,
        .on_rx_done = on_rx_done,
        .on_error = on_error,
    };
    err = twai_node_register_event_callbacks(s_tcan337.node, &callbacks,
                                             &s_tcan337);
    if (err != ESP_OK) {
        (void)twai_node_delete(s_tcan337.node);
        s_tcan337.node = NULL;
    }
    return err;
}

static esp_err_t destroy_node(void) {
    if (s_tcan337.node == NULL) {
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;
    if (s_tcan337.node_enabled) {
        err = twai_node_disable(s_tcan337.node);
        if (err == ESP_ERR_INVALID_STATE) {
            /* A bus-off controller is already disabled in hardware. */
            err = ESP_OK;
        }
        s_tcan337.node_enabled = false;
    }
    if (err == ESP_OK) {
        err = twai_node_delete(s_tcan337.node);
    }
    if (err == ESP_OK) {
        s_tcan337.node = NULL;
    }
    return err;
}

static esp_err_t validate_frame(const tcan337_frame_t* frame) {
    if (frame == NULL || frame->data_length > sizeof(frame->data)) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((!frame->extended && frame->id > TWAI_STD_ID_MASK) ||
        (frame->extended && frame->id > TWAI_EXT_ID_MASK)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t transmit_locked(const tcan337_frame_t* frame,
                                 uint32_t timeout_ms) {
    esp_err_t err = validate_frame(frame);
    if (err != ESP_OK) {
        return err;
    }

    twai_frame_t twai_frame = {
        .header =
            {
                .id = frame->id,
                .dlc = frame->data_length,
                .ide = frame->extended,
                .rtr = frame->remote,
            },
        .buffer = frame->remote ? NULL : (uint8_t*)frame->data,
        .buffer_len = frame->remote ? 0 : frame->data_length,
    };
    const uint32_t failed_before = atomic_load_explicit(
        &s_tcan337.failed_transmissions, memory_order_relaxed);

    err = twai_node_transmit(s_tcan337.node, &twai_frame, (int)timeout_ms);
    if (err == ESP_OK) {
        err = twai_node_transmit_wait_all_done(s_tcan337.node, (int)timeout_ms);
    }
    if (err == ESP_OK &&
        atomic_load_explicit(&s_tcan337.failed_transmissions,
                             memory_order_relaxed) != failed_before) {
        err = ESP_FAIL;
    }
    return err;
}

esp_err_t tcan337_init(const tcan337_config_t* config) {
    if (config == NULL || !GPIO_IS_VALID_OUTPUT_GPIO(config->tx_io) ||
        !GPIO_IS_VALID_GPIO(config->rx_io) ||
        !gpio_is_optional_input(config->fault_io) ||
        !gpio_is_optional_output(config->silent_io) || config->bitrate == 0 ||
        config->bitrate > TCAN337_MAX_BITRATE) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_tcan337.driver_state != TCAN337_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }

    s_tcan337.config = *config;
    atomic_init(&s_tcan337.received_frames, 0);
    atomic_init(&s_tcan337.dropped_frames, 0);
    atomic_init(&s_tcan337.transmitted_frames, 0);
    atomic_init(&s_tcan337.failed_transmissions, 0);
    atomic_init(&s_tcan337.last_error_flags, 0);
    s_tcan337.rx_queue =
        xQueueCreate(TCAN337_RX_QUEUE_DEPTH, sizeof(tcan337_frame_t));
    s_tcan337.mutex = xSemaphoreCreateMutex();
    if (s_tcan337.rx_queue == NULL || s_tcan337.mutex == NULL) {
        if (s_tcan337.rx_queue != NULL) {
            vQueueDelete(s_tcan337.rx_queue);
        }
        if (s_tcan337.mutex != NULL) {
            vSemaphoreDelete(s_tcan337.mutex);
        }
        memset(&s_tcan337, 0, sizeof(s_tcan337));
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = configure_optional_pins(config);
    if (err == ESP_OK) {
        err = create_node(true, true, false);
    }
    if (err != ESP_OK) {
        vQueueDelete(s_tcan337.rx_queue);
        vSemaphoreDelete(s_tcan337.mutex);
        memset(&s_tcan337, 0, sizeof(s_tcan337));
        return err;
    }

    s_tcan337.driver_state = TCAN337_STATE_READY;
    ESP_LOGI(TAG,
             "ESP32-S3 TWAI initialized: TX GPIO%d, RX GPIO%d, %" PRIu32
             " bit/s",
             config->tx_io, config->rx_io, config->bitrate);
    return ESP_OK;
}

esp_err_t tcan337_run_loopback_test(uint32_t beacon_id) {
    if (s_tcan337.driver_state != TCAN337_STATE_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    if (beacon_id > TWAI_EXT_ID_MASK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_tcan337.mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    xQueueReset(s_tcan337.rx_queue);
    esp_err_t err = twai_node_enable(s_tcan337.node);
    if (err == ESP_OK) {
        s_tcan337.node_enabled = true;
        s_tcan337.driver_state = TCAN337_STATE_SELF_TEST;
    }

    const tcan337_frame_t sent = {
        .id = beacon_id,
        .extended = beacon_id > TWAI_STD_ID_MASK,
        .data_length = 8,
        .data = {'T', 'C', 'A', 'N', '3', '3', '7', 'T'},
    };
    if (err == ESP_OK) {
        err = transmit_locked(&sent, 250);
    }

    tcan337_frame_t received = {0};
    if (err == ESP_OK && xQueueReceive(s_tcan337.rx_queue, &received,
                                       pdMS_TO_TICKS(250)) != pdTRUE) {
        err = ESP_ERR_TIMEOUT;
    }
    if (err == ESP_OK &&
        (received.id != sent.id || received.extended != sent.extended ||
         received.remote || received.data_length != sent.data_length ||
         memcmp(received.data, sent.data, sent.data_length) != 0)) {
        err = ESP_ERR_INVALID_RESPONSE;
    }

    if (s_tcan337.node_enabled) {
        const esp_err_t disable_err = twai_node_disable(s_tcan337.node);
        s_tcan337.node_enabled = false;
        if (err == ESP_OK) {
            err = disable_err;
        }
    }
    s_tcan337.driver_state = TCAN337_STATE_READY;
    xSemaphoreGive(s_tcan337.mutex);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Self-reception test passed with ID 0x%08" PRIX32,
                 beacon_id);
    }
    return err;
}

esp_err_t tcan337_start(bool listen_only) {
    if (s_tcan337.driver_state != TCAN337_STATE_READY) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tcan337.mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = destroy_node();
    if (err == ESP_OK) {
        xQueueReset(s_tcan337.rx_queue);
        err = create_node(false, false, listen_only);
    }
    if (err == ESP_OK) {
        /* Do not include boot self-test traffic in normal-mode diagnostics. */
        reset_runtime_statistics();
    }
    if (err == ESP_OK) {
        err = twai_node_enable(s_tcan337.node);
    }
    if (err == ESP_OK) {
        s_tcan337.node_enabled = true;
        s_tcan337.listen_only = listen_only;
        s_tcan337.driver_state = TCAN337_STATE_RUNNING;
    }
    xSemaphoreGive(s_tcan337.mutex);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CAN receiver started in %s mode",
                 listen_only ? "listen-only" : "normal");
    }
    return err;
}

esp_err_t tcan337_transmit(const tcan337_frame_t* frame, uint32_t timeout_ms) {
    if (s_tcan337.driver_state != TCAN337_STATE_RUNNING ||
        s_tcan337.listen_only) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tcan337.mutex, pdMS_TO_TICKS(timeout_ms + 100U)) !=
        pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err = transmit_locked(frame, timeout_ms);
    xSemaphoreGive(s_tcan337.mutex);
    return err;
}

esp_err_t tcan337_receive(tcan337_frame_t* frame, uint32_t timeout_ms) {
    if (s_tcan337.driver_state != TCAN337_STATE_RUNNING || frame == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueReceive(s_tcan337.rx_queue, frame, pdMS_TO_TICKS(timeout_ms)) !=
        pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t tcan337_get_diagnostics(tcan337_diagnostics_t* diagnostics) {
    if (s_tcan337.driver_state != TCAN337_STATE_RUNNING ||
        diagnostics == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_node_status_t status = {0};
    twai_node_record_t record = {0};
    esp_err_t err = twai_node_get_info(s_tcan337.node, &status, &record);
    if (err != ESP_OK) {
        return err;
    }

    *diagnostics = (tcan337_diagnostics_t){
        .state = map_bus_state(status.state),
        .tx_error_count = status.tx_error_count,
        .rx_error_count = status.rx_error_count,
        .bus_error_count = record.bus_err_num,
        .last_error_flags = atomic_exchange_explicit(
            &s_tcan337.last_error_flags, 0, memory_order_relaxed),
        .received_frames = atomic_load_explicit(&s_tcan337.received_frames,
                                                memory_order_relaxed),
        .dropped_frames = atomic_load_explicit(&s_tcan337.dropped_frames,
                                               memory_order_relaxed),
        .transmitted_frames = atomic_load_explicit(
            &s_tcan337.transmitted_frames, memory_order_relaxed),
        .failed_transmissions = atomic_load_explicit(
            &s_tcan337.failed_transmissions, memory_order_relaxed),
        .fault_monitoring_available = s_tcan337.config.fault_io != GPIO_NUM_NC,
        .transceiver_fault = s_tcan337.config.fault_io != GPIO_NUM_NC &&
                             gpio_get_level(s_tcan337.config.fault_io) != 0,
    };
    return ESP_OK;
}

esp_err_t tcan337_recover(void) {
    if (s_tcan337.driver_state != TCAN337_STATE_RUNNING) {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_node_recover(s_tcan337.node);
}
