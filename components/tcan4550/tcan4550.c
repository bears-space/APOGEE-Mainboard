#include "tcan4550.h"

#include <inttypes.h>
#include <stddef.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define TCAN_SPI_READ_OPCODE 0x41U
#define TCAN_SPI_WRITE_OPCODE 0x61U
#define TCAN_SPI_MAX_WORDS 32U
#define TCAN_SPI_HEADER_BYTES 4U

#define TCAN_REG_DEVICE_ID1 0x0000U
#define TCAN_REG_DEVICE_ID2 0x0004U
#define TCAN_REG_REVISION 0x0008U
#define TCAN_REG_SPI_STATUS 0x000CU
#define TCAN_REG_MODES 0x0800U
#define TCAN_REG_SCRATCH 0x0808U
#define TCAN_REG_DEVICE_IR 0x0820U

#define TCAN_REG_DBTP 0x100CU
#define TCAN_REG_TEST 0x1010U
#define TCAN_REG_CCCR 0x1018U
#define TCAN_REG_NBTP 0x101CU
#define TCAN_REG_TSCC 0x1020U
#define TCAN_REG_ECR 0x1040U
#define TCAN_REG_PSR 0x1044U
#define TCAN_REG_TDCR 0x1048U
#define TCAN_REG_IR 0x1050U
#define TCAN_REG_IE 0x1054U
#define TCAN_REG_ILS 0x1058U
#define TCAN_REG_ILE 0x105CU
#define TCAN_REG_GFC 0x1080U
#define TCAN_REG_SIDFC 0x1084U
#define TCAN_REG_XIDFC 0x1088U
#define TCAN_REG_XIDAM 0x1090U
#define TCAN_REG_RXF0C 0x10A0U
#define TCAN_REG_RXF0S 0x10A4U
#define TCAN_REG_RXF0A 0x10A8U
#define TCAN_REG_RXBC 0x10ACU
#define TCAN_REG_RXF1C 0x10B0U
#define TCAN_REG_RXESC 0x10BCU
#define TCAN_REG_TXBC 0x10C0U
#define TCAN_REG_TXESC 0x10C8U
#define TCAN_REG_TXBRP 0x10CCU
#define TCAN_REG_TXBAR 0x10D0U
#define TCAN_REG_TXBCR 0x10D4U
#define TCAN_REG_TXBTO 0x10D8U
#define TCAN_REG_TXEFC 0x10F0U

#define TCAN_MRAM_BASE 0x8000U
#define TCAN_MRAM_SIZE_BYTES 2048U
#define TCAN_MRAM_ELEMENT_BYTES 72U
#define TCAN_RX_FIFO_ELEMENTS 16U
#define TCAN_RX_FIFO_START 0x0000U
#define TCAN_TX_BUFFER_START \
    (TCAN_RX_FIFO_START + TCAN_RX_FIFO_ELEMENTS * TCAN_MRAM_ELEMENT_BYTES)

#define TCAN_DEVICE_ID1_VALUE 0x4E414354UL
#define TCAN_DEVICE_ID2_VALUE 0x30353534UL
#define TCAN_ENDIAN_VALUE 0x87654321UL
#define TCAN_REG_ENDN 0x1004U

#define TCAN_MODES_CLK_REF_40MHZ (1UL << 27)
#define TCAN_MODES_TEST_MODE_EN (1UL << 21)
#define TCAN_MODES_MODE_MASK (3UL << 6)
#define TCAN_MODES_MODE_NORMAL (2UL << 6)
#define TCAN_MODES_RESERVED_SET (1UL << 5)
#define TCAN_MODES_WATCHDOG_EN (1UL << 3)
#define TCAN_MODES_DEVICE_RESET (1UL << 2)
#define TCAN_MODES_TEST_CONFIG (1UL << 0)

#define TCAN_CCCR_BRSE (1UL << 9)
#define TCAN_CCCR_FDOE (1UL << 8)
#define TCAN_CCCR_TEST (1UL << 7)
#define TCAN_CCCR_MON (1UL << 5)
#define TCAN_CCCR_CSR (1UL << 4)
#define TCAN_CCCR_CSA (1UL << 3)
#define TCAN_CCCR_CCE (1UL << 1)
#define TCAN_CCCR_INIT (1UL << 0)
#define TCAN_TEST_LBCK (1UL << 4)

#define TCAN_RXF0S_LOST (1UL << 25)
#define TCAN_RXF0S_GET_INDEX_MASK (0x3FUL << 8)
#define TCAN_RXF0S_FILL_MASK 0x7FUL
#define TCAN_IR_RF0N (1UL << 0)
#define TCAN_IR_RF0L (1UL << 3)

#define TCAN_TX_BUFFER_0 (1UL << 0)
#define TCAN_SPI_ERROR_W1C_MASK 0x3F3F0000UL
#define TCAN_DEVICE_IR_W1C_MASK 0x00FDE500UL

typedef enum {
    TCAN_STATE_UNINITIALIZED,
    TCAN_STATE_INTERNAL_LOOPBACK,
    TCAN_STATE_EXTERNAL_LOOPBACK,
    TCAN_STATE_RUNNING,
} tcan_state_t;

typedef struct {
    tcan4550_config_t config;
    spi_device_handle_t spi;
    SemaphoreHandle_t mutex;
    tcan_state_t state;
    uint32_t nbtp;
    uint32_t dbtp;
    uint32_t tdcr;
    uint8_t last_global_status;
    uint8_t tx_buffer[TCAN_SPI_HEADER_BYTES + TCAN_SPI_MAX_WORDS * 4U]
        __attribute__((aligned(4)));
    uint8_t rx_buffer[TCAN_SPI_HEADER_BYTES + TCAN_SPI_MAX_WORDS * 4U]
        __attribute__((aligned(4)));
} tcan4550_context_t;

static const char* TAG = "tcan4550";
static tcan4550_context_t s_tcan;

static void encode_be32(uint8_t* destination, uint32_t value) {
    destination[0] = (uint8_t)(value >> 24);
    destination[1] = (uint8_t)(value >> 16);
    destination[2] = (uint8_t)(value >> 8);
    destination[3] = (uint8_t)value;
}

static uint32_t decode_be32(const uint8_t* source) {
    return ((uint32_t)source[0] << 24) | ((uint32_t)source[1] << 16) |
           ((uint32_t)source[2] << 8) | (uint32_t)source[3];
}

static esp_err_t spi_words(uint8_t opcode, uint16_t address,
                           const uint32_t* write_words, uint32_t* read_words,
                           size_t word_count) {
    if (word_count == 0 || word_count > TCAN_SPI_MAX_WORDS) {
        return ESP_ERR_INVALID_ARG;
    }

    const size_t transfer_bytes = TCAN_SPI_HEADER_BYTES + word_count * 4U;
    memset(s_tcan.tx_buffer, 0, transfer_bytes);
    memset(s_tcan.rx_buffer, 0, transfer_bytes);
    s_tcan.tx_buffer[0] = opcode;
    s_tcan.tx_buffer[1] = (uint8_t)(address >> 8);
    s_tcan.tx_buffer[2] = (uint8_t)address;
    s_tcan.tx_buffer[3] = (uint8_t)word_count;

    if (write_words != NULL) {
        for (size_t index = 0; index < word_count; ++index) {
            encode_be32(&s_tcan.tx_buffer[TCAN_SPI_HEADER_BYTES + index * 4U],
                        write_words[index]);
        }
    }

    spi_transaction_t transaction = {
        .length = transfer_bytes * 8U,
        .tx_buffer = s_tcan.tx_buffer,
        .rx_buffer = s_tcan.rx_buffer,
    };
    esp_err_t err = spi_device_polling_transmit(s_tcan.spi, &transaction);
    if (err != ESP_OK) {
        return err;
    }

    s_tcan.last_global_status = s_tcan.rx_buffer[0];
    if (read_words != NULL) {
        for (size_t index = 0; index < word_count; ++index) {
            read_words[index] = decode_be32(
                &s_tcan.rx_buffer[TCAN_SPI_HEADER_BYTES + index * 4U]);
        }
    }
    return ESP_OK;
}

static esp_err_t register_read(uint16_t address, uint32_t* value) {
    return spi_words(TCAN_SPI_READ_OPCODE, address, NULL, value, 1);
}

static esp_err_t register_write(uint16_t address, uint32_t value) {
    return spi_words(TCAN_SPI_WRITE_OPCODE, address, &value, NULL, 1);
}

static esp_err_t wait_for_register_bits(uint16_t address, uint32_t mask,
                                        uint32_t expected,
                                        uint32_t timeout_ms) {
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    do {
        uint32_t value = 0;
        ESP_RETURN_ON_ERROR(register_read(address, &value), TAG,
                            "Failed to read register 0x%04X", address);
        if ((value & mask) == expected) {
            return ESP_OK;
        }
        /* One scheduler tick is 10 ms with the project's 100 Hz tick rate. */
        vTaskDelay(1);
    } while ((xTaskGetTickCount() - start) <= timeout);

    return ESP_ERR_TIMEOUT;
}

static esp_err_t calculate_nominal_timing(uint32_t clock_hz, uint32_t bitrate,
                                          uint32_t* register_value) {
    if (clock_hz == 0 || bitrate == 0 || register_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint32_t prescaler = 1; prescaler <= 512; ++prescaler) {
        const uint64_t denominator = (uint64_t)bitrate * prescaler;
        if (denominator == 0 || ((uint64_t)clock_hz % denominator) != 0) {
            continue;
        }

        const uint32_t total_tq = (uint32_t)((uint64_t)clock_hz / denominator);
        const uint32_t before_sample = (total_tq * 80U + 50U) / 100U;
        if (before_sample < 3 || before_sample >= total_tq) {
            continue;
        }

        const uint32_t time_seg1 = before_sample - 1U;
        const uint32_t time_seg2 = total_tq - before_sample;
        if (time_seg1 < 2 || time_seg1 > 256 || time_seg2 < 2 ||
            time_seg2 > 128) {
            continue;
        }

        const uint32_t sync_jump_width = time_seg2;
        *register_value = ((sync_jump_width - 1U) << 25) |
                          ((prescaler - 1U) << 16) | ((time_seg1 - 1U) << 8) |
                          (time_seg2 - 1U);
        return ESP_OK;
    }

    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t calculate_data_timing(uint32_t clock_hz, uint32_t bitrate,
                                       uint32_t* dbtp, uint32_t* tdcr) {
    if (clock_hz == 0 || bitrate == 0 || dbtp == NULL || tdcr == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint32_t prescaler = 1; prescaler <= 32; ++prescaler) {
        const uint64_t denominator = (uint64_t)bitrate * prescaler;
        if (denominator == 0 || ((uint64_t)clock_hz % denominator) != 0) {
            continue;
        }

        const uint32_t total_tq = (uint32_t)((uint64_t)clock_hz / denominator);
        const uint32_t before_sample = (total_tq * 75U + 50U) / 100U;
        if (before_sample < 2 || before_sample >= total_tq) {
            continue;
        }

        const uint32_t time_seg1 = before_sample - 1U;
        const uint32_t time_seg2 = total_tq - before_sample;
        if (time_seg1 < 1 || time_seg1 > 32 || time_seg2 < 1 ||
            time_seg2 > 16) {
            continue;
        }

        const uint32_t sync_jump_width = time_seg2;
        *dbtp = (1UL << 23) | ((prescaler - 1U) << 16) |
                ((time_seg1 - 1U) << 8) | ((time_seg2 - 1U) << 4) |
                (sync_jump_width - 1U);
        *tdcr = (before_sample - 1U) << 8;
        return ESP_OK;
    }

    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t mcan_enter_initialization(void) {
    uint32_t cccr = 0;
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_CCCR, &cccr), TAG,
                        "Could not read CCCR");
    cccr &= ~(TCAN_CCCR_CSR | TCAN_CCCR_CSA);
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_CCCR, cccr | TCAN_CCCR_INIT),
                        TAG, "Could not request M_CAN initialization");
    ESP_RETURN_ON_ERROR(wait_for_register_bits(TCAN_REG_CCCR, TCAN_CCCR_INIT,
                                               TCAN_CCCR_INIT, 100),
                        TAG, "M_CAN did not enter initialization");
    ESP_RETURN_ON_ERROR(
        register_write(TCAN_REG_CCCR, (cccr | TCAN_CCCR_INIT | TCAN_CCCR_CCE) &
                                          ~(TCAN_CCCR_CSR | TCAN_CCCR_CSA)),
        TAG, "Could not unlock protected M_CAN registers");
    return wait_for_register_bits(TCAN_REG_CCCR, TCAN_CCCR_INIT | TCAN_CCCR_CCE,
                                  TCAN_CCCR_INIT | TCAN_CCCR_CCE, 100);
}

static esp_err_t mcan_leave_initialization(uint32_t cccr_mode) {
    ESP_RETURN_ON_ERROR(
        register_write(TCAN_REG_CCCR,
                       cccr_mode & ~(TCAN_CCCR_CSR | TCAN_CCCR_CSA |
                                     TCAN_CCCR_CCE | TCAN_CCCR_INIT)),
        TAG, "Could not start M_CAN");
    return wait_for_register_bits(TCAN_REG_CCCR, TCAN_CCCR_INIT, 0, 100);
}

static esp_err_t clear_message_ram(void) {
    uint32_t zero_words[TCAN_SPI_MAX_WORDS] = {0};
    for (uint16_t offset = 0; offset < TCAN_MRAM_SIZE_BYTES;
         offset += (uint16_t)sizeof(zero_words)) {
        ESP_RETURN_ON_ERROR(
            spi_words(TCAN_SPI_WRITE_OPCODE, TCAN_MRAM_BASE + offset,
                      zero_words, NULL, TCAN_SPI_MAX_WORDS),
            TAG, "Could not clear message RAM at 0x%04X",
            TCAN_MRAM_BASE + offset);
    }
    return ESP_OK;
}

static esp_err_t configure_message_ram(void) {
    ESP_RETURN_ON_ERROR(clear_message_ram(), TAG, "Message RAM clear failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_SIDFC, 0), TAG,
                        "SID filter configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_XIDFC, 0), TAG,
                        "XID filter configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_GFC, 0), TAG,
                        "Global filter configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_XIDAM, 0x1FFFFFFFUL), TAG,
                        "Extended ID mask configuration failed");
    ESP_RETURN_ON_ERROR(
        register_write(TCAN_REG_RXF0C,
                       (TCAN_RX_FIFO_ELEMENTS << 16) | TCAN_RX_FIFO_START),
        TAG, "RX FIFO 0 configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_RXF1C, 0), TAG,
                        "RX FIFO 1 configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_RXBC, 0), TAG,
                        "RX buffer configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_RXESC, 7), TAG,
                        "RX element size configuration failed");
    ESP_RETURN_ON_ERROR(
        register_write(TCAN_REG_TXBC,
                       (1UL << 16) | (uint32_t)TCAN_TX_BUFFER_START),
        TAG, "TX buffer configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_TXESC, 7), TAG,
                        "TX element size configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_TXEFC, 0), TAG,
                        "TX event FIFO configuration failed");
    return ESP_OK;
}

static esp_err_t configure_mcan(void) {
    ESP_RETURN_ON_ERROR(mcan_enter_initialization(), TAG,
                        "Could not enter M_CAN initialization");

    const uint32_t cccr =
        TCAN_CCCR_INIT | TCAN_CCCR_CCE | TCAN_CCCR_FDOE | TCAN_CCCR_BRSE;
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_CCCR, cccr), TAG,
                        "CAN-FD control configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_NBTP, s_tcan.nbtp), TAG,
                        "Nominal bit timing configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_DBTP, s_tcan.dbtp), TAG,
                        "Data bit timing configuration failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_TDCR, s_tcan.tdcr), TAG,
                        "Transmitter delay compensation failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_TSCC, 2), TAG,
                        "Timestamp configuration failed");
    ESP_RETURN_ON_ERROR(configure_message_ram(), TAG,
                        "Message RAM configuration failed");

    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_IE, 0), TAG,
                        "M_CAN interrupt disable failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_ILS, 0), TAG,
                        "M_CAN interrupt routing failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_ILE, 0), TAG,
                        "M_CAN interrupt output disable failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_IR, UINT32_MAX), TAG,
                        "M_CAN interrupt clear failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_TEST, 0), TAG,
                        "M_CAN test mode clear failed");

    ESP_RETURN_ON_ERROR(
        mcan_leave_initialization(TCAN_CCCR_FDOE | TCAN_CCCR_BRSE), TAG,
        "Could not start configured M_CAN");
    s_tcan.state = TCAN_STATE_RUNNING;
    return ESP_OK;
}

static esp_err_t configure_device_mode(void) {
    uint32_t device_interrupts = 0;
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_DEVICE_IR, &device_interrupts),
                        TAG, "Could not read device interrupts");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_DEVICE_IR, device_interrupts),
                        TAG, "Could not clear device interrupts");

    uint32_t modes = 0;
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_MODES, &modes), TAG,
                        "Could not read device modes");
    modes &= ~(TCAN_MODES_CLK_REF_40MHZ | TCAN_MODES_TEST_MODE_EN |
               TCAN_MODES_MODE_MASK | TCAN_MODES_WATCHDOG_EN |
               TCAN_MODES_DEVICE_RESET | TCAN_MODES_TEST_CONFIG);
    modes |= TCAN_MODES_RESERVED_SET | TCAN_MODES_MODE_NORMAL;
    if (s_tcan.config.oscillator_hz == 40000000U) {
        modes |= TCAN_MODES_CLK_REF_40MHZ;
    }
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_MODES, modes), TAG,
                        "Could not enter normal device mode");
    return wait_for_register_bits(TCAN_REG_MODES, TCAN_MODES_MODE_MASK,
                                  TCAN_MODES_MODE_NORMAL, 100);
}

static esp_err_t discard_received_frames(void) {
    for (uint32_t count = 0; count < TCAN_RX_FIFO_ELEMENTS; ++count) {
        uint32_t status = 0;
        ESP_RETURN_ON_ERROR(register_read(TCAN_REG_RXF0S, &status), TAG,
                            "Could not read RX FIFO status");
        if ((status & TCAN_RXF0S_FILL_MASK) == 0) {
            return ESP_OK;
        }
        const uint32_t get_index = (status & TCAN_RXF0S_GET_INDEX_MASK) >> 8;
        ESP_RETURN_ON_ERROR(register_write(TCAN_REG_RXF0A, get_index), TAG,
                            "Could not acknowledge RX FIFO");
    }
    return ESP_OK;
}

static esp_err_t set_mcan_mode(tcan_state_t state, bool monitor_only) {
    ESP_RETURN_ON_ERROR(mcan_enter_initialization(), TAG,
                        "Could not stop M_CAN to change mode");

    uint32_t cccr =
        TCAN_CCCR_INIT | TCAN_CCCR_CCE | TCAN_CCCR_FDOE | TCAN_CCCR_BRSE;
    uint32_t test = 0;
    if (state == TCAN_STATE_INTERNAL_LOOPBACK) {
        cccr |= TCAN_CCCR_TEST | TCAN_CCCR_MON;
        test = TCAN_TEST_LBCK;
    } else if (state == TCAN_STATE_EXTERNAL_LOOPBACK) {
        cccr |= TCAN_CCCR_TEST;
        test = TCAN_TEST_LBCK;
    } else if (monitor_only) {
        cccr |= TCAN_CCCR_MON;
    }

    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_CCCR, cccr), TAG,
                        "Could not configure M_CAN mode");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_TEST, test), TAG,
                        "Could not configure M_CAN loopback");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_IR, UINT32_MAX), TAG,
                        "Could not clear M_CAN flags");
    ESP_RETURN_ON_ERROR(mcan_leave_initialization(cccr), TAG,
                        "Could not activate M_CAN mode");
    ESP_RETURN_ON_ERROR(discard_received_frames(), TAG,
                        "Could not empty RX FIFO");
    s_tcan.state = state;
    return ESP_OK;
}

static uint8_t dlc_to_length(uint8_t dlc) {
    static const uint8_t lengths[] = {0, 1,  2,  3,  4,  5,  6,  7,
                                      8, 12, 16, 20, 24, 32, 48, 64};
    return dlc < sizeof(lengths) ? lengths[dlc] : 0;
}

static esp_err_t length_to_dlc(uint8_t length, uint8_t* dlc) {
    if (dlc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (length <= 8) {
        *dlc = length;
        return ESP_OK;
    }

    switch (length) {
        case 12:
            *dlc = 9;
            return ESP_OK;
        case 16:
            *dlc = 10;
            return ESP_OK;
        case 20:
            *dlc = 11;
            return ESP_OK;
        case 24:
            *dlc = 12;
            return ESP_OK;
        case 32:
            *dlc = 13;
            return ESP_OK;
        case 48:
            *dlc = 14;
            return ESP_OK;
        case 64:
            *dlc = 15;
            return ESP_OK;
        default:
            return ESP_ERR_INVALID_SIZE;
    }
}

static esp_err_t validate_frame(const tcan4550_frame_t* frame, uint8_t* dlc) {
    if (frame == NULL || dlc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((frame->extended && frame->id > 0x1FFFFFFFUL) ||
        (!frame->extended && frame->id > 0x7FFU)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!frame->fd_format &&
        (frame->data_length > 8 || frame->bit_rate_switch)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (frame->fd_format && frame->remote) {
        return ESP_ERR_INVALID_ARG;
    }
    return length_to_dlc(frame->data_length, dlc);
}

static esp_err_t transmit_locked(const tcan4550_frame_t* frame,
                                 uint32_t timeout_ms) {
    uint8_t dlc = 0;
    ESP_RETURN_ON_ERROR(validate_frame(frame, &dlc), TAG, "Invalid CAN frame");

    uint32_t pending = 0;
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_TXBRP, &pending), TAG,
                        "Could not read TX pending state");
    if ((pending & TCAN_TX_BUFFER_0) != 0) {
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t element[TCAN_MRAM_ELEMENT_BYTES / 4U] = {0};
    if (frame->extended) {
        element[0] = frame->id & 0x1FFFFFFFUL;
        element[0] |= 1UL << 30;
    } else {
        element[0] = (frame->id & 0x7FFU) << 18;
    }
    if (frame->remote) {
        element[0] |= 1UL << 29;
    }
    if (frame->error_state_indicator) {
        element[0] |= 1UL << 31;
    }
    element[1] = (uint32_t)dlc << 16;
    if (frame->bit_rate_switch) {
        element[1] |= 1UL << 20;
    }
    if (frame->fd_format) {
        element[1] |= 1UL << 21;
    }

    for (uint8_t index = 0; index < frame->data_length; ++index) {
        const size_t word_index = 2U + index / 4U;
        element[word_index] |= (uint32_t)frame->data[index]
                               << ((index % 4U) * 8U);
    }

    ESP_RETURN_ON_ERROR(
        spi_words(TCAN_SPI_WRITE_OPCODE,
                  TCAN_MRAM_BASE + (uint16_t)TCAN_TX_BUFFER_START, element,
                  NULL, sizeof(element) / sizeof(element[0])),
        TAG, "Could not write TX message RAM");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_TXBAR, TCAN_TX_BUFFER_0), TAG,
                        "Could not request transmission");

    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    do {
        uint32_t transmitted = 0;
        ESP_RETURN_ON_ERROR(register_read(TCAN_REG_TXBTO, &transmitted), TAG,
                            "Could not read TX completion");
        if ((transmitted & TCAN_TX_BUFFER_0) != 0) {
            return ESP_OK;
        }
        vTaskDelay(1);
    } while ((xTaskGetTickCount() - start) <= timeout);

    (void)register_write(TCAN_REG_TXBCR, TCAN_TX_BUFFER_0);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t receive_locked(tcan4550_frame_t* frame) {
    uint32_t fifo_status = 0;
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_RXF0S, &fifo_status), TAG,
                        "Could not read RX FIFO status");
    if ((fifo_status & TCAN_RXF0S_FILL_MASK) == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    const uint32_t get_index = (fifo_status & TCAN_RXF0S_GET_INDEX_MASK) >> 8;
    const uint16_t address = TCAN_MRAM_BASE + TCAN_RX_FIFO_START +
                             (uint16_t)(get_index * TCAN_MRAM_ELEMENT_BYTES);
    uint32_t element[TCAN_MRAM_ELEMENT_BYTES / 4U] = {0};
    ESP_RETURN_ON_ERROR(spi_words(TCAN_SPI_READ_OPCODE, address, NULL, element,
                                  sizeof(element) / sizeof(element[0])),
                        TAG, "Could not read RX message RAM");

    memset(frame, 0, sizeof(*frame));
    frame->error_state_indicator = (element[0] & (1UL << 31)) != 0;
    frame->extended = (element[0] & (1UL << 30)) != 0;
    frame->remote = (element[0] & (1UL << 29)) != 0;
    frame->id = frame->extended ? element[0] & 0x1FFFFFFFUL
                                : (element[0] >> 18) & 0x7FFU;
    frame->timestamp = (uint16_t)element[1];
    const uint8_t dlc = (uint8_t)((element[1] >> 16) & 0xFU);
    frame->data_length_code = dlc;
    frame->bit_rate_switch = (element[1] & (1UL << 20)) != 0;
    frame->fd_format = (element[1] & (1UL << 21)) != 0;
    frame->filter_index = (uint8_t)((element[1] >> 24) & 0x7FU);
    frame->accepted_non_matching = (element[1] & (1UL << 31)) != 0;
    frame->data_length = frame->remote ? 0 : dlc_to_length(dlc);
    frame->rx_fifo_message_lost = (fifo_status & TCAN_RXF0S_LOST) != 0;

    for (uint8_t index = 0; index < frame->data_length; ++index) {
        const size_t word_index = 2U + index / 4U;
        frame->data[index] =
            (uint8_t)(element[word_index] >> ((index % 4U) * 8U));
    }

    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_RXF0A, get_index), TAG,
                        "Could not acknowledge RX message");
    uint32_t clear_flags = TCAN_IR_RF0N;
    if ((fifo_status & TCAN_RXF0S_LOST) != 0) {
        clear_flags |= TCAN_IR_RF0L;
    }
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_IR, clear_flags), TAG,
                        "Could not clear RX flags");
    return ESP_OK;
}

esp_err_t tcan4550_init(const tcan4550_config_t* config) {
    if (config == NULL || config->spi_clock_hz == 0 ||
        config->spi_clock_hz > 18000000U ||
        (config->oscillator_hz != 20000000U &&
         config->oscillator_hz != 40000000U)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_tcan.state != TCAN_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&s_tcan, 0, sizeof(s_tcan));
    s_tcan.config = *config;
    ESP_RETURN_ON_ERROR(
        calculate_nominal_timing(config->oscillator_hz, config->nominal_bitrate,
                                 &s_tcan.nbtp),
        TAG, "Cannot represent requested nominal CAN bitrate");
    ESP_RETURN_ON_ERROR(
        calculate_data_timing(config->oscillator_hz, config->data_bitrate,
                              &s_tcan.dbtp, &s_tcan.tdcr),
        TAG, "Cannot represent requested CAN-FD data bitrate");

    spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_io,
        .miso_io_num = config->miso_io,
        .sclk_io_num = config->sclk_io,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = sizeof(s_tcan.tx_buffer),
    };
    esp_err_t err =
        spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = (int)config->spi_clock_hz,
        .mode = 0,
        .spics_io_num = config->cs_io,
        .queue_size = 1,
    };
    ESP_RETURN_ON_ERROR(
        spi_bus_add_device(config->spi_host, &device_config, &s_tcan.spi), TAG,
        "Could not add TCAN4550 to SPI bus");
    s_tcan.mutex = xSemaphoreCreateMutex();
    if (s_tcan.mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    uint32_t id1 = 0;
    uint32_t id2 = 0;
    uint32_t revision = 0;
    uint32_t endian = 0;
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_DEVICE_ID1, &id1), TAG,
                        "Device ID1 read failed");
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_DEVICE_ID2, &id2), TAG,
                        "Device ID2 read failed");
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_REVISION, &revision), TAG,
                        "Revision read failed");
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_ENDN, &endian), TAG,
                        "Endian register read failed");
    if (id1 != TCAN_DEVICE_ID1_VALUE || id2 != TCAN_DEVICE_ID2_VALUE ||
        endian != TCAN_ENDIAN_VALUE) {
        ESP_LOGE(TAG,
                 "Unexpected identity: ID1=0x%08" PRIX32 " ID2=0x%08" PRIX32
                 " ENDN=0x%08" PRIX32,
                 id1, id2, endian);
        return ESP_ERR_NOT_FOUND;
    }

    uint32_t old_scratch = 0;
    uint32_t scratch = 0;
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_SCRATCH, &old_scratch), TAG,
                        "Scratch register read failed");
    ESP_RETURN_ON_ERROR(register_write(TCAN_REG_SCRATCH, 0xA5C35A3CUL), TAG,
                        "Scratch register write failed");
    ESP_RETURN_ON_ERROR(register_read(TCAN_REG_SCRATCH, &scratch), TAG,
                        "Scratch register verification read failed");
    (void)register_write(TCAN_REG_SCRATCH, old_scratch);
    if (scratch != 0xA5C35A3CUL) {
        ESP_LOGE(TAG, "SPI write verification failed: read 0x%08" PRIX32,
                 scratch);
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(
        register_write(TCAN_REG_SPI_STATUS, TCAN_SPI_ERROR_W1C_MASK), TAG,
        "SPI error clear failed");
    ESP_RETURN_ON_ERROR(configure_device_mode(), TAG,
                        "TCAN4550 could not enter normal mode");
    ESP_RETURN_ON_ERROR(configure_mcan(), TAG, "M_CAN configuration failed");

    ESP_LOGI(TAG,
             "SPI verified: TCAN4550 revision=0x%08" PRIX32
             " global_status=0x%02X",
             revision, s_tcan.last_global_status);
    ESP_LOGI(TAG,
             "CAN timing: oscillator=%" PRIu32 " Hz nominal=%" PRIu32
             " bit/s data=%" PRIu32 " bit/s NBTP=0x%08" PRIX32
             " DBTP=0x%08" PRIX32,
             config->oscillator_hz, config->nominal_bitrate,
             config->data_bitrate, s_tcan.nbtp, s_tcan.dbtp);
    return ESP_OK;
}

esp_err_t tcan4550_run_loopback_test(tcan4550_loopback_t loopback,
                                     uint32_t beacon_id) {
    if (s_tcan.state == TCAN_STATE_UNINITIALIZED || beacon_id > 0x1FFFFFFFUL ||
        (loopback != TCAN4550_LOOPBACK_INTERNAL &&
         loopback != TCAN4550_LOOPBACK_EXTERNAL)) {
        return ESP_ERR_INVALID_ARG;
    }

    const tcan_state_t state = loopback == TCAN4550_LOOPBACK_INTERNAL
                                   ? TCAN_STATE_INTERNAL_LOOPBACK
                                   : TCAN_STATE_EXTERNAL_LOOPBACK;
    if (xSemaphoreTake(s_tcan.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = set_mcan_mode(state, false);
    tcan4550_frame_t sent = {
        .id = beacon_id,
        .extended = beacon_id > 0x7FFU,
        .fd_format = true,
        .bit_rate_switch = true,
        .data_length = 16,
        .data = {'T', 'C', 'A', 'N', '4', '5', '5', '0', 'S', 'E', 'L', 'F',
                 'T', 'E', 'S', 'T'},
    };
    if (err == ESP_OK) {
        sent.data[8] = loopback == TCAN4550_LOOPBACK_INTERNAL ? 'I' : 'E';
        err = transmit_locked(&sent, 250);
    }

    tcan4550_frame_t received = {0};
    if (err == ESP_OK) {
        const TickType_t start = xTaskGetTickCount();
        do {
            err = receive_locked(&received);
            if (err == ESP_OK) {
                break;
            }
            if (err != ESP_ERR_NOT_FOUND) {
                break;
            }
            vTaskDelay(1);
        } while ((xTaskGetTickCount() - start) <= pdMS_TO_TICKS(250));
        if (err == ESP_ERR_NOT_FOUND) {
            err = ESP_ERR_TIMEOUT;
        }
    }

    if (err == ESP_OK &&
        (received.id != sent.id || received.extended != sent.extended ||
         !received.fd_format || !received.bit_rate_switch ||
         received.data_length != sent.data_length ||
         memcmp(received.data, sent.data, sent.data_length) != 0)) {
        err = ESP_ERR_INVALID_RESPONSE;
    }
    xSemaphoreGive(s_tcan.mutex);

    if (err == ESP_OK) {
        ESP_LOGI(
            TAG, "%s loopback passed with CAN ID 0x%08" PRIX32,
            loopback == TCAN4550_LOOPBACK_INTERNAL ? "Internal" : "External",
            beacon_id);
    }
    return err;
}

esp_err_t tcan4550_start(bool monitor_only) {
    if (s_tcan.state == TCAN_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tcan.mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err = set_mcan_mode(TCAN_STATE_RUNNING, monitor_only);
    xSemaphoreGive(s_tcan.mutex);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CAN receiver started in %s mode",
                 monitor_only ? "passive monitor" : "normal");
    }
    return err;
}

esp_err_t tcan4550_transmit(const tcan4550_frame_t* frame,
                            uint32_t timeout_ms) {
    if (s_tcan.state == TCAN_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tcan.mutex, pdMS_TO_TICKS(timeout_ms + 100U)) !=
        pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err = transmit_locked(frame, timeout_ms);
    xSemaphoreGive(s_tcan.mutex);
    return err;
}

esp_err_t tcan4550_receive(tcan4550_frame_t* frame, uint32_t timeout_ms) {
    if (s_tcan.state == TCAN_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    do {
        if (xSemaphoreTake(s_tcan.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
        esp_err_t err = receive_locked(frame);
        xSemaphoreGive(s_tcan.mutex);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        if (err != ESP_ERR_NOT_FOUND) {
            return err;
        }
        if (timeout_ms == 0) {
            break;
        }
        /*
         * Do not convert a sub-tick delay with pdMS_TO_TICKS(): at 100 Hz,
         * 2 ms rounds down to zero and this polling loop starves the idle task.
         */
        vTaskDelay(1);
    } while ((xTaskGetTickCount() - start) <= timeout);

    return ESP_ERR_TIMEOUT;
}

esp_err_t tcan4550_get_diagnostics(tcan4550_diagnostics_t* diagnostics) {
    if (s_tcan.state == TCAN_STATE_UNINITIALIZED || diagnostics == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tcan.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err =
        register_read(TCAN_REG_SPI_STATUS, &diagnostics->spi_status);
    if (err == ESP_OK) {
        err =
            register_read(TCAN_REG_DEVICE_IR, &diagnostics->device_interrupts);
    }
    if (err == ESP_OK) {
        err = register_read(TCAN_REG_IR, &diagnostics->mcan_interrupts);
    }
    if (err == ESP_OK) {
        err = register_read(TCAN_REG_PSR, &diagnostics->protocol_status);
    }
    if (err == ESP_OK) {
        err = register_read(TCAN_REG_ECR, &diagnostics->error_counters);
    }
    xSemaphoreGive(s_tcan.mutex);
    return err;
}

esp_err_t tcan4550_clear_mcan_interrupts(uint32_t flags) {
    if (s_tcan.state == TCAN_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tcan.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err = register_write(TCAN_REG_IR, flags);
    xSemaphoreGive(s_tcan.mutex);
    return err;
}

esp_err_t tcan4550_clear_device_interrupts(uint32_t flags) {
    if (s_tcan.state == TCAN_STATE_UNINITIALIZED) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_tcan.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    const esp_err_t err =
        register_write(TCAN_REG_DEVICE_IR, flags & TCAN_DEVICE_IR_W1C_MASK);
    xSemaphoreGive(s_tcan.mutex);
    return err;
}
