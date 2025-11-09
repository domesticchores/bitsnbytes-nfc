#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"       

#define SCL_PIN             (22)
#define SDA_PIN             (21)
#define RESET_PIN           (-1)
#define IRQ_PIN             (-1)

#define NFC_TIMEOUT         (300)
#define MAX_UID_LENGTH      (10)

#include "sdkconfig.h"
#include "pn532_driver_i2c.h"
#include "pn532.h"

static const char *TAG = "BNB_MAIN";
static const char *UART_TAG = "BNB_UART";
static const char *I2C_TAG = "BNB_I2C";

struct nfc_resp {
    uint8_t uid[MAX_UID_LENGTH];
    uint8_t length;
};

pn532_io_t pn532_io; 

void pn532_i2c_init() {
    esp_err_t err;
    
    ESP_LOGI(I2C_TAG, "Initializing PN532...");
    ESP_ERROR_CHECK(pn532_new_driver_i2c(SDA_PIN, SCL_PIN, RESET_PIN, IRQ_PIN, 0, &pn532_io));

    do {
        err = pn532_init(&pn532_io);
        if (err != ESP_OK) {
            ESP_LOGW(I2C_TAG, "Failed to initialize PN532. Retrying...");
            pn532_release(&pn532_io);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    } while (err != ESP_OK);

    uint32_t version_data = 0;
    ESP_ERROR_CHECK(pn532_get_firmware_version(&pn532_io, &version_data));
    ESP_LOGI(I2C_TAG, "PN53x Found (Ver. %d.%d)", (int)(version_data >> 16) & 0xFF, (int)(version_data >> 8) & 0xFF);
}

struct nfc_resp run_nfc() {
    struct nfc_resp r = {{0}, 0}; 
    esp_err_t err;

    ESP_LOGI(TAG, "Waiting for ISO14443A Card...");

    err = pn532_read_passive_target_id(&pn532_io, PN532_BRTY_ISO14443A_106KBPS, r.uid, &r.length, NFC_TIMEOUT);

    if (ESP_OK == err) {
        ESP_LOGI(TAG, "UID Length: %d", r.length);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, r.uid, r.length, ESP_LOG_INFO);
    } else {
        ESP_LOGE(TAG, "Error reading UID: %d: %s", err, esp_err_to_name(err));
        memset(r.uid,0xFF,7);
        r.length = 0;
    }

    return r;
}

#define PI_UART_TX_PIN 17
#define PI_UART_RX_PIN 16
#define PI_UART_PORT_NUM UART_NUM_2
#define PI_UART_BAUD_RATE 9600

static const int PI_UART_RX_BUFFER_SIZE = 256; 
static const int PI_UART_TX_BUFFER_SIZE = 256; 

#define PAYLOAD_SIZE 7


void init_uart() {
    ESP_LOGI(TAG, "Initializing UART for %d-byte binary communication.", PAYLOAD_SIZE);

    ESP_ERROR_CHECK(uart_driver_install(PI_UART_PORT_NUM, PI_UART_RX_BUFFER_SIZE, PI_UART_TX_BUFFER_SIZE, 0, NULL, 0));

    uart_config_t pi_uart_cfg = {
        .baud_rate = PI_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(PI_UART_PORT_NUM, &pi_uart_cfg));

    ESP_ERROR_CHECK(uart_set_pin(PI_UART_PORT_NUM, PI_UART_TX_PIN, PI_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized. Sending %d-byte payloads.", PAYLOAD_SIZE);
}

int uart_send_data(const uint8_t* data) {
    return uart_write_bytes(PI_UART_PORT_NUM, (const char*)data, PAYLOAD_SIZE);
}

void app_main() {
    init_uart();

    vTaskDelay(pdMS_TO_TICKS(500)); 

    uint8_t *data = (uint8_t *) malloc(PI_UART_RX_BUFFER_SIZE);
    
    // uint8_t tx_payload[PAYLOAD_SIZE] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00};
    // uint32_t message_count = 0;
    // uart_send_data(tx_payload);
    // ESP_LOGI(TAG, "Sent initial packet.");

    while (1) {
        // tx_payload[7] = (uint8_t)(1);
        
        int rx_len = uart_read_bytes(PI_UART_PORT_NUM, data, PAYLOAD_SIZE, 50 / portTICK_PERIOD_MS);
        
        if (rx_len == PAYLOAD_SIZE) {
            ESP_LOGI(UART_TAG, "ACK Received from Pi (RX): Valid 8-byte packet.");
            ESP_LOG_BUFFER_HEXDUMP(TAG, data, rx_len, ESP_LOG_INFO);

            struct nfc_resp r = run_nfc();

            uart_send_data(r.uid);
            ESP_LOG_BUFFER_HEXDUMP(UART_TAG, r.uid, r.length, ESP_LOG_DEBUG);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    free(data);
}