#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define PI_UART_TX_PIN 17
#define PI_UART_RX_PIN 16
#define PI_UART_PORT_NUM UART_NUM_2
#define PI_UART_BAUD_RATE 9600

static const int PI_UART_RX_BUFFER_SIZE = 256; 
static const int PI_UART_TX_BUFFER_SIZE = 256; 

#define PAYLOAD_SIZE 8 
static const char *TAG = "UART_ESP32_BIN";


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
    
    uint8_t tx_payload[PAYLOAD_SIZE] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x00, 0x00, 0x00};
    // uint32_t message_count = 0;
    // uart_send_data(tx_payload);
    // ESP_LOGI(TAG, "Sent initial packet.");

    while (1) {
        tx_payload[7] = (uint8_t)(1);
        

        int rx_len = uart_read_bytes(PI_UART_PORT_NUM, data, PAYLOAD_SIZE, 50 / portTICK_PERIOD_MS);
        
        if (rx_len == PAYLOAD_SIZE) {
            ESP_LOGI(TAG, "ACK Received from Pi (RX): Valid 8-byte packet.");
            ESP_LOG_BUFFER_HEXDUMP(TAG, data, rx_len, ESP_LOG_INFO);

            uart_send_data(tx_payload);
            ESP_LOG_BUFFER_HEXDUMP(TAG, tx_payload, PAYLOAD_SIZE, ESP_LOG_DEBUG);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    free(data);
}