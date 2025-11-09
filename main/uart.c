#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define UART_PORT_NUM UART_NUM_2
#define UART_BAUD_RATE 9600
static const int UART_RX_BUFFER_SIZE = 1024;
static const int UART_TX_BUFFER_SIZE = 1024;

static const char *TAG = "BNB_UART";

void init_uart() {
    ESP_LOGI(TAG, "Initializing UART on TX:%d / RX:%d at %d baud", UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE);

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUFFER_SIZE, UART_TX_BUFFER_SIZE, 0, NULL, 0));

    uart_config_t pi_uart_cfg = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &pi_uart_cfg));

    // 3. Configure UART pins
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

int uart_send_data(const char* data) {
    const int len = strlen(data);
    return uart_write_bytes(UART_PORT_NUM, data, len);
}


void app_main() {
    init_uart();

    vTaskDelay(pdMS_TO_TICKS(500)); 

    ESP_LOGI(TAG, "UART initialized. Starting communication loop.");

    uint8_t *data = (uint8_t *) malloc(UART_RX_BUFFER_SIZE + 1);
    
    uart_send_data("ESP32 Ready. Sending heartbeats...\n");

    while (1) {
        char heartbeat[64];
        static int message_count = 0;
        snprintf(heartbeat, sizeof(heartbeat), "ESP32 Heartbeat #%d\n", ++message_count);
        uart_send_data(heartbeat);

        int rx_len = uart_read_bytes(UART_PORT_NUM, data, UART_RX_BUFFER_SIZE, 20 / portTICK_PERIOD_MS);
        
        if (rx_len > 0) {
            data[rx_len] = 0; // Null-terminate the string
            ESP_LOGI(TAG, "Received %d bytes from Pi: '%s'", rx_len, (char*)data);
            
            uart_send_data("ACK: ");
            uart_send_data((char*)data);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    free(data);
}