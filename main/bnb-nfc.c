#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_slave.h" 
#include "driver/i2c.h"       

#define SCL_PIN             (22)
#define SDA_PIN             (21)
#define RESET_PIN           (-1)
#define IRQ_PIN             (-1)

#define RCV_HOST            SPI2_HOST // HSPI
#define GPIO_MOSI           23
#define GPIO_MISO           19
#define GPIO_SCLK           18
#define GPIO_CS             5

#define MAX_UID_LEN         7
#define TRANSACTION_LEN     8         

#include "sdkconfig.h"
#include "pn532_driver_i2c.h"
#include "pn532.h"

static const char *TAG = "NFC_APP";
static const char *SPI_TAG = "SPI_SLAVE";
static const char *I2C_TAG = "I2C_NFC";

struct nfc_resp {
    uint8_t uid[MAX_UID_LEN];
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

    err = pn532_read_passive_target_id(&pn532_io, PN532_BRTY_ISO14443A_106KBPS, r.uid, &r.length, 100);

    if (ESP_OK == err) {
        ESP_LOGI(TAG, "UID Length: %d", r.length);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, r.uid, r.length, ESP_LOG_INFO);
    } else {
        ESP_LOGE(TAG, "Error reading UID: %s", esp_err_to_name(err));
        r.length = 0;
    }

    return r;
}

void spi_slave_init() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
    };

    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_DISABLED));
    ESP_LOGI(SPI_TAG, "SPI initialized");
}

void app_main() {
    esp_err_t err;

    pn532_i2c_init();

    spi_slave_init();

    // aligned is good practice apparently
    __attribute__((aligned(4))) static char tx_buffer[TRANSACTION_LEN]; 
    __attribute__((aligned(4))) static char rx_buffer[TRANSACTION_LEN]; 
    
    spi_slave_transaction_t t = {
        .length = TRANSACTION_LEN * 8,
        .tx_buffer = tx_buffer,       
        .rx_buffer = rx_buffer,       
    };

    while (1) {
        memset(tx_buffer, 0, TRANSACTION_LEN);
        memset(rx_buffer, 0, TRANSACTION_LEN);
        
        tx_buffer[0] = 0x00; 
        
        ESP_LOGI(TAG, "Waiting for NFC trigger...");
        
        err = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);

        if (err != ESP_OK) {
            ESP_LOGE(SPI_TAG, "SPI Signal Error: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (rx_buffer[0] == 0x01) {
            ESP_LOGI(SPI_TAG, "Trigger received");

            struct nfc_resp response = run_nfc();

            tx_buffer[0] = (response.length > 0) ? response.length : 0xFF; 
            
            if (response.length > 0) {
                memcpy(&tx_buffer[1], response.uid, response.length);
            }
            
            ESP_LOGI(SPI_TAG, "NFC done!");
            
            err = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
            
            if (err == ESP_OK) {
                ESP_LOGI(SPI_TAG, "Response sent successfully: %d).", tx_buffer[0]);
                ESP_LOG_BUFFER_HEX_LEVEL(SPI_TAG, tx_buffer, TRANSACTION_LEN, ESP_LOG_INFO);
            } else {
                 ESP_LOGE(SPI_TAG, "Failed to send response: %s", esp_err_to_name(err));
            }
            
        } else {
            ESP_LOGI(SPI_TAG, "Non-trigger command (0x%02X) received. Waiting.", (uint8_t)rx_buffer[0]);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}