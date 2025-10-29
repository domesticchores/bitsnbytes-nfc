// NFC reader module for CSH Bits n' Bytes
// based on example code from https://github.com/Garag/esp-idf-pn532/tree/main


#include <stdio.h>
#include <stdlib.h>
#include <esp_log.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>

#include "ext_vfs.h"
#include "ioctl/esp_spi_ioctl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "pn532_driver_i2c.h"
#include "pn532_driver_hsu.h"
#include "pn532_driver_spi.h"
#include "pn532.h"

#include "driver/i2c.h"
#include "esp_log.h"

// we are NOT using IRQ for this module. ensure it is not set to interrupt in the sdkconfig.
#define SCL_PIN    (22)
#define SDA_PIN    (21)
#define RESET_PIN  (-1)
#define IRQ_PIN    (-1)

#define SPI_DEVICE_CLOCK        (1000000)
#define SPI_DEVICE_CS_PIN       (5)
#define SPI_DEVICE_SCLK_PIN     (18)
#define SPI_DEVICE_MOSI_PIN     (23)
#define SPI_DEVICE_MISO_PIN     (19)

#define SPI_RECV_BUF_SIZE       (32)

#define SPI_DEVICE "/dev/spi/2"


static const char *TAG = "bnb_nfc";

void run_nfc(pn532_io_t pn532_io,esp_err_t err) {
    uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
    uint8_t uid_length;                     // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
    int repeater = 0;

    if (repeater > 0) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, uid, uid_length, ESP_LOG_INFO);
        repeater--;
    }
    ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");

    // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
    err = pn532_read_passive_target_id(&pn532_io, PN532_BRTY_ISO14443A_106KBPS, uid, &uid_length, 1000);

    if (ESP_OK == err)
    {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, uid, uid_length, ESP_LOG_INFO);
        repeater = 10;

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } else {
        ESP_LOGI(TAG,"NULL");
    }
}

void app_main()
{
    pn532_io_t pn532_io;
    esp_err_t err;

    ESP_LOGI("INIT", "Loading bitsnbytes NFC for PN532");

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // load in I2C mode
    ESP_LOGI(TAG, "init PN532 in I2C mode");
    ESP_ERROR_CHECK(pn532_new_driver_i2c(SDA_PIN, SCL_PIN, RESET_PIN, IRQ_PIN, 0, &pn532_io));

    // ensure its loaded before doing anything
    do {
        err = pn532_init(&pn532_io);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to initialize PN532");
            pn532_release(&pn532_io);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } while(err != ESP_OK);

    ESP_LOGI(TAG, "get firmware version");
    uint32_t version_data = 0;
    do {
        err = pn532_get_firmware_version(&pn532_io, &version_data);
        if (ESP_OK != err) {
            ESP_LOGI(TAG, "Didn't find PN53x board");
            pn532_reset(&pn532_io);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } while (ESP_OK != err);

    // Log firmware infos
    ESP_LOGI(TAG, "Found chip PN5%x", (unsigned int)(version_data >> 24) & 0xFF);
    ESP_LOGI(TAG, "Firmware ver. %d.%d", (int)(version_data >> 16) & 0xFF, (int)(version_data >> 8) & 0xFF);

    // init SPI
       int fd;
    int ret;
    spi_cfg_t cfg;
    const char device[] = SPI_DEVICE;
    __attribute__((aligned(4))) char recv_buffer[SPI_RECV_BUF_SIZE];

    ext_vfs_init();

    fd = open(device, O_RDWR);
    if (fd < 0) {
        printf("Opening device %s for writing failed, errno=%d.\n", device, errno);
        return;
    } else {
        printf("Opening device %s for writing OK, fd=%d.\n", device, fd);
    }

    cfg.flags = SPI_MODE_0;
    cfg.cs_pin = SPI_DEVICE_CS_PIN;
    cfg.sclk_pin = SPI_DEVICE_SCLK_PIN;
    cfg.mosi_pin = SPI_DEVICE_MOSI_PIN;
    cfg.miso_pin = SPI_DEVICE_MISO_PIN;
    cfg.master.clock = SPI_DEVICE_CLOCK;
    ret = ioctl(fd, SPIIOCSCFG, &cfg);
    if (ret < 0) {
        printf("Configure SPI failed, errno=%d.\n", errno);
        return;
    }
    while (1)
    {

        spi_ex_msg_t msg;

        memset(recv_buffer, 0, SPI_RECV_BUF_SIZE);
        msg.rx_buffer = recv_buffer;
        msg.tx_buffer = NULL;
        msg.size = SPI_RECV_BUF_SIZE;
        ret = ioctl(fd, SPIIOCEXCHANGE, &msg);
        if (ret < 0) {
            ESP_LOGI(TAG,"Receive total %d bytes from device failed, errno=%d", SPI_RECV_BUF_SIZE, errno);
            return;
        } else {
            if (msg.size) {
                ESP_LOGI(TAG,"Receive total %d bytes from device: %s\n", (int)msg.size, recv_buffer);
            }
            // connect to nfc
            run_nfc(pn532_io, err);
        }
    }
}
