
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_PORT       I2C_NUM_0
#define SDA_GPIO       21
#define SCL_GPIO       22

static const char *TAG = "BMP280_TEST";
static i2c_master_bus_handle_t bus_handle;

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

    ESP_LOGI(TAG, "Scanning I2C bus...");

    uint8_t address = 0;
    esp_err_t err;

    if (i2c_master_probe(bus_handle, 0x76, 100) == ESP_OK) {
        address = 0x76;
    } else if (i2c_master_probe(bus_handle, 0x77, 100) == ESP_OK) {
        address = 0x77;
    }

    if (address == 0) {
        ESP_LOGE(TAG, "BMP280 NOT FOUND at 0x76 or 0x77");
        ESP_LOGE(TAG, "Check power, GND, SDA=21, SCL=22 and wiring.");
        return;
    }

    ESP_LOGI(TAG, "I2C device found at address 0x%02X", address);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle)
    );

    // BMP280 register 0xD0 contains the chip ID.
    uint8_t reg = 0xD0;
    uint8_t chip_id = 0;

    err = i2c_master_transmit_receive(
        dev_handle, &reg, 1, &chip_id, 1, 1000
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Chip ID read failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Chip ID = 0x%02X", chip_id);

    if (chip_id != 0x58) {
        ESP_LOGW(TAG, "Expected BMP280 ID 0x58. This may be another sensor.");
        if (chip_id == 0x60) {
            ESP_LOGW(TAG, "ID 0x60 usually indicates a BME280.");
        }
    } else {
        ESP_LOGI(TAG, "BMP280 chip ID verified.");
    }

    // Configure normal mode, temperature and pressure oversampling x1.
    uint8_t reset_cmd[] = {0xE0, 0xB6};
    err = i2c_master_transmit(dev_handle, reset_cmd, 2, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Reset command failed: %s", esp_err_to_name(err));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t ctrl_cmd[] = {0xF4, 0x27};
    uint8_t config_cmd[] = {0xF5, 0xA0};

    err = i2c_master_transmit(dev_handle, ctrl_cmd, 2, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Control configuration failed: %s",
                 esp_err_to_name(err));
        return;
    }

    err = i2c_master_transmit(dev_handle, config_cmd, 2, 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor configuration failed: %s",
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Reading raw sensor data...");

    while (1) {
        uint8_t data_reg = 0xF7;
        uint8_t data[6];

        err = i2c_master_transmit_receive(
            dev_handle, &data_reg, 1, data, 6, 1000
        );

        if (err == ESP_OK) {
            uint32_t pressure_raw =
                ((uint32_t)data[0] << 12) |
                ((uint32_t)data[1] << 4) |
                (data[2] >> 4);

            uint32_t temperature_raw =
                ((uint32_t)data[3] << 12) |
                ((uint32_t)data[4] << 4) |
                (data[5] >> 4);

            ESP_LOGI(TAG,
                     "Raw pressure=%lu, raw temperature=%lu",
                     (unsigned long)pressure_raw,
                     (unsigned long)temperature_raw);
        } else {
            ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
