
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000

static const char *TAG = "OLED_TEST";

void app_main(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(
        I2C_MASTER_NUM, conf.mode, 0, 0, 0
    ));

    ESP_LOGI(TAG, "Scanning I2C bus...");

    bool found = false;

    for (uint8_t address = 0x08; address < 0x78; address++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(
            I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100)
        );

        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at address 0x%02X", address);
            found = true;
        }
    }

    if (!found) {
        ESP_LOGE(TAG, "No I2C device found. Check wiring and interface.");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
