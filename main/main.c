
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define SDA_GPIO 21
#define SCL_GPIO 22

void app_main(void)
{
    i2c_master_bus_handle_t bus;

    i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&config, &bus));

    printf("\nScanning I2C addresses...\n");

    int found = 0;

    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        esp_err_t err = i2c_master_probe(bus, addr, 100);

        if (err == ESP_OK) {
            printf("Device found at 0x%02X\n", addr);
            found++;
        }
    }

    if (found == 0) {
        printf("No I2C devices found.\n");
    } else {
        printf("Total devices found: %d\n", found);
    }

    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}
