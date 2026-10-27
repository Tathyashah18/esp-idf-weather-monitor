#pragma once

// ---- Pin map ----
#define DHT_GPIO            4
#define DHT_SENSOR_TYPE     DHT_TYPE_DHT11

#define LDR_ADC_CHANNEL     ADC_CHANNEL_6   // GPIO34
#define RAIN_ADC_CHANNEL    ADC_CHANNEL_7   // GPIO35

#define I2C_SDA_GPIO        21
#define I2C_SCL_GPIO        22
#define I2C_PORT_NUM        0
#define I2C_FREQ_HZ         400000

#define BMP280_I2C_ADDR     0x77            // try 0x77 if needed
#define SSD1306_I2C_ADDR    0x3C

// ---- Sampling intervals (ms) ----
#define DHT_INTERVAL_MS      2500
#define LDR_INTERVAL_MS      1500
#define RAIN_INTERVAL_MS     1500
#define BMP280_INTERVAL_MS   3000
#define DISPLAY_INTERVAL_MS  1000
#define LOG_INTERVAL_MS      5000

// ---- Task priorities ----
#define PRIO_SENSOR          5
#define PRIO_DISPLAY         3
#define PRIO_LOGGING         2