/**********************************************************************
 * Task-Based Weather Monitoring Station – ESP-IDF (VS Code)
 * Sensors : DHT11 | LDR | Rain sensor | BMP280
 * Output  : SSD1306 OLED + UART log
 *
 * Synchronization:
 *   xDataMutex     – protects g_weather (shared memory)
 *   xDataReadySem  – binary semaphore: “new data available”
 *   xI2CMutex      – serialises the shared I2C bus
 *
 * Note: SSD1306 driver (v1.2.x) uses 4-argument ssd1306_display_text.
 **********************************************************************/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "dht.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "config.h"

static const char *TAG = "WEATHER";

// ==================== SHARED MEMORY ====================
typedef struct {
    float temperature;      // °C
    float humidity;         // %
    float lightPercent;     // 0–100
    float rainPercent;      // 0–100 (100 = wet)
    float pressureHpa;      // hPa
    bool  dhtValid;
    bool  bmpValid;
    TickType_t timestamp;
} WeatherData_t;

static WeatherData_t g_weather = {0};

// ==================== SYNCHRONIZATION ====================
static SemaphoreHandle_t xDataMutex    = NULL;
static SemaphoreHandle_t xDataReadySem = NULL;
static SemaphoreHandle_t xI2CMutex     = NULL;

// ==================== I2C BUS & DEVICES ====================
static i2c_master_bus_handle_t i2c_bus;
static bmp280_handle_t         bmp280_dev;
static ssd1306_handle_t        oled_dev;

// ==================== ADC HANDLE ====================
static adc_oneshot_unit_handle_t adc_handle;

// ==================== HELPERS ====================

static void get_snapshot(WeatherData_t *out) {
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
        *out = g_weather;
        xSemaphoreGive(xDataMutex);
    }
}

static inline void publish_new_data(void) {
    xSemaphoreGive(xDataReadySem);
}

// ==================== INITIALISATION ROUTINES ====================

static void init_i2c_bus(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
    ESP_LOGI(TAG, "I2C bus initialised (SDA=%d, SCL=%d)", I2C_SDA_GPIO, I2C_SCL_GPIO);
}

static void init_bmp280(void) {
    bmp280_config_t cfg = I2C_BMP280_CONFIG_DEFAULT;
    cfg.i2c_address = BMP280_I2C_ADDR;
    esp_err_t ret = bmp280_init(i2c_bus, &cfg, &bmp280_dev);
    if (ret != ESP_OK || bmp280_dev == NULL) {
        ESP_LOGE(TAG, "BMP280 init failed (%s) – pressure disabled", esp_err_to_name(ret));
        bmp280_dev = NULL;
    } else {
        ESP_LOGI(TAG, "BMP280 ready");
    }
}

static void init_ssd1306(void) {
    ssd1306_config_t cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    cfg.i2c_address = SSD1306_I2C_ADDR;
    esp_err_t ret = ssd1306_init(i2c_bus, &cfg, &oled_dev);
    if (ret != ESP_OK || oled_dev == NULL) {
        ESP_LOGE(TAG, "SSD1306 init failed (%s)", esp_err_to_name(ret));
        oled_dev = NULL;
    } else {
        ssd1306_clear_display(oled_dev, false);
        ESP_LOGI(TAG, "SSD1306 ready");
    }
}

static void init_adc(void) {
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,      // 0–3.3 V range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL,  &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, RAIN_ADC_CHANNEL, &chan_cfg));
    ESP_LOGI(TAG, "ADC oneshot ready");
}

// ==================== SENSOR TASKS ====================

// ---------- DHT11 : Temperature + Humidity ----------
static void vDhtTask(void *pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(DHT_INTERVAL_MS);

    while (1) {
        float hum = 0, temp = 0;
        esp_err_t ret = dht_read_float_data(DHT_SENSOR_TYPE, DHT_GPIO, &hum, &temp);

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            if (ret == ESP_OK) {
                g_weather.temperature = temp;
                g_weather.humidity    = hum;
                g_weather.dhtValid    = true;
            } else {
                g_weather.dhtValid    = false;
            }
            g_weather.timestamp = xTaskGetTickCount();
            xSemaphoreGive(xDataMutex);
        }

        if (ret == ESP_OK) publish_new_data();
        else               ESP_LOGW(TAG, "DHT11 read failed (%s)", esp_err_to_name(ret));

        vTaskDelayUntil(&last, period);
    }
}

// ---------- LDR : Light intensity ----------
static void vLdrTask(void *pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LDR_INTERVAL_MS);

    while (1) {
        int raw = 0;
        adc_oneshot_read(adc_handle, LDR_ADC_CHANNEL, &raw);
        float pct = (raw / 4095.0f) * 100.0f;

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            g_weather.lightPercent = pct;
            g_weather.timestamp    = xTaskGetTickCount();
            xSemaphoreGive(xDataMutex);
        }
        publish_new_data();
        vTaskDelayUntil(&last, period);
    }
}

// ---------- Rain sensor ----------
static void vRainTask(void *pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(RAIN_INTERVAL_MS);

    while (1) {
        int raw = 0;
        adc_oneshot_read(adc_handle, RAIN_ADC_CHANNEL, &raw);
        // YL-83: lower voltage = wetter → invert so 100 % = fully wet
        float pct = (1.0f - (raw / 4095.0f)) * 100.0f;

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            g_weather.rainPercent = pct;
            g_weather.timestamp   = xTaskGetTickCount();
            xSemaphoreGive(xDataMutex);
        }
        publish_new_data();
        vTaskDelayUntil(&last, period);
    }
}

// ---------- BMP280 : Pressure ----------
static void vBmp280Task(void *pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BMP280_INTERVAL_MS);

    while (1) {
        float hpa = 0.0f;
        bool valid = false;

        if (bmp280_dev != NULL &&
            xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(200)) == pdTRUE) {

            float temp, press;
            esp_err_t ret = bmp280_get_measurements(bmp280_dev, &temp, &press);
            if (ret == ESP_OK && press > 0) {
                hpa   = press / 100.0f;       // Pa → hPa
                valid = true;
            }
            xSemaphoreGive(xI2CMutex);
        }

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            g_weather.pressureHpa = hpa;
            g_weather.bmpValid    = valid;
            g_weather.timestamp   = xTaskGetTickCount();
            xSemaphoreGive(xDataMutex);
        }
        if (valid) publish_new_data();

        vTaskDelayUntil(&last, period);
    }
}

// ==================== DISPLAY TASK ====================
static void vDisplayTask(void *pv) {
    while (1) {
        // Wait for fresh data (max 500 ms)
        xSemaphoreTake(xDataReadySem, pdMS_TO_TICKS(500));

        WeatherData_t snap;
        get_snapshot(&snap);

        if (oled_dev != NULL &&
            xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(200)) == pdTRUE) {

            char line[32];
            ssd1306_clear_display(oled_dev, false);

            // ---- CORRECTED: 4-argument ssd1306_display_text calls ----
            ssd1306_display_text(oled_dev, 0, "WEATHER STATION", false);

            if (snap.dhtValid) {
                snprintf(line, sizeof(line), "Temp : %.1f C", snap.temperature);
                ssd1306_display_text(oled_dev, 1, line, false);

                snprintf(line, sizeof(line), "Hum  : %.1f %%", snap.humidity);
                ssd1306_display_text(oled_dev, 2, line, false);
            } else {
                ssd1306_display_text(oled_dev, 1, "Temp : --.- C", false);
                ssd1306_display_text(oled_dev, 2, "Hum  : --.- %", false);
            }

            snprintf(line, sizeof(line), "Light: %.1f %%", snap.lightPercent);
            ssd1306_display_text(oled_dev, 3, line, false);

            snprintf(line, sizeof(line), "Rain : %.1f %%", snap.rainPercent);
            ssd1306_display_text(oled_dev, 4, line, false);

            if (snap.bmpValid) {
                snprintf(line, sizeof(line), "Press: %.1f hPa", snap.pressureHpa);
                ssd1306_display_text(oled_dev, 5, line, false);
            } else {
                ssd1306_display_text(oled_dev, 5, "Press: --.- hPa", false);
            }

            // No ssd1306_update_display() needed for this driver.
            xSemaphoreGive(xI2CMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
    }
}

// ==================== LOGGING TASK ====================
static void vLoggingTask(void *pv) {
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LOG_INTERVAL_MS);

    while (1) {
        WeatherData_t snap;
        get_snapshot(&snap);

        ESP_LOGI(TAG,
                 "T=%.2f H=%.2f L=%.2f R=%.2f P=%.2f t=%lu",
                 snap.temperature, snap.humidity,
                 snap.lightPercent, snap.rainPercent,
                 snap.pressureHpa, (unsigned long)snap.timestamp);

        // Optional: append to SPIFFS / SD card here.

        vTaskDelayUntil(&last, period);
    }
}

// ==================== APP MAIN ====================
void app_main(void) {
    ESP_LOGI(TAG, "Booting Weather Station (ESP-IDF)");

    // ---- Create synchronization objects ----
    xDataMutex    = xSemaphoreCreateMutex();
    xDataReadySem = xSemaphoreCreateBinary();
    xI2CMutex     = xSemaphoreCreateMutex();
    configASSERT(xDataMutex && xDataReadySem && xI2CMutex);

    // ---- Hardware init ----
    init_i2c_bus();
    init_bmp280();
    init_ssd1306();
    init_adc();
    // dht_init() is NOT required for esp-idf-lib/dht – removed.

    // ---- Create tasks ----
    xTaskCreate(vDhtTask,      "DHT",    4096, NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(vLdrTask,      "LDR",    2048, NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(vRainTask,     "RAIN",   2048, NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(vBmp280Task,   "BMP280", 3072, NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(vDisplayTask,  "DISP",   4096, NULL, PRIO_DISPLAY, NULL);
    xTaskCreate(vLoggingTask,  "LOG",    3072, NULL, PRIO_LOGGING, NULL);

    ESP_LOGI(TAG, "All tasks created — scheduler running");
}