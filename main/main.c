
/*
 * Task-Based Weather Monitoring Station
 * Platform : ESP-IDF
 * Sensors  : DHT11, LDR, Rain Sensor, BMP280
 * Display  : SSD1306 128x64 OLED
 *
 * I2C:
 *   SDA = GPIO 21
 *   SCL = GPIO 22
 *   BMP280 = 0x76
 *   OLED = 0x3C
 *
 * ADC:
 *   LDR  = ADC1_CHANNEL_6 (GPIO34)
 *   Rain = ADC1_CHANNEL_7 (GPIO35)
 *
 * DHT11:
 *   Data = GPIO4
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"

#include "dht.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "config.h"

static const char *TAG = "WEATHER";

/* =========================================================
   SHARED WEATHER DATA
   ========================================================= */

typedef struct {
    float temperature;
    float humidity;
    float lightPercent;
    float rainPercent;
    float pressureHpa;

    bool dhtValid;
    bool bmpValid;

    TickType_t timestamp;
} WeatherData_t;

static WeatherData_t g_weather = {0};

/* =========================================================
   SYNCHRONISATION
   ========================================================= */

static SemaphoreHandle_t xDataMutex;
static SemaphoreHandle_t xDataReadySem;
static SemaphoreHandle_t xI2CMutex;

/* =========================================================
   HARDWARE HANDLES
   ========================================================= */

static i2c_master_bus_handle_t i2c_bus = NULL;
static bmp280_handle_t bmp280_dev = NULL;
static ssd1306_handle_t oled_dev = NULL;

static adc_oneshot_unit_handle_t adc_handle = NULL;

/* =========================================================
   SHARED DATA HELPERS
   ========================================================= */

static void get_snapshot(WeatherData_t *out)
{
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
        *out = g_weather;
        xSemaphoreGive(xDataMutex);
    }
}

static void publish_new_data(void)
{
    xSemaphoreGive(xDataReadySem);
}

/* =========================================================
   I2C INITIALISATION
   ========================================================= */

static void init_i2c_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT_NUM,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_cfg, &i2c_bus)
    );

    ESP_LOGI(TAG,
             "I2C initialised: SDA=%d SCL=%d",
             I2C_SDA_GPIO,
             I2C_SCL_GPIO);
}

/* =========================================================
   BMP280 INITIALISATION
   ========================================================= */

static void init_bmp280(void)
{
    bmp280_config_t cfg = I2C_BMP280_CONFIG_DEFAULT;

    /* Scanner confirmed that the device responds at 0x76. */
    cfg.i2c_address = 0x76;

    esp_err_t ret = bmp280_init(i2c_bus, &cfg, &bmp280_dev);

    if (ret != ESP_OK || bmp280_dev == NULL) {
        ESP_LOGE(TAG,
                 "BMP280 initialisation failed: %s",
                 esp_err_to_name(ret));

        bmp280_dev = NULL;
        return;
    }

    ESP_LOGI(TAG, "BMP280 initialised at address 0x76");
}

/* =========================================================
   SSD1306 OLED INITIALISATION
   ========================================================= */

static void init_ssd1306(void)
{
    ssd1306_config_t cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;

    cfg.i2c_address = SSD1306_I2C_ADDR;

    esp_err_t ret = ssd1306_init(i2c_bus, &cfg, &oled_dev);

    if (ret != ESP_OK || oled_dev == NULL) {
        ESP_LOGE(TAG,
                 "OLED initialisation failed: %s",
                 esp_err_to_name(ret));

        oled_dev = NULL;
        return;
    }

    ssd1306_clear_display(oled_dev, false);

    ESP_LOGI(TAG, "SSD1306 OLED initialised");
}

/* =========================================================
   ADC INITIALISATION
   ========================================================= */

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&unit_cfg, &adc_handle)
    );

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            LDR_ADC_CHANNEL,
            &chan_cfg
        )
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            RAIN_ADC_CHANNEL,
            &chan_cfg
        )
    );

    ESP_LOGI(TAG, "ADC initialised");
}

/* =========================================================
   DHT11 TASK: TEMPERATURE AND HUMIDITY
   ========================================================= */

static void vDhtTask(void *pvParameters)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(DHT_INTERVAL_MS);

    while (1) {
        float humidity = 0.0f;
        float temperature = 0.0f;

        esp_err_t ret = dht_read_float_data(
            DHT_SENSOR_TYPE,
            DHT_GPIO,
            &humidity,
            &temperature
        );

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            if (ret == ESP_OK) {
                g_weather.temperature = temperature;
                g_weather.humidity = humidity;
                g_weather.dhtValid = true;
            } else {
                g_weather.dhtValid = false;
            }

            g_weather.timestamp = xTaskGetTickCount();

            xSemaphoreGive(xDataMutex);
        }

        if (ret == ESP_OK) {
            publish_new_data();
        } else {
            ESP_LOGW(TAG,
                     "DHT11 read failed: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, period);
    }
}

/* =========================================================
   LDR TASK: LIGHT LEVEL
   ========================================================= */

static void vLdrTask(void *pvParameters)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LDR_INTERVAL_MS);

    while (1) {
        int raw = 0;

        esp_err_t ret = adc_oneshot_read(
            adc_handle,
            LDR_ADC_CHANNEL,
            &raw
        );

        if (ret == ESP_OK) {
            float percentage = (raw / 4095.0f) * 100.0f;

            if (xSemaphoreTake(
                    xDataMutex,
                    portMAX_DELAY) == pdTRUE) {

                g_weather.lightPercent = percentage;
                g_weather.timestamp = xTaskGetTickCount();

                xSemaphoreGive(xDataMutex);
            }

            publish_new_data();
        } else {
            ESP_LOGW(TAG, "LDR ADC read failed: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, period);
    }
}

/* =========================================================
   RAIN SENSOR TASK
   ========================================================= */

static void vRainTask(void *pvParameters)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(RAIN_INTERVAL_MS);

    while (1) {
        int raw = 0;

        esp_err_t ret = adc_oneshot_read(
            adc_handle,
            RAIN_ADC_CHANNEL,
            &raw
        );

        if (ret == ESP_OK) {
            /*
             * Assumption: lower ADC voltage means wetter.
             * Adjust the mapping if your module behaves differently.
             */
            float percentage =
                (1.0f - (raw / 4095.0f)) * 100.0f;

            if (xSemaphoreTake(
                    xDataMutex,
                    portMAX_DELAY) == pdTRUE) {

                g_weather.rainPercent = percentage;
                g_weather.timestamp = xTaskGetTickCount();

                xSemaphoreGive(xDataMutex);
            }

            publish_new_data();
        } else {
            ESP_LOGW(TAG, "Rain ADC read failed: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, period);
    }
}

/* =========================================================
   BMP280 TASK: PRESSURE AND SENSOR TEMPERATURE
   ========================================================= */

static void vBmp280Task(void *pvParameters)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BMP280_INTERVAL_MS);

    while (1) {
        float temperature = 0.0f;
        float pressure = 0.0f;
        float pressureHpa = 0.0f;
        bool valid = false;

        if (bmp280_dev != NULL) {
            if (xSemaphoreTake(
                    xI2CMutex,
                    pdMS_TO_TICKS(500)) == pdTRUE) {

                esp_err_t ret = bmp280_get_measurements(
                    bmp280_dev,
                    &temperature,
                    &pressure
                );

                xSemaphoreGive(xI2CMutex);

                if (ret == ESP_OK && pressure > 0.0f) {
                    /*
                     * esp-idf-lib BMP280 driver returns pressure
                     * in Pa. Convert Pa to hPa.
                     */
                    pressureHpa = pressure / 100.0f;
                    valid = true;

                    ESP_LOGI(TAG,
                             "BMP280: Temp=%.2f C, Pressure=%.2f hPa",
                             temperature,
                             pressureHpa);
                } else {
                    ESP_LOGW(TAG,
                             "BMP280 measurement failed: %s, pressure=%.2f",
                             esp_err_to_name(ret),
                             pressure);
                }
            } else {
                ESP_LOGW(TAG, "BMP280 could not acquire I2C mutex");
            }
        } else {
            ESP_LOGE(TAG, "BMP280 handle is NULL");
        }

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            g_weather.pressureHpa = pressureHpa;
            g_weather.bmpValid = valid;
            g_weather.timestamp = xTaskGetTickCount();

            xSemaphoreGive(xDataMutex);
        }

        publish_new_data();

        vTaskDelayUntil(&last, period);
    }
}

/* =========================================================
   OLED DISPLAY TASK
   ========================================================= */

static void vDisplayTask(void *pvParameters)
{
    while (1) {
        xSemaphoreTake(
            xDataReadySem,
            pdMS_TO_TICKS(DISPLAY_INTERVAL_MS)
        );

        WeatherData_t snap;
        get_snapshot(&snap);

        if (oled_dev != NULL) {
            if (xSemaphoreTake(
                    xI2CMutex,
                    pdMS_TO_TICKS(500)) == pdTRUE) {

                char line[32];

                ssd1306_clear_display(oled_dev, false);

                ssd1306_display_text(
                    oled_dev, 0, "WEATHER STATION", false
                );

                if (snap.dhtValid) {
                    snprintf(line, sizeof(line),
                             "Temp : %.1f C",
                             snap.temperature);

                    ssd1306_display_text(
                        oled_dev, 1, line, false
                    );

                    snprintf(line, sizeof(line),
                             "Hum  : %.1f %%", snap.humidity);

                    ssd1306_display_text(
                        oled_dev, 2, line, false
                    );
                } else {
                    ssd1306_display_text(
                        oled_dev, 1, "Temp : --.- C", false
                    );

                    ssd1306_display_text(
                        oled_dev, 2, "Hum  : --.- %", false
                    );
                }

                snprintf(line, sizeof(line),
                         "Light: %.1f %%", snap.lightPercent);

                ssd1306_display_text(
                    oled_dev, 3, line, false
                );

                snprintf(line, sizeof(line),
                         "Rain : %.1f %%", snap.rainPercent);

                ssd1306_display_text(
                    oled_dev, 4, line, false
                );

                if (snap.bmpValid) {
                    snprintf(line, sizeof(line),
                             "Press: %.1f hPa",
                             snap.pressureHpa);

                    ssd1306_display_text(
                        oled_dev, 5, line, false
                    );
                } else {
                    ssd1306_display_text(
                        oled_dev, 5, "Press: --.- hPa", false
                    );
                }

                xSemaphoreGive(xI2CMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
    }
}

/* =========================================================
   UART LOGGING TASK
   ========================================================= */

static void vLoggingTask(void *pvParameters)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LOG_INTERVAL_MS);

    while (1) {
        WeatherData_t snap;
        get_snapshot(&snap);

        ESP_LOGI(TAG,
                 "T=%.2f C, H=%.2f %%, L=%.2f %%, "
                 "R=%.2f %%, P=%.2f hPa, BMP=%s, tick=%lu",
                 snap.temperature,
                 snap.humidity,
                 snap.lightPercent,
                 snap.rainPercent,
                 snap.pressureHpa,
                 snap.bmpValid ? "OK" : "ERROR",
                 (unsigned long)snap.timestamp);

        vTaskDelayUntil(&last, period);
    }
}

/* =========================================================
   APPLICATION ENTRY POINT
   ========================================================= */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Weather Monitoring Station");

    /* Create synchronisation objects first. */
    xDataMutex = xSemaphoreCreateMutex();
    xDataReadySem = xSemaphoreCreateBinary();
    xI2CMutex = xSemaphoreCreateMutex();

    configASSERT(xDataMutex != NULL);
    configASSERT(xDataReadySem != NULL);
    configASSERT(xI2CMutex != NULL);

    /* Initialise hardware. */
    init_i2c_bus();
    init_bmp280();
    init_ssd1306();
    init_adc();

    /* Start sensor and output tasks. */
    BaseType_t result;

    result = xTaskCreate(
        vDhtTask, "DHT11", 4096, NULL,
        PRIO_SENSOR, NULL
    );
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vLdrTask, "LDR", 2048, NULL,
        PRIO_SENSOR, NULL
    );
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vRainTask, "RAIN", 2048, NULL,
        PRIO_SENSOR, NULL
    );
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vBmp280Task, "BMP280", 3072, NULL,
        PRIO_SENSOR, NULL
    );
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vDisplayTask, "DISPLAY", 4096, NULL,
        PRIO_DISPLAY, NULL
    );
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vLoggingTask, "LOGGING", 3072, NULL,
        PRIO_LOGGING, NULL
    );
    configASSERT(result == pdPASS);

    ESP_LOGI(TAG, "All weather station tasks started");
}
