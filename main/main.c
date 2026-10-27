
/**********************************************************************
 * Task-Based Weather Monitoring Station - ESP-IDF 6.1
 * Sensors : DHT11 | LDR | Rain sensor | BMP280/HW-611
 * Display : SSD1306-compatible OLED
 *
 * Features:
 * - Separate FreeRTOS tasks for sensors, display and logging
 * - Mutex protection for shared weather data
 * - Shared I2C bus for BMP280 and OLED
 * - Improved BMP280 initialisation and measurement diagnostics
 **********************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

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

/* ==================== WEATHER DATA ==================== */

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

/* ==================== SYNCHRONISATION ==================== */

static SemaphoreHandle_t xDataMutex = NULL;
static SemaphoreHandle_t xDataReadySem = NULL;
static SemaphoreHandle_t xI2CMutex = NULL;

/* ==================== HARDWARE HANDLES ==================== */

static i2c_master_bus_handle_t i2c_bus = NULL;
static bmp280_handle_t bmp280_dev = NULL;
static ssd1306_handle_t oled_dev = NULL;

static adc_oneshot_unit_handle_t adc_handle = NULL;

/* ==================== SHARED DATA HELPERS ==================== */

static void get_snapshot(WeatherData_t *out)
{
    if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
        *out = g_weather;
        xSemaphoreGive(xDataMutex);
    }
}

static void publish_new_data(void)
{
    if (xDataReadySem != NULL) {
        xSemaphoreGive(xDataReadySem);
    }
}

/* ==================== I2C INITIALISATION ==================== */

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

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c_bus);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialisation failed: %s",
                 esp_err_to_name(ret));
        abort();
    }

    ESP_LOGI(TAG,
             "I2C bus ready: SDA=GPIO%d, SCL=GPIO%d, port=%d",
             I2C_SDA_GPIO, I2C_SCL_GPIO, I2C_PORT_NUM);
}

/* ==================== BMP280 INITIALISATION ==================== */

static void init_bmp280(void)
{
    bmp280_config_t cfg = I2C_BMP280_CONFIG_DEFAULT;
    cfg.i2c_address = BMP280_I2C_ADDR;

    bmp280_dev = NULL;

    esp_err_t ret = bmp280_init(i2c_bus, &cfg, &bmp280_dev);

    if (ret != ESP_OK || bmp280_dev == NULL) {
        ESP_LOGE(TAG,
                 "BMP280 initialisation failed: %s, address=0x%02X",
                 esp_err_to_name(ret),
                 BMP280_I2C_ADDR);

        bmp280_dev = NULL;
        return;
    }

    ESP_LOGI(TAG,
             "BMP280 initialised successfully at address 0x%02X",
             BMP280_I2C_ADDR);

    /*
     * The driver should configure the sensor and read its
     * calibration data during initialisation.
     */
}

/* ==================== OLED INITIALISATION ==================== */

static void init_ssd1306(void)
{
    ssd1306_config_t cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    cfg.i2c_address = SSD1306_I2C_ADDR;

    oled_dev = NULL;

    esp_err_t ret = ssd1306_init(i2c_bus, &cfg, &oled_dev);

    if (ret != ESP_OK || oled_dev == NULL) {
        ESP_LOGE(TAG, "OLED initialisation failed: %s",
                 esp_err_to_name(ret));

        oled_dev = NULL;
        return;
    }

    ssd1306_clear_display(oled_dev, false);

    ESP_LOGI(TAG,
             "OLED initialised at address 0x%02X",
             SSD1306_I2C_ADDR);
}

/* ==================== ADC INITIALISATION ==================== */

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
            adc_handle, LDR_ADC_CHANNEL, &chan_cfg
        )
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle, RAIN_ADC_CHANNEL, &chan_cfg
        )
    );

    ESP_LOGI(TAG, "ADC initialised");
}

/* ==================== DHT11 TASK ==================== */

static void vDhtTask(void *pv)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(DHT_INTERVAL_MS);

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

                ESP_LOGW(TAG, "DHT11 read failed: %s",
                         esp_err_to_name(ret));
            }

            g_weather.timestamp = xTaskGetTickCount();
            xSemaphoreGive(xDataMutex);
        }

        if (ret == ESP_OK) {
            publish_new_data();
        }

        vTaskDelayUntil(&last, period);
    }
}

/* ==================== LDR TASK ==================== */

static void vLdrTask(void *pv)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(LDR_INTERVAL_MS);

    while (1) {
        int raw = 0;

        esp_err_t ret = adc_oneshot_read(
            adc_handle, LDR_ADC_CHANNEL, &raw
        );

        if (ret == ESP_OK) {
            float percent = (raw / 4095.0f) * 100.0f;

            if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
                g_weather.lightPercent = percent;
                g_weather.timestamp = xTaskGetTickCount();
                xSemaphoreGive(xDataMutex);
            }

            publish_new_data();
        } else {
            ESP_LOGE(TAG, "LDR ADC read failed: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, period);
    }
}

/* ==================== RAIN SENSOR TASK ==================== */

static void vRainTask(void *pv)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(RAIN_INTERVAL_MS);

    while (1) {
        int raw = 0;

        esp_err_t ret = adc_oneshot_read(
            adc_handle, RAIN_ADC_CHANNEL, &raw
        );

        if (ret == ESP_OK) {
            /* Lower analogue voltage indicates wetter surface. */
            float percent =
                (1.0f - (raw / 4095.0f)) * 100.0f;

            if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
                g_weather.rainPercent = percent;
                g_weather.timestamp = xTaskGetTickCount();
                xSemaphoreGive(xDataMutex);
            }

            publish_new_data();
        } else {
            ESP_LOGE(TAG, "Rain ADC read failed: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, period);
    }
}

/* ==================== BMP280 PRESSURE TASK ==================== */

static void vBmp280Task(void *pv)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(BMP280_INTERVAL_MS);

    while (1) {
        float pressureHpa = 0.0f;
        bool valid = false;

        if (bmp280_dev != NULL) {
            if (xSemaphoreTake(
                    xI2CMutex, pdMS_TO_TICKS(500)) == pdTRUE) {

                float temperature = 0.0f;
                float pressurePa = 0.0f;

                esp_err_t ret = bmp280_get_measurements(
                    bmp280_dev,
                    &temperature,
                    &pressurePa
                );

                if (ret == ESP_OK && isfinite(pressurePa)
                    && pressurePa > 0.0f) {

                    pressureHpa = pressurePa / 100.0f;
                    valid = true;

                    ESP_LOGI(TAG,
                             "BMP280: Temp=%.2f C, Pressure=%.2f Pa",
                             temperature, pressurePa);
                } else {
                    ESP_LOGE(TAG,
                             "BMP280 measurement failed: %s, raw pressure=%.2f",
                             esp_err_to_name(ret), pressurePa);
                }

                xSemaphoreGive(xI2CMutex);
            } else {
                ESP_LOGW(TAG, "Timed out waiting for I2C mutex");
            }
        } else {
            ESP_LOGW(TAG, "BMP280 unavailable; check initialisation");
        }

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            g_weather.pressureHpa = pressureHpa;
            g_weather.bmpValid = valid;
            g_weather.timestamp = xTaskGetTickCount();
            xSemaphoreGive(xDataMutex);
        }

        if (valid) {
            publish_new_data();
        }

        vTaskDelayUntil(&last, period);
    }
}

/* ==================== OLED DISPLAY TASK ==================== */

static void vDisplayTask(void *pv)
{
    while (1) {
        xSemaphoreTake(
            xDataReadySem, pdMS_TO_TICKS(500)
        );

        WeatherData_t snap;
        get_snapshot(&snap);

        if (oled_dev != NULL &&
            xSemaphoreTake(
                xI2CMutex, pdMS_TO_TICKS(500)) == pdTRUE) {

            char line[32];

            ssd1306_clear_display(oled_dev, false);

            ssd1306_display_text(
                oled_dev, 0, "WEATHER STATION", false
            );

            if (snap.dhtValid) {
                snprintf(line, sizeof(line),
                         "Temp : %.1f C", snap.temperature);
                ssd1306_display_text(oled_dev, 1, line, false);

                snprintf(line, sizeof(line),
                         "Hum  : %.1f %%", snap.humidity);
                ssd1306_display_text(oled_dev, 2, line, false);
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
            ssd1306_display_text(oled_dev, 3, line, false);

            snprintf(line, sizeof(line),
                     "Rain : %.1f %%", snap.rainPercent);
            ssd1306_display_text(oled_dev, 4, line, false);

            if (snap.bmpValid) {
                snprintf(line, sizeof(line),
                         "Press: %.1f hPa", snap.pressureHpa);
                ssd1306_display_text(oled_dev, 5, line, false);
            } else {
                ssd1306_display_text(
                    oled_dev, 5, "Press: --.- hPa", false
                );
            }

            xSemaphoreGive(xI2CMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(DISPLAY_INTERVAL_MS));
    }
}

/* ==================== UART LOGGING TASK ==================== */

static void vLoggingTask(void *pv)
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(LOG_INTERVAL_MS);

    while (1) {
        WeatherData_t snap;
        get_snapshot(&snap);

        ESP_LOGI(TAG,
                 "T=%.2f H=%.2f L=%.2f R=%.2f P=%.2f "
                 "DHT=%s BMP280=%s t=%lu",
                 snap.temperature,
                 snap.humidity,
                 snap.lightPercent,
                 snap.rainPercent,
                 snap.pressureHpa,
                 snap.dhtValid ? "OK" : "ERROR",
                 snap.bmpValid ? "OK" : "ERROR",
                 (unsigned long)snap.timestamp);

        vTaskDelayUntil(&last, period);
    }
}

/* ==================== APPLICATION ENTRY ==================== */

void app_main(void)
{
    ESP_LOGI(TAG, "Booting Weather Monitoring Station");

    xDataMutex = xSemaphoreCreateMutex();
    xDataReadySem = xSemaphoreCreateBinary();
    xI2CMutex = xSemaphoreCreateMutex();

    configASSERT(
        xDataMutex != NULL &&
        xDataReadySem != NULL &&
        xI2CMutex != NULL
    );

    init_i2c_bus();
    init_bmp280();
    init_ssd1306();
    init_adc();

    xTaskCreate(
        vDhtTask, "DHT", 4096, NULL, PRIO_SENSOR, NULL
    );

    xTaskCreate(
        vLdrTask, "LDR", 2048, NULL, PRIO_SENSOR, NULL
    );

    xTaskCreate(
        vRainTask, "RAIN", 2048, NULL, PRIO_SENSOR, NULL
    );

    xTaskCreate(
        vBmp280Task, "BMP280", 3072, NULL, PRIO_SENSOR, NULL
    );

    xTaskCreate(
        vDisplayTask, "DISPLAY", 4096, NULL, PRIO_DISPLAY, NULL
    );

    xTaskCreate(
        vLoggingTask, "LOG", 3072, NULL, PRIO_LOGGING, NULL
    );

    ESP_LOGI(TAG, "All weather-monitoring tasks started");
}
