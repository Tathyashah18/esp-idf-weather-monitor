#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdatomic.h>

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

/* ============================================================
 * CONFIGURATION
 * ============================================================ */

/*
 * 1: normal operation, protect shared data using a mutex.
 * 0: controlled demonstration mode, disable data mutex protection.
 *
 * USE_MUTEX=0 is intentionally unsafe and must only be used
 * for a controlled demonstration, not normal operation.
 */
#ifndef USE_MUTEX
#define USE_MUTEX                  1
#endif

#define ADC_SAMPLE_COUNT           8
#define ADC_SAMPLE_DELAY_MS        2
#define I2C_MUTEX_TIMEOUT_MS       1000
#define DATA_MUTEX_WARN_MS         5

#define DHT_STALE_MS               (DHT_INTERVAL_MS * 3)
#define LDR_STALE_MS               (LDR_INTERVAL_MS * 3)
#define RAIN_STALE_MS              (RAIN_INTERVAL_MS * 3)
#define BMP_STALE_MS               (BMP280_INTERVAL_MS * 3)

static const char *TAG = "WEATHER";

/* ============================================================
 * SHARED DATA
 * ============================================================ */

typedef struct {
    float temperature;
    float humidity;
    float lightPercent;
    float rainPercent;
    float pressureHpa;

    bool dhtValid;
    bool lightValid;
    bool rainValid;
    bool bmpValid;

    TickType_t dhtLastSuccess;
    TickType_t lightLastSuccess;
    TickType_t rainLastSuccess;
    TickType_t bmpLastSuccess;

    TickType_t timestamp;
} WeatherData_t;

static WeatherData_t g_weather = {0};

/*
 * Even counter = no write in progress.
 * Odd counter = a write is in progress.
 *
 * This is a diagnostic sequence counter, not a replacement
 * for the mutex. The ordinary fields in g_weather must not be
 * read concurrently with writes in production code.
 */
static atomic_uint g_dataSequence = ATOMIC_VAR_INIT(0);

/* ============================================================
 * SYNCHRONISATION OBJECTS
 * ============================================================ */

static SemaphoreHandle_t xDataMutex = NULL;
static SemaphoreHandle_t xDataReadySem = NULL;
static SemaphoreHandle_t xI2CMutex = NULL;

/* Mutex statistics */
static atomic_uint g_dataLockContentions = ATOMIC_VAR_INIT(0);
static atomic_uint g_dataLockSlowWaits = ATOMIC_VAR_INIT(0);
static atomic_uint g_dataLockMaxWaitTicks = ATOMIC_VAR_INIT(0);

/* ============================================================
 * HARDWARE HANDLES
 * ============================================================ */

static i2c_master_bus_handle_t i2c_bus = NULL;
static bmp280_handle_t bmp280_dev = NULL;
static ssd1306_handle_t oled_dev = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;

/* ============================================================
 * DATA MUTEX AND DIAGNOSTICS
 * ============================================================ */

static void update_max_wait(TickType_t elapsed)
{
    unsigned int previous = atomic_load(
        &g_dataLockMaxWaitTicks
    );

    while (elapsed > previous) {
        if (atomic_compare_exchange_weak(
                &g_dataLockMaxWaitTicks,
                &previous,
                (unsigned int)elapsed)) {
            break;
        }
    }
}

static void data_lock(const char *taskName)
{
#if USE_MUTEX
    TickType_t start = xTaskGetTickCount();

    /*
     * This is a blocking wait. For a simple contention metric,
     * try first, then block if the mutex is occupied.
     */
    if (xSemaphoreTake(xDataMutex, 0) != pdTRUE) {
        atomic_fetch_add(&g_dataLockContentions, 1);

        xSemaphoreTake(xDataMutex, portMAX_DELAY);
    }

    TickType_t elapsed = xTaskGetTickCount() - start;

    update_max_wait(elapsed);

    if (elapsed >= pdMS_TO_TICKS(DATA_MUTEX_WARN_MS)) {
        atomic_fetch_add(&g_dataLockSlowWaits, 1);

        ESP_LOGW(TAG,
                 "%s waited %lu ticks for data mutex",
                 taskName,
                 (unsigned long)elapsed);
    }
#else
    (void)taskName;
#endif
}

static void data_unlock(void)
{
#if USE_MUTEX
    xSemaphoreGive(xDataMutex);
#endif
}

/* ============================================================
 * SEQUENCE COUNTER
 * ============================================================ */

static void begin_data_write(void)
{
    /*
     * Writers must be serialised when USE_MUTEX=1.
     * The counter becomes odd while a write is in progress.
     */
    atomic_fetch_add_explicit(
        &g_dataSequence,
        1,
        memory_order_relaxed
    );
}

static void end_data_write(void)
{
    atomic_fetch_add_explicit(
        &g_dataSequence,
        1,
        memory_order_release
    );
}

/*
 * Call while holding the data mutex in normal operation.
 * Each complete logical update should have one begin/end pair.
 */
static void publish_new_data(void)
{
    if (xDataReadySem != NULL) {
        /*
         * Binary semaphore notifications can coalesce.
         * It signals that data changed, not the number of updates.
         */
        xSemaphoreGive(xDataReadySem);
    }
}

/* ============================================================
 * STALE READING HANDLING
 * ============================================================ */

static void invalidate_stale_readings_locked(TickType_t now)
{
    if (g_weather.dhtValid &&
        (TickType_t)(now - g_weather.dhtLastSuccess) >
        pdMS_TO_TICKS(DHT_STALE_MS)) {
        g_weather.dhtValid = false;
    }

    if (g_weather.lightValid &&
        (TickType_t)(now - g_weather.lightLastSuccess) >
        pdMS_TO_TICKS(LDR_STALE_MS)) {
        g_weather.lightValid = false;
    }

    if (g_weather.rainValid &&
        (TickType_t)(now - g_weather.rainLastSuccess) >
        pdMS_TO_TICKS(RAIN_STALE_MS)) {
        g_weather.rainValid = false;
    }

    if (g_weather.bmpValid &&
        (TickType_t)(now - g_weather.bmpLastSuccess) >
        pdMS_TO_TICKS(BMP_STALE_MS)) {
        g_weather.bmpValid = false;
    }
}

/*
 * Copy a consistent snapshot.
 *
 * With USE_MUTEX=1, the mutex protects both stale-value updates
 * and the copy. The sequence counter provides a diagnostic value
 * that can be reported in the UART log.
 */
static bool get_validated_snapshot(
    WeatherData_t *out,
    unsigned int *sequenceOut,
    const char *taskName)
{
    if (out == NULL) {
        return false;
    }

    data_lock(taskName);

    TickType_t now = xTaskGetTickCount();

    invalidate_stale_readings_locked(now);
    g_weather.timestamp = now;

    unsigned int before = atomic_load_explicit(
        &g_dataSequence,
        memory_order_acquire
    );

    *out = g_weather;

    unsigned int after = atomic_load_explicit(
        &g_dataSequence,
        memory_order_acquire
    );

    data_unlock();

    if (sequenceOut != NULL) {
        *sequenceOut = after;
    }

    /*
     * An even, unchanged sequence indicates no writer overlapped
     * the snapshot according to this diagnostic counter.
     *
     * With USE_MUTEX=0, this check is only diagnostic. Concurrent
     * access to ordinary C struct fields is still unsafe.
     */
    return (before == after) && ((before & 1U) == 0U);
}

/* ============================================================
 * ADC AVERAGING
 * ============================================================ */

static esp_err_t read_adc_average(
    adc_channel_t channel,
    int *average)
{
    if (average == NULL || adc_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int sum = 0;

    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        int raw = 0;

        esp_err_t ret = adc_oneshot_read(
            adc_handle,
            channel,
            &raw
        );

        if (ret != ESP_OK) {
            return ret;
        }

        sum += raw;

        if (i < ADC_SAMPLE_COUNT - 1) {
            vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_DELAY_MS));
        }
    }

    *average = sum / ADC_SAMPLE_COUNT;

    return ESP_OK;
}

/* ============================================================
 * I2C INITIALISATION
 * ============================================================ */

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
             "I2C ready: SDA=%d, SCL=%d",
             I2C_SDA_GPIO,
             I2C_SCL_GPIO);
}

/* ============================================================
 * BMP280 INITIALISATION
 * ============================================================ */

static void init_bmp280(void)
{
    bmp280_config_t cfg = I2C_BMP280_CONFIG_DEFAULT;
    cfg.i2c_address = BMP280_I2C_ADDR;

    esp_err_t ret = bmp280_init(
        i2c_bus,
        &cfg,
        &bmp280_dev
    );

    if (ret != ESP_OK || bmp280_dev == NULL) {
        ESP_LOGE(TAG,
                 "BMP280 init failed: %s",
                 esp_err_to_name(ret));

        bmp280_dev = NULL;
    } else {
        ESP_LOGI(TAG,
                 "BMP280 initialised at 0x%02X",
                 BMP280_I2C_ADDR);
    }
}

/* ============================================================
 * SSD1306 INITIALISATION
 * ============================================================ */

static void init_ssd1306(void)
{
    ssd1306_config_t cfg =
        I2C_SSD1306_128x64_CONFIG_DEFAULT;

    cfg.i2c_address = SSD1306_I2C_ADDR;

    esp_err_t ret = ssd1306_init(
        i2c_bus,
        &cfg,
        &oled_dev
    );

    if (ret != ESP_OK || oled_dev == NULL) {
        ESP_LOGE(TAG,
                 "SSD1306 init failed: %s",
                 esp_err_to_name(ret));

        oled_dev = NULL;
        return;
    }

    /*
     * The display is initialised before tasks start.
     * The existing SSD1306 API used by this project has
     * a four-argument display_text function.
     */
    ssd1306_clear_display(oled_dev, false);

    ESP_LOGI(TAG,
             "SSD1306 initialised at 0x%02X",
             SSD1306_I2C_ADDR);
}

/* ============================================================
 * ADC INITIALISATION
 * ============================================================ */

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

/* ============================================================
 * DHT11 TASK
 * ============================================================ */

static void vDhtTask(void *pv)
{
    (void)pv;

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

        TickType_t now = xTaskGetTickCount();

        data_lock("DHT");
        begin_data_write();

        if (ret == ESP_OK) {
            g_weather.temperature = temperature;
            g_weather.humidity = humidity;
            g_weather.dhtValid = true;
            g_weather.dhtLastSuccess = now;
        } else {
            g_weather.dhtValid = false;
        }

        g_weather.timestamp = now;

        end_data_write();
        data_unlock();

        if (ret == ESP_OK) {
            ESP_LOGI(TAG,
                     "DHT11: Temp=%.2f C, Humidity=%.2f %%",
                     temperature,
                     humidity);

            publish_new_data();
        } else {
            ESP_LOGW(TAG,
                     "DHT11 read failed: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, period);
    }
}

/* ============================================================
 * LDR TASK
 * ============================================================ */

static void vLdrTask(void *pv)
{
    (void)pv;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(LDR_INTERVAL_MS);

    while (1) {
        int raw = 0;

        esp_err_t ret = read_adc_average(
            LDR_ADC_CHANNEL,
            &raw
        );

        TickType_t now = xTaskGetTickCount();

        data_lock("LDR");
        begin_data_write();

        if (ret == ESP_OK) {
            g_weather.lightPercent =
                (raw / 4095.0f) * 100.0f;

            g_weather.lightValid = true;
            g_weather.lightLastSuccess = now;
        } else {
            g_weather.lightValid = false;
        }

        g_weather.timestamp = now;

        end_data_write();
        data_unlock();

        if (ret == ESP_OK) {
            ESP_LOGI(TAG,
                     "LDR: ADC=%d, Scaled=%.1f%%",
                     raw,
                     (raw / 4095.0f) * 100.0f);

            publish_new_data();
        } else {
            ESP_LOGE(TAG,
                     "LDR ADC failed: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, period);
    }
}

/* ============================================================
 * RAIN SENSOR TASK
 * ============================================================ */

static void vRainTask(void *pv)
{
    (void)pv;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(RAIN_INTERVAL_MS);

    while (1) {
        int raw = 0;

        esp_err_t ret = read_adc_average(
            RAIN_ADC_CHANNEL,
            &raw
        );

        TickType_t now = xTaskGetTickCount();

        data_lock("RAIN");
        begin_data_write();

        if (ret == ESP_OK) {
            /*
             * Assumption: lower ADC readings indicate wetter
             * conditions. Verify this using actual sensor readings.
             */
            g_weather.rainPercent =
                (1.0f - raw / 4095.0f) * 100.0f;

            g_weather.rainValid = true;
            g_weather.rainLastSuccess = now;
        } else {
            g_weather.rainValid = false;
        }

        g_weather.timestamp = now;

        end_data_write();
        data_unlock();

        if (ret == ESP_OK) {
            ESP_LOGI(TAG,
                     "RAIN: ADC=%d, Scaled=%.1f%%",
                     raw,
                     (1.0f - raw / 4095.0f) * 100.0f);

            publish_new_data();
        } else {
            ESP_LOGE(TAG,
                     "Rain ADC failed: %s",
                     esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, period);
    }
}

/* ============================================================
 * BMP280 TASK
 * ============================================================ */

static void vBmp280Task(void *pv)
{
    (void)pv;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(BMP280_INTERVAL_MS);

    while (1) {
        float temperature = 0.0f;
        float pressure = 0.0f;
        bool valid = false;

        if (bmp280_dev != NULL) {
            if (xSemaphoreTake(
                    xI2CMutex,
                    pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS))
                == pdTRUE) {

                esp_err_t ret = bmp280_get_measurements(
                    bmp280_dev,
                    &temperature,
                    &pressure
                );

                if (ret == ESP_OK && pressure > 0.0f) {
                    valid = true;
                } else {
                    ESP_LOGW(TAG,
                             "BMP280 measurement failed: %s",
                             esp_err_to_name(ret));
                }

                xSemaphoreGive(xI2CMutex);
            } else {
                ESP_LOGW(TAG,
                         "BMP280 timed out waiting for I2C");
            }
        }

        TickType_t now = xTaskGetTickCount();

        data_lock("BMP280");
        begin_data_write();

        if (valid) {
            /* Driver pressure output is assumed to be in Pa. */
            g_weather.pressureHpa = pressure / 100.0f;
            g_weather.bmpValid = true;
            g_weather.bmpLastSuccess = now;
        } else {
            g_weather.bmpValid = false;
        }

        g_weather.timestamp = now;

        end_data_write();
        data_unlock();

        if (valid) {
            ESP_LOGI(TAG,
                     "BMP280: Temp=%.2f C, Pressure=%.2f hPa",
                     temperature,
                     pressure / 100.0f);

            publish_new_data();
        }

        vTaskDelayUntil(&last, period);
    }
}

/* ============================================================
 * OLED DISPLAY TASK
 * ============================================================ */

static void vDisplayTask(void *pv)
{
    (void)pv;

    while (1) {
        /*
         * Refresh on notification or timeout.
         * No additional delay is required after this wait.
         */
        (void)xSemaphoreTake(
            xDataReadySem,
            pdMS_TO_TICKS(DISPLAY_INTERVAL_MS)
        );

        WeatherData_t snap = {0};
        unsigned int sequence = 0;

        bool consistent = get_validated_snapshot(
            &snap,
            &sequence,
            "DISPLAY"
        );

        if (!consistent) {
            ESP_LOGW(TAG,
                     "DISPLAY: snapshot sequence inconsistent");
        }

        if (oled_dev == NULL) {
            continue;
        }

        if (xSemaphoreTake(
                xI2CMutex,
                pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS))
            != pdTRUE) {
            ESP_LOGW(TAG, "DISPLAY: I2C mutex timeout");
            continue;
        }

        char line[32];

        ssd1306_clear_display(oled_dev, false);

        ssd1306_display_text(
            oled_dev, 0, "WEATHER STATION", false);

        if (snap.dhtValid) {
            snprintf(line, sizeof(line),
                     "Temp: %.1f C", snap.temperature);
        } else {
            snprintf(line, sizeof(line), "Temp: N/A");
        }
        ssd1306_display_text(oled_dev, 1, line, false);

        if (snap.dhtValid) {
            snprintf(line, sizeof(line),
                     "Hum: %.1f %%", snap.humidity);
        } else {
            snprintf(line, sizeof(line), "Hum: N/A");
        }
        ssd1306_display_text(oled_dev, 2, line, false);

        if (snap.lightValid) {
            snprintf(line, sizeof(line),
                     "Light: %.1f %%", snap.lightPercent);
        } else {
            snprintf(line, sizeof(line), "Light: N/A");
        }
        ssd1306_display_text(oled_dev, 3, line, false);

        if (snap.rainValid) {
            snprintf(line, sizeof(line),
                     "Rain: %.1f %%", snap.rainPercent);
        } else {
            snprintf(line, sizeof(line), "Rain: N/A");
        }
        ssd1306_display_text(oled_dev, 4, line, false);

        if (snap.bmpValid) {
            snprintf(line, sizeof(line),
                     "Press: %.1f hPa", snap.pressureHpa);
        } else {
            snprintf(line, sizeof(line), "Press: N/A");
        }
        ssd1306_display_text(oled_dev, 5, line, false);

        xSemaphoreGive(xI2CMutex);

        ESP_LOGD(TAG,
                 "OLED refreshed, sequence=%u",
                 sequence);
    }
}

/* ============================================================
 * UART LOGGING TASK
 * ============================================================ */

static void vLoggingTask(void *pv)
{
    (void)pv;

    TickType_t last = xTaskGetTickCount();
    const TickType_t period =
        pdMS_TO_TICKS(LOG_INTERVAL_MS);

    while (1) {
        WeatherData_t snap = {0};
        unsigned int sequence = 0;

        bool consistent = get_validated_snapshot(
            &snap,
            &sequence,
            "LOG"
        );

        ESP_LOGI(TAG,
                 "DHT=%s T=%.2fC H=%.2f%% | "
                 "LDR=%s %.1f%% | "
                 "RAIN=%s %.1f%% | "
                 "BMP=%s %.2fhPa | "
                 "SEQ=%u CONSISTENT=%s",
                 snap.dhtValid ? "OK" : "INVALID",
                 snap.temperature,
                 snap.humidity,
                 snap.lightValid ? "OK" : "INVALID",
                 snap.lightPercent,
                 snap.rainValid ? "OK" : "INVALID",
                 snap.rainPercent,
                 snap.bmpValid ? "OK" : "INVALID",
                 snap.pressureHpa,
                 sequence,
                 consistent ? "YES" : "NO");

        ESP_LOGI(TAG,
                 "MUTEX_STATS: contentions=%u slow_waits=%u "
                 "max_wait_ticks=%u USE_MUTEX=%d",
                 atomic_load(&g_dataLockContentions),
                 atomic_load(&g_dataLockSlowWaits),
                 atomic_load(&g_dataLockMaxWaitTicks),
                 USE_MUTEX);

        vTaskDelayUntil(&last, period);
    }
}

/* ============================================================
 * APPLICATION ENTRY POINT
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "Starting weather monitoring station");
    ESP_LOGI(TAG, "USE_MUTEX=%d", USE_MUTEX);

    xDataMutex = xSemaphoreCreateMutex();
    xDataReadySem = xSemaphoreCreateBinary();
    xI2CMutex = xSemaphoreCreateMutex();

    configASSERT(xDataMutex != NULL);
    configASSERT(xDataReadySem != NULL);
    configASSERT(xI2CMutex != NULL);

    init_i2c_bus();
    init_bmp280();
    init_ssd1306();
    init_adc();

    BaseType_t result;

    result = xTaskCreate(
        vDhtTask, "DHT",
        4096, NULL, PRIO_SENSOR, NULL);
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vLdrTask, "LDR",
        2048, NULL, PRIO_SENSOR, NULL);
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vRainTask, "RAIN",
        2048, NULL, PRIO_SENSOR, NULL);
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vBmp280Task, "BMP280",
        3072, NULL, PRIO_SENSOR, NULL);
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vDisplayTask, "DISPLAY",
        4096, NULL, PRIO_DISPLAY, NULL);
    configASSERT(result == pdPASS);

    result = xTaskCreate(
        vLoggingTask, "LOG",
        4096, NULL, PRIO_LOGGING, NULL);
    configASSERT(result == pdPASS);

    ESP_LOGI(TAG, "All tasks created successfully");
}
