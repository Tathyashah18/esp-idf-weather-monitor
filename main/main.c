/**********************************************************************
 * Weather Monitoring Station — DHT11 + LDR + RAIN + BMP280
 * Output : SSD1306 OLED + CSV log on SPIFFS
 *
 * KEY FEATURE (this version):
 *   - CSV is cleared on every boot  (CLEAR_CSV_ON_BOOT = 1)
 *   - csv_open() uses fopen(..., "w") which truncates any existing file
 *   - Optionally wipes the whole SPIFFS partition before opening
 *
 * Synchronization:
 *   xDataMutex    - protects g_weather (shared memory)
 *   xDataReadySem - binary semaphore (new data -> DISPLAY)
 *   xI2CMutex     - serialises the shared I2C bus (BMP280 + OLED)
 *   xAdcMutex     - serialises LDR + RAIN on the ADC peripheral
 **********************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"

#include "dht.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "config.h"

/* ============================================================
 * USER-ADJUSTABLE FLAGS
 * ============================================================ */

#define USE_MUTEX              1
#define TRACE_SYNC             1

/* Clear the CSV at every boot (1) or keep appending across reboots (0) */
#define CLEAR_CSV_ON_BOOT      1

/* Additionally format the entire SPIFFS partition at boot (very destructive) */
#define FORMAT_SPIFFS_ON_BOOT  0

#define ADC_SAMPLE_COUNT       8
#define ADC_SAMPLE_DELAY_MS    2
#define I2C_MUTEX_TIMEOUT_MS   1000
#define DATA_MUTEX_WARN_MS     5

#define DHT_STALE_MS      (DHT_INTERVAL_MS    * 3)
#define LDR_STALE_MS      (LDR_INTERVAL_MS    * 3)
#define RAIN_STALE_MS     (RAIN_INTERVAL_MS   * 3)
#define BMP_STALE_MS      (BMP280_INTERVAL_MS * 3)

static const char *TAG = "WEATHER";

/* ============================================================
 * SYNC TRACE MACRO
 * ============================================================ */

#if TRACE_SYNC
#   define SYNC_TRACE(task, obj, action) \
        ESP_LOGI("SYNC", "%-8s | %-16s | %s", task, obj, action)
#else
#   define SYNC_TRACE(task, obj, action) do { } while (0)
#endif

/* ============================================================
 * SHARED SENSOR DATA
 * ============================================================ */

typedef struct {
    float temperature;
    float humidity;
    float lightPercent;
    float rainPercent;
    float pressureHpa;

    bool  dhtValid;
    bool  lightValid;
    bool  rainValid;
    bool  bmpValid;

    TickType_t dhtLastSuccess;
    TickType_t lightLastSuccess;
    TickType_t rainLastSuccess;
    TickType_t bmpLastSuccess;
    TickType_t timestamp;
} WeatherData_t;

static WeatherData_t g_weather = {0};
static atomic_uint   g_dataSequence = ATOMIC_VAR_INIT(0);

static atomic_uint g_dataLockContentions = ATOMIC_VAR_INIT(0);
static atomic_uint g_dataLockSlowWaits   = ATOMIC_VAR_INIT(0);
static atomic_uint g_dataLockMaxWait     = ATOMIC_VAR_INIT(0);

/* ============================================================
 * SYNC OBJECTS + HW HANDLES
 * ============================================================ */

static SemaphoreHandle_t xDataMutex    = NULL;
static SemaphoreHandle_t xDataReadySem = NULL;
static SemaphoreHandle_t xI2CMutex     = NULL;
static SemaphoreHandle_t xAdcMutex     = NULL;

static i2c_master_bus_handle_t   i2c_bus    = NULL;
static bmp280_handle_t           bmp280_dev = NULL;
static ssd1306_handle_t          oled_dev   = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;

/* ============================================================
 * CSV FILE HANDLE + STATS
 * ============================================================ */

static FILE    *g_csv      = NULL;
static uint32_t g_csv_rows = 0;

/* ============================================================
 * DATA MUTEX — traced
 * ============================================================ */

static void update_max_wait(TickType_t elapsed)
{
    unsigned int prev = atomic_load(&g_dataLockMaxWait);
    while (elapsed > prev) {
        if (atomic_compare_exchange_weak(&g_dataLockMaxWait, &prev, (unsigned)elapsed))
            break;
    }
}

static void data_lock(const char *task_name)
{
#if USE_MUTEX
    TickType_t start = xTaskGetTickCount();
    if (xSemaphoreTake(xDataMutex, 0) != pdTRUE) {
        atomic_fetch_add(&g_dataLockContentions, 1);
        SYNC_TRACE(task_name, "xDataMutex", "WAITING");
        xSemaphoreTake(xDataMutex, portMAX_DELAY);
    }
    SYNC_TRACE(task_name, "xDataMutex", "TAKEN");

    TickType_t elapsed = xTaskGetTickCount() - start;
    update_max_wait(elapsed);

    if (elapsed >= pdMS_TO_TICKS(DATA_MUTEX_WARN_MS)) {
        atomic_fetch_add(&g_dataLockSlowWaits, 1);
        ESP_LOGW(TAG, "%s waited %" PRIu32 " ticks for data mutex",
                 task_name, (uint32_t)elapsed);
    }
#else
    (void)task_name;
#endif
}

static void data_unlock(void)
{
#if USE_MUTEX
    SYNC_TRACE(pcTaskGetName(NULL), "xDataMutex", "GIVEN");
    xSemaphoreGive(xDataMutex);
#endif
}

/* ============================================================
 * DATA SEQUENCE + NEW-DATA SIGNAL
 * ============================================================ */

static void begin_data_write(void) {
    atomic_fetch_add_explicit(&g_dataSequence, 1, memory_order_relaxed);
}
static void end_data_write(void) {
    atomic_fetch_add_explicit(&g_dataSequence, 1, memory_order_release);
}

static void publish_new_data(const char *task_name)
{
    if (xDataReadySem) {
        SYNC_TRACE(task_name, "xDataReadySem", "GIVEN");
        xSemaphoreGive(xDataReadySem);
    }
}

/* ============================================================
 * I2C / ADC MUTEX WRAPPERS
 * ============================================================ */

static bool i2c_lock(const char *task_name)
{
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(I2C_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        SYNC_TRACE(task_name, "xI2CMutex", "TAKEN");
        return true;
    }
    SYNC_TRACE(task_name, "xI2CMutex", "TIMEOUT");
    return false;
}

static void i2c_unlock(const char *task_name)
{
    SYNC_TRACE(task_name, "xI2CMutex", "GIVEN");
    xSemaphoreGive(xI2CMutex);
}

static bool adc_lock(const char *task_name)
{
    if (xSemaphoreTake(xAdcMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        SYNC_TRACE(task_name, "xAdcMutex", "TAKEN");
        return true;
    }
    SYNC_TRACE(task_name, "xAdcMutex", "TIMEOUT");
    return false;
}

static void adc_unlock(const char *task_name)
{
    SYNC_TRACE(task_name, "xAdcMutex", "GIVEN");
    xSemaphoreGive(xAdcMutex);
}

/* ============================================================
 * STALE INVALIDATION + SNAPSHOT
 * ============================================================ */

static void invalidate_stale_readings_locked(TickType_t now)
{
    if (g_weather.dhtValid   &&
        (now - g_weather.dhtLastSuccess)   > pdMS_TO_TICKS(DHT_STALE_MS))
        g_weather.dhtValid = false;

    if (g_weather.lightValid &&
        (now - g_weather.lightLastSuccess) > pdMS_TO_TICKS(LDR_STALE_MS))
        g_weather.lightValid = false;

    if (g_weather.rainValid  &&
        (now - g_weather.rainLastSuccess)  > pdMS_TO_TICKS(RAIN_STALE_MS))
        g_weather.rainValid = false;

    if (g_weather.bmpValid   &&
        (now - g_weather.bmpLastSuccess)   > pdMS_TO_TICKS(BMP_STALE_MS))
        g_weather.bmpValid = false;
}

static bool get_validated_snapshot(WeatherData_t *out,
                                   unsigned int  *sequence_out,
                                   const char    *task_name)
{
    if (!out) return false;

    data_lock(task_name);

    TickType_t now = xTaskGetTickCount();
    invalidate_stale_readings_locked(now);
    g_weather.timestamp = now;

    unsigned int before = atomic_load_explicit(&g_dataSequence, memory_order_acquire);
    *out = g_weather;
    unsigned int after  = atomic_load_explicit(&g_dataSequence, memory_order_acquire);

    data_unlock();

    if (sequence_out) *sequence_out = after;
    return (before == after) && ((before & 1U) == 0U);
}

/* ============================================================
 * ADC AVERAGE
 * ============================================================ */

static esp_err_t read_adc_average(adc_channel_t channel,
                                  int *average,
                                  const char *task_name)
{
    if (!average || !adc_handle) return ESP_ERR_INVALID_ARG;
    if (!adc_lock(task_name))    return ESP_ERR_TIMEOUT;

    int sum = 0;
    esp_err_t ret = ESP_OK;

    for (int i = 0; i < ADC_SAMPLE_COUNT; i++) {
        int raw = 0;
        ret = adc_oneshot_read(adc_handle, channel, &raw);
        if (ret != ESP_OK) break;
        sum += raw;
        if (i < ADC_SAMPLE_COUNT - 1)
            vTaskDelay(pdMS_TO_TICKS(ADC_SAMPLE_DELAY_MS));
    }
    if (ret == ESP_OK) *average = sum / ADC_SAMPLE_COUNT;

    adc_unlock(task_name);
    return ret;
}

/* ============================================================
 * TIME INITIALISATION  (compile-time stamp, no Wi-Fi required)
 * ============================================================ */

static void init_time(void)
{
    static const char *months[] = {"Jan","Feb","Mar","Apr","May","Jun",
                                   "Jul","Aug","Sep","Oct","Nov","Dec"};
    char mon[4] = {0};
    int day = 1, year = 2024, h = 0, m = 0, s = 0;

    sscanf(__DATE__, "%3s %d %d", mon, &day, &year);
    sscanf(__TIME__, "%d:%d:%d", &h, &m, &s);

    int month = 0;
    for (int i = 0; i < 12; i++)
        if (strncmp(mon, months[i], 3) == 0) { month = i; break; }

    struct tm tm0 = {
        .tm_year = year - 1900, .tm_mon = month, .tm_mday = day,
        .tm_hour = h, .tm_min = m, .tm_sec = s,
    };
    struct timeval tv = { .tv_sec = mktime(&tm0), .tv_usec = 0 };
    settimeofday(&tv, NULL);

    setenv("TZ", LOCAL_TIMEZONE, 1);
    tzset();

    char buf[32];
    struct tm now;
    time_t t = time(NULL);
    localtime_r(&t, &now);
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &now);
    ESP_LOGI(TAG, "Clock starts at %s %s", buf, LOCAL_TIMEZONE);
}

/* ============================================================
 * SPIFFS MOUNT  (with optional full format)
 * ============================================================ */

static bool init_storage(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = STORAGE_MOUNT_POINT,
        .partition_label        = STORAGE_PARTITION,
        .max_files              = 4,
        .format_if_mount_failed = true,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        return false;
    }

#if FORMAT_SPIFFS_ON_BOOT
    ESP_LOGW(TAG, "Formatting entire SPIFFS partition ...");
    esp_vfs_spiffs_unregister(STORAGE_PARTITION);
    esp_spiffs_format(STORAGE_PARTITION);
    if (esp_vfs_spiffs_register(&conf) != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS remount after format failed");
        return false;
    }
#endif

    size_t total = 0, used = 0;
    esp_spiffs_info(STORAGE_PARTITION, &total, &used);
    ESP_LOGI(TAG, "SPIFFS ready: total=%u used=%u free=%u bytes",
             (unsigned)total, (unsigned)used, (unsigned)(total - used));
    return true;
}

/* ============================================================
 * CSV — CLEAR + OPEN + WRITE
 * ============================================================ */

static void csv_clear(void)
{
    if (remove(CSV_PATH) == 0) {
        ESP_LOGI(TAG, "Cleared existing CSV: %s", CSV_PATH);
    } else {
        ESP_LOGI(TAG, "No existing CSV to clear (%s)", CSV_PATH);
    }
}

static bool csv_open(void)
{
    if (g_csv) return true;

    /* "w" truncates any existing file -> always starts fresh */
    g_csv = fopen(CSV_PATH, "w");
    if (!g_csv) {
        ESP_LOGE(TAG, "Cannot open %s for writing", CSV_PATH);
        return false;
    }

    fprintf(g_csv,
        "Date,Time,Uptime_s,"
        "Temperature_C,Humidity_pct,Light_pct,Rain_pct,Pressure_hPa\n");
    fflush(g_csv);

    g_csv_rows = 0;
    ESP_LOGI(TAG, "CSV opened fresh at %s", CSV_PATH);
    return true;
}

static void csv_write_row(const WeatherData_t *w)
{
    if (!g_csv) return;

    time_t t = time(NULL);
    struct tm ti;
    localtime_r(&t, &ti);

    char date_s[12], time_s[10];
    strftime(date_s, sizeof(date_s), "%Y-%m-%d", &ti);
    strftime(time_s, sizeof(time_s), "%H:%M:%S", &ti);

    uint32_t up_s = (uint32_t)(((uint64_t)w->timestamp * portTICK_PERIOD_MS) / 1000);

    if (w->dhtValid)
        fprintf(g_csv, "%s,%s,%lu,%.2f,%.2f,", date_s, time_s,
                (unsigned long)up_s, w->temperature, w->humidity);
    else
        fprintf(g_csv, "%s,%s,%lu,N/A,N/A,", date_s, time_s, (unsigned long)up_s);

    if (w->lightValid) fprintf(g_csv, "%.1f,", w->lightPercent);
    else               fprintf(g_csv, "N/A,");

    if (w->rainValid)  fprintf(g_csv, "%.1f,", w->rainPercent);
    else               fprintf(g_csv, "N/A,");

    if (w->bmpValid)   fprintf(g_csv, "%.2f\n", w->pressureHpa);
    else               fprintf(g_csv, "N/A\n");

    fflush(g_csv);
    g_csv_rows++;
}

/* ============================================================
 * HARDWARE INIT
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
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));
    ESP_LOGI(TAG, "I2C ready: SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);
}

static void init_bmp280(void)
{
    bmp280_config_t cfg = I2C_BMP280_CONFIG_DEFAULT;
    cfg.i2c_address = BMP280_I2C_ADDR;
    if (bmp280_init(i2c_bus, &cfg, &bmp280_dev) != ESP_OK || !bmp280_dev) {
        ESP_LOGE(TAG, "BMP280 init failed");
        bmp280_dev = NULL;
    } else {
        ESP_LOGI(TAG, "BMP280 ready at 0x%02X", BMP280_I2C_ADDR);
    }
}

static void init_ssd1306(void)
{
    ssd1306_config_t cfg = I2C_SSD1306_128x64_CONFIG_DEFAULT;
    cfg.i2c_address = SSD1306_I2C_ADDR;
    if (ssd1306_init(i2c_bus, &cfg, &oled_dev) != ESP_OK || !oled_dev) {
        ESP_LOGE(TAG, "SSD1306 init failed");
        oled_dev = NULL;
        return;
    }
    ssd1306_clear_display(oled_dev, false);
    ESP_LOGI(TAG, "SSD1306 ready at 0x%02X", SSD1306_I2C_ADDR);
}

static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, LDR_ADC_CHANNEL,  &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, RAIN_ADC_CHANNEL, &chan_cfg));
    ESP_LOGI(TAG, "ADC ready: LDR (CH6), RAIN (CH7)");
}

/* ============================================================
 * SENSOR TASKS
 * ============================================================ */

static void vDhtTask(void *pv)
{
    (void)pv;
    TickType_t last    = xTaskGetTickCount();
    const TickType_t p = pdMS_TO_TICKS(DHT_INTERVAL_MS);

    while (1) {
        float hum = 0.0f, temp = 0.0f;
        esp_err_t ret = dht_read_float_data(DHT_SENSOR_TYPE, DHT_GPIO, &hum, &temp);
        TickType_t now = xTaskGetTickCount();

        data_lock("DHT");
        begin_data_write();
        if (ret == ESP_OK) {
            g_weather.temperature    = temp;
            g_weather.humidity       = hum;
            g_weather.dhtValid       = true;
            g_weather.dhtLastSuccess = now;
        } else {
            g_weather.dhtValid = false;
        }
        g_weather.timestamp = now;
        end_data_write();
        data_unlock();

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "DHT11: T=%.2f C H=%.2f %%", temp, hum);
            publish_new_data("DHT");
        } else {
            ESP_LOGW(TAG, "DHT11 read failed: %s", esp_err_to_name(ret));
        }

        vTaskDelayUntil(&last, p);
    }
}

static void vLdrTask(void *pv)
{
    (void)pv;
    TickType_t last    = xTaskGetTickCount();
    const TickType_t p = pdMS_TO_TICKS(LDR_INTERVAL_MS);

    while (1) {
        int raw = 0;
        esp_err_t ret = read_adc_average(LDR_ADC_CHANNEL, &raw, "LDR");
        TickType_t now = xTaskGetTickCount();

        data_lock("LDR");
        begin_data_write();
        if (ret == ESP_OK) {
            g_weather.lightPercent     = (raw / 4095.0f) * 100.0f;
            g_weather.lightValid       = true;
            g_weather.lightLastSuccess = now;
        } else {
            g_weather.lightValid = false;
        }
        g_weather.timestamp = now;
        end_data_write();
        data_unlock();

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "LDR: ADC=%d Scaled=%.1f%%", raw, (raw / 4095.0f) * 100.0f);
            publish_new_data("LDR");
        }
        vTaskDelayUntil(&last, p);
    }
}

static void vRainTask(void *pv)
{
    (void)pv;
    TickType_t last    = xTaskGetTickCount();
    const TickType_t p = pdMS_TO_TICKS(RAIN_INTERVAL_MS);

    while (1) {
        int raw = 0;
        esp_err_t ret = read_adc_average(RAIN_ADC_CHANNEL, &raw, "RAIN");
        TickType_t now = xTaskGetTickCount();

        data_lock("RAIN");
        begin_data_write();
        if (ret == ESP_OK) {
            g_weather.rainPercent     = (1.0f - raw / 4095.0f) * 100.0f;
            g_weather.rainValid       = true;
            g_weather.rainLastSuccess = now;
        } else {
            g_weather.rainValid = false;
        }
        g_weather.timestamp = now;
        end_data_write();
        data_unlock();

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "RAIN: ADC=%d Scaled=%.1f%%", raw,
                     (1.0f - raw / 4095.0f) * 100.0f);
            publish_new_data("RAIN");
        }
        vTaskDelayUntil(&last, p);
    }
}

static void vBmp280Task(void *pv)
{
    (void)pv;
    TickType_t last    = xTaskGetTickCount();
    const TickType_t p = pdMS_TO_TICKS(BMP280_INTERVAL_MS);

    while (1) {
        float t = 0.0f, press = 0.0f;
        bool ok = false;

        if (bmp280_dev && i2c_lock("BMP280")) {
            if (bmp280_get_measurements(bmp280_dev, &t, &press) == ESP_OK && press > 0)
                ok = true;
            i2c_unlock("BMP280");
        }

        TickType_t now = xTaskGetTickCount();

        data_lock("BMP280");
        begin_data_write();
        if (ok) {
            g_weather.pressureHpa    = press / 100.0f;
            g_weather.bmpValid       = true;
            g_weather.bmpLastSuccess = now;
        } else {
            g_weather.bmpValid = false;
        }
        g_weather.timestamp = now;
        end_data_write();
        data_unlock();

        if (ok) {
            ESP_LOGI(TAG, "BMP280: T=%.2f C P=%.2f hPa", t, press / 100.0f);
            publish_new_data("BMP280");
        }
        vTaskDelayUntil(&last, p);
    }
}

/* ============================================================
 * OLED DISPLAY TASK — static labels, numbers update
 * ============================================================ */

static void pad_line(char *dst, size_t sz, const char *src)
{
    if (!dst || sz == 0) return;
    memset(dst, ' ', sz);
    size_t len = strlen(src);
    if (len > sz - 1) len = sz - 1;
    memcpy(dst, src, len);
    dst[sz - 1] = '\0';
}

static void vDisplayTask(void *pv)
{
    (void)pv;
    TickType_t last    = xTaskGetTickCount();
    const TickType_t p = pdMS_TO_TICKS(DISPLAY_INTERVAL_MS);

    char prev[6][17] = {{0}};
    char cur[6][17]  = {{0}};
    bool first = true;

    while (1) {
        if (xSemaphoreTake(xDataReadySem, pdMS_TO_TICKS(500)) == pdTRUE) {
            SYNC_TRACE("DISPLAY", "xDataReadySem", "TAKEN");
        }

        WeatherData_t s = {0};
        get_validated_snapshot(&s, NULL, "DISPLAY");

        if (oled_dev) {
            char buf[32];

            snprintf(cur[0], sizeof(cur[0]), "WEATHER STATION");

            if (s.dhtValid) snprintf(buf, sizeof(buf), "T: %.1f C",   s.temperature);
            else            snprintf(buf, sizeof(buf), "T: --.- C");
            pad_line(cur[1], sizeof(cur[1]), buf);

            if (s.dhtValid) snprintf(buf, sizeof(buf), "H: %.1f %%",  s.humidity);
            else            snprintf(buf, sizeof(buf), "H: --.- %%");
            pad_line(cur[2], sizeof(cur[2]), buf);

            if (s.lightValid) snprintf(buf, sizeof(buf), "L: %.1f %%", s.lightPercent);
            else              snprintf(buf, sizeof(buf), "L: --.- %%");
            pad_line(cur[3], sizeof(cur[3]), buf);

            if (s.rainValid) snprintf(buf, sizeof(buf), "R: %.1f %%",  s.rainPercent);
            else             snprintf(buf, sizeof(buf), "R: --.- %%");
            pad_line(cur[4], sizeof(cur[4]), buf);

            if (s.bmpValid) snprintf(buf, sizeof(buf), "P: %.1f hPa", s.pressureHpa);
            else            snprintf(buf, sizeof(buf), "P: --.- hPa");
            pad_line(cur[5], sizeof(cur[5]), buf);

            if (i2c_lock("DISPLAY")) {
                for (int r = 0; r < 6; r++) {
                    if (first || strcmp(cur[r], prev[r]) != 0) {
                        ssd1306_display_text(oled_dev, r, cur[r], false);
                        memcpy(prev[r], cur[r], sizeof(prev[r]));
                    }
                }
                first = false;
                i2c_unlock("DISPLAY");
            }
        }

        vTaskDelayUntil(&last, p);
    }
}

/* ============================================================
 * CSV LOGGING TASK — one row per second
 * ============================================================ */

static void vCsvLogTask(void *pv)
{
    (void)pv;
    TickType_t last    = xTaskGetTickCount();
    const TickType_t p = pdMS_TO_TICKS(1000);

    while (1) {
        WeatherData_t s = {0};
        get_validated_snapshot(&s, NULL, "CSVLOG");
        csv_write_row(&s);

        /* Status line every 30 s */
        if ((g_csv_rows % 30) == 0 && g_csv_rows > 0) {
            size_t total = 0, used = 0;
            esp_spiffs_info(STORAGE_PARTITION, &total, &used);
            ESP_LOGI(TAG, "CSV rows=%lu used=%u/%u bytes",
                     (unsigned long)g_csv_rows, (unsigned)used, (unsigned)total);
        }

        vTaskDelayUntil(&last, p);
    }
}

/* ============================================================
 * LOG TASK — 5-second snapshot summary
 * ============================================================ */

static void vLogTask(void *pv)
{
    (void)pv;
    TickType_t last    = xTaskGetTickCount();
    const TickType_t p = pdMS_TO_TICKS(LOG_INTERVAL_MS);

    while (1) {
        WeatherData_t s = {0};
        unsigned int seq = 0;
        bool consistent = get_validated_snapshot(&s, &seq, "LOG");

        ESP_LOGI(TAG,
            "DHT=%s T=%.2fC H=%.2f%% | LDR=%s %.1f%% | RAIN=%s %.1f%% | "
            "BMP=%s %.2fhPa | SEQ=%u CONSISTENT=%s",
            s.dhtValid   ? "OK" : "INV", s.temperature, s.humidity,
            s.lightValid ? "OK" : "INV", s.lightPercent,
            s.rainValid  ? "OK" : "INV", s.rainPercent,
            s.bmpValid   ? "OK" : "INV", s.pressureHpa,
            seq, consistent ? "YES" : "NO");

        ESP_LOGI(TAG,
            "MUTEX_STATS: contentions=%u slow_waits=%u max_wait=%u USE_MUTEX=%d",
            atomic_load(&g_dataLockContentions),
            atomic_load(&g_dataLockSlowWaits),
            atomic_load(&g_dataLockMaxWait),
            USE_MUTEX);

        vTaskDelayUntil(&last, p);
    }
}

/* ============================================================
 * APP MAIN
 * ============================================================ */

void app_main(void)
{
    ESP_LOGI(TAG, "Weather Station starting (DHT+LDR+RAIN+BMP280)");
    ESP_LOGI(TAG, "USE_MUTEX=%d TRACE_SYNC=%d CLEAR_CSV=%d FORMAT=%d",
             USE_MUTEX, TRACE_SYNC, CLEAR_CSV_ON_BOOT, FORMAT_SPIFFS_ON_BOOT);

    xDataMutex    = xSemaphoreCreateMutex();
    xDataReadySem = xSemaphoreCreateBinary();
    xI2CMutex     = xSemaphoreCreateMutex();
    xAdcMutex     = xSemaphoreCreateMutex();
    configASSERT(xDataMutex && xDataReadySem && xI2CMutex && xAdcMutex);

    init_time();

    if (init_storage()) {
#if CLEAR_CSV_ON_BOOT
        csv_clear();
#endif
        if (!csv_open()) {
            ESP_LOGE(TAG, "CSV disabled – logging will not run");
        }
    }

    init_i2c_bus();
    init_bmp280();
    init_ssd1306();
    init_adc();

    xTaskCreate(vDhtTask,     "DHT",     4096, NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(vLdrTask,     "LDR",     2048, NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(vRainTask,    "RAIN",    2048, NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(vBmp280Task,  "BMP280",  3072, NULL, PRIO_SENSOR,  NULL);
    xTaskCreate(vDisplayTask, "DISPLAY", 4096, NULL, PRIO_DISPLAY, NULL);
    xTaskCreate(vCsvLogTask,  "CSVLOG",  4096, NULL, PRIO_LOGGING, NULL);
    xTaskCreate(vLogTask,     "LOG",     3072, NULL, PRIO_LOGGING, NULL);

    ESP_LOGI(TAG, "All 7 tasks started");
}