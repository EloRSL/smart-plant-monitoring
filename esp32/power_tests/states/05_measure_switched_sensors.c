// power test 05: one V2/V3 measurement (power sensors, 1 s warm-up, read, power off)
// every 5 s. Wiring: soil VCC on GPIO4, DHT11 VCC on GPIO33. Build with the V2
// menuconfig (README).

#include <stdbool.h>

#include "dht.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define DHT_GPIO          GPIO_NUM_26
#define BUZZER_GPIO       GPIO_NUM_27
#define MARKER_GPIO       GPIO_NUM_32
#define SOIL_VCC_GPIO     GPIO_NUM_4
#define DHT_VCC_GPIO      GPIO_NUM_33
#define MOISTURE_CHANNEL  ADC_CHANNEL_6

// same values as V2 with V2_TRIM_MEASUREMENT = 1
#define SOIL_RAW_AIR          0
#define SOIL_RAW_WATER        2723
#define SOIL_SAMPLES          4
#define DHT_RETRIES           1
#define DHT_RETRY_DELAY_MS    1100
#define SENSOR_WARMUP_MS      1000

#define PERIOD_MS             5000

static const char *TAG = "plant";
static adc_oneshot_unit_handle_t s_adc;

static void sensor_power(bool on)
{
    gpio_set_level(SOIL_VCC_GPIO, on ? 1 : 0);
    gpio_set_level(DHT_VCC_GPIO, on ? 1 : 0);
}

static void sensors_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));
    adc_oneshot_chan_cfg_t channel = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, MOISTURE_CHANNEL, &channel));
    ESP_ERROR_CHECK(gpio_set_pull_mode(DHT_GPIO, GPIO_FLOATING));

    gpio_hold_dis(SOIL_VCC_GPIO);
    gpio_hold_dis(DHT_VCC_GPIO);
    gpio_reset_pin(SOIL_VCC_GPIO);
    gpio_reset_pin(DHT_VCC_GPIO);
    gpio_set_direction(SOIL_VCC_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(DHT_VCC_GPIO, GPIO_MODE_OUTPUT);
    sensor_power(false);
}

static bool read_soil_raw(int *raw_out)
{
    int sum = 0, ok = 0;
    for (int i = 0; i < SOIL_SAMPLES; ++i) {
        int raw;
        if (adc_oneshot_read(s_adc, MOISTURE_CHANNEL, &raw) == ESP_OK) {
            sum += raw;
            ok++;
        }
    }
    if (ok == 0) return false;
    *raw_out = sum / ok;
    return true;
}

// same work as measure() in V2
static void measure(void)
{
    sensor_power(true);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_WARMUP_MS));

    float hum = 0, temp = 0;
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt <= DHT_RETRIES; ++attempt) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(DHT_RETRY_DELAY_MS));
        err = dht_read_float_data(DHT_TYPE_DHT11, DHT_GPIO, &hum, &temp);
        if (err == ESP_OK) break;
    }
    int raw = -1;
    read_soil_raw(&raw);

    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);
    sensor_power(false);

    // info level: not printed with the Warning log level, same as V2
    ESP_LOGI(TAG, "measured: err=%d raw=%d", (int)err, raw);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);    // as V2_QUIET_LOG

    gpio_hold_dis(BUZZER_GPIO);
    gpio_hold_dis(MARKER_GPIO);
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MARKER_GPIO);
    gpio_set_direction(MARKER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MARKER_GPIO, 0);

    sensors_init();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
        gpio_set_level(MARKER_GPIO, 1);
        measure();
        gpio_set_level(MARKER_GPIO, 0);
    }
}
