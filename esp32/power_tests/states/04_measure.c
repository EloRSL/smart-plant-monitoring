// power test 04: one V0/V1 measurement (DHT11 + 16 ADC reads) every 3 s, sensors always
// powered. Wiring: sensor VCC on 3V3. Run at 160 MHz (V0/V1) or with the V2 menuconfig
// (README).

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
#define MOISTURE_CHANNEL  ADC_CHANNEL_6

// same values as in V0/V1
#define SOIL_RAW_AIR          0
#define SOIL_RAW_WATER        2723
#define SOIL_SAMPLES          16
#define DHT_STARTUP_DELAY_MS  2000
#define DHT_RETRIES           2
#define DHT_RETRY_DELAY_MS    1100

#define PERIOD_MS             3000     // one measurement every 3 s

static const char *TAG = "plant";
static adc_oneshot_unit_handle_t s_adc;

static void sensors_init(void)
{
    adc_oneshot_unit_init_cfg_t unit = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit, &s_adc));
    adc_oneshot_chan_cfg_t channel = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, MOISTURE_CHANNEL, &channel));
    ESP_ERROR_CHECK(gpio_set_pull_mode(DHT_GPIO, GPIO_FLOATING));
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

// same work as measure() in V0/V1
static void measure(void)
{
    float hum = 0, temp = 0;
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt <= DHT_RETRIES; ++attempt) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(DHT_RETRY_DELAY_MS));
        err = dht_read_float_data(DHT_TYPE_DHT11, DHT_GPIO, &hum, &temp);
        if (err == ESP_OK) break;
    }
    int raw = -1;
    read_soil_raw(&raw);
    int soil = (SOIL_RAW_AIR - raw) * 1000 / (SOIL_RAW_AIR - SOIL_RAW_WATER);
    if (soil < 0) soil = 0;
    if (soil > 1000) soil = 1000;
    ESP_LOGI(TAG, "measured: air_temperature=%d air_humidity=%d soil_humidity=%d (raw %d)",
             err == ESP_OK ? (int)temp : -128, err == ESP_OK ? (int)hum : -1, soil, raw);
}

void app_main(void)
{
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MARKER_GPIO);
    gpio_set_direction(MARKER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MARKER_GPIO, 0);

    sensors_init();
    vTaskDelay(pdMS_TO_TICKS(DHT_STARTUP_DELAY_MS));

    while (1) {
        gpio_set_level(MARKER_GPIO, 1);   // measurement starts
        measure();
        gpio_set_level(MARKER_GPIO, 0);   // measurement done
        vTaskDelay(pdMS_TO_TICKS(PERIOD_MS));
    }
}
