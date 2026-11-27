// power test 10: alternating 3 s light sleep (sensors off, then on) and 0.5 s awake.
// Wiring: soil VCC on GPIO4, DHT11 VCC on GPIO33. Build with the V2 menuconfig
// (README).

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_sleep.h"

#define WAKE_PIN       GPIO_NUM_25
#define BUZZER_GPIO    GPIO_NUM_27
#define MARKER_GPIO    GPIO_NUM_32
#define SOIL_VCC_GPIO  GPIO_NUM_4
#define DHT_VCC_GPIO   GPIO_NUM_33
#define DHT_GPIO       GPIO_NUM_26

#define LIGHT_SLEEP_MS 3000
#define AWAKE_MS       500

static void out_low(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

static void sensors_power(int on)
{
    gpio_set_level(SOIL_VCC_GPIO, on);
    gpio_set_level(DHT_VCC_GPIO, on);
}

// GPIO output levels are kept during light sleep, so the sensors stay powered (or
// unpowered) while the CPU sleeps
static void light_sleep_ms(uint32_t ms)
{
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)ms * 1000));
    (void)esp_light_sleep_start();
}

void app_main(void)
{
    // release holds left by state 02, otherwise these pins cannot change
    gpio_hold_dis(BUZZER_GPIO);
    gpio_hold_dis(MARKER_GPIO);
    gpio_hold_dis(SOIL_VCC_GPIO);
    gpio_hold_dis(DHT_VCC_GPIO);

    out_low(BUZZER_GPIO);
    out_low(MARKER_GPIO);
    out_low(SOIL_VCC_GPIO);
    out_low(DHT_VCC_GPIO);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    gpio_set_level(WAKE_PIN, 1);
    gpio_config_t wake = { .pin_bit_mask = 1ULL << WAKE_PIN, .mode = GPIO_MODE_OUTPUT_OD };
    gpio_config(&wake);

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (1) {
        // A: light sleep, sensors off
        sensors_power(0);
        light_sleep_ms(LIGHT_SLEEP_MS);

        // B: awake
        vTaskDelay(pdMS_TO_TICKS(AWAKE_MS));

        // C: light sleep, sensors on (as a warm-up done in light sleep)
        sensors_power(1);
        light_sleep_ms(LIGHT_SLEEP_MS);
        sensors_power(0);

        // D: awake
        vTaskDelay(pdMS_TO_TICKS(AWAKE_MS));
    }
}
