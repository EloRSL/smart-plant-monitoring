// power test 01: deep sleep with the sensors on 3V3 and pins not held, as in V1.
// Wiring: sensor VCC on 3V3. Wakes every 15 s.

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_err.h"
#include "esp_sleep.h"

#define WAKE_PIN      GPIO_NUM_25
#define PI_TX_PIN     GPIO_NUM_17
#define PI_RX_PIN     GPIO_NUM_16
#define BUZZER_GPIO   GPIO_NUM_27
#define MARKER_GPIO   GPIO_NUM_32
#define BUTTON_GPIO   GPIO_NUM_0

#define HOLD_S        15

void app_main(void)
{
    // same pin state as V1 just before esp_deep_sleep_start()

    // buzzer LOW but NOT held: floats in sleep, like V1
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    // marker LOW = asleep, NOT held (V1)
    gpio_reset_pin(MARKER_GPIO);
    gpio_set_direction(MARKER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MARKER_GPIO, 0);

    // wake pin released (open drain)
    gpio_set_level(WAKE_PIN, 1);
    gpio_config_t wake = { .pin_bit_mask = 1ULL << WAKE_PIN, .mode = GPIO_MODE_OUTPUT_OD };
    gpio_config(&wake);

    // UART pins as after link_close(): plain inputs, no pulls
    gpio_config_t uart_pins = {
        .pin_bit_mask = (1ULL << PI_TX_PIN) | (1ULL << PI_RX_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&uart_pins);

    // same wake-up sources as V1: timer + BOOT button (keeps RTC periph on)
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)HOLD_S * 1000000));
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(BUTTON_GPIO, 0));
    rtc_gpio_pullup_en(BUTTON_GPIO);
    rtc_gpio_pulldown_dis(BUTTON_GPIO);

    esp_deep_sleep_start();
}
