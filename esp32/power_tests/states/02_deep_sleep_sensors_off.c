// power test 02: deep sleep with switched-off sensors and held pins, as in V2/V3.
// Wiring: soil VCC on GPIO4, DHT11 VCC on GPIO33. Wakes every 15 s.

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_err.h"
#include "esp_sleep.h"

#define WAKE_PIN       GPIO_NUM_25
#define PI_TX_PIN      GPIO_NUM_17
#define PI_RX_PIN      GPIO_NUM_16
#define BUZZER_GPIO    GPIO_NUM_27
#define MARKER_GPIO    GPIO_NUM_32
#define BUTTON_GPIO    GPIO_NUM_0
#define SOIL_VCC_GPIO  GPIO_NUM_4
#define DHT_VCC_GPIO   GPIO_NUM_33
#define DHT_GPIO       GPIO_NUM_26

#define HOLD_S         15

static void out_low(gpio_num_t pin)
{
    gpio_reset_pin(pin);
    gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    gpio_set_level(pin, 0);
}

void app_main(void)
{
    // release pins held during the previous sleep, otherwise they cannot change
    gpio_hold_dis(BUZZER_GPIO);
    gpio_hold_dis(MARKER_GPIO);
    gpio_hold_dis(SOIL_VCC_GPIO);
    gpio_hold_dis(DHT_VCC_GPIO);

    // everything off, as in V2 just before esp_deep_sleep_start()
    out_low(BUZZER_GPIO);
    out_low(MARKER_GPIO);
    out_low(SOIL_VCC_GPIO);            // soil sensor unpowered
    out_low(DHT_VCC_GPIO);             // DHT11 unpowered
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);   // data pin does not feed the DHT

    gpio_set_level(WAKE_PIN, 1);
    gpio_config_t wake = { .pin_bit_mask = 1ULL << WAKE_PIN, .mode = GPIO_MODE_OUTPUT_OD };
    gpio_config(&wake);

    gpio_config_t uart_pins = {
        .pin_bit_mask = (1ULL << PI_TX_PIN) | (1ULL << PI_RX_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&uart_pins);

    // hold them LOW through deep sleep (all are RTC pins)
    gpio_hold_en(BUZZER_GPIO);
    gpio_hold_en(MARKER_GPIO);
    gpio_hold_en(SOIL_VCC_GPIO);
    gpio_hold_en(DHT_VCC_GPIO);

    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)HOLD_S * 1000000));
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(BUTTON_GPIO, 0));
    rtc_gpio_pullup_en(BUTTON_GPIO);
    rtc_gpio_pulldown_dis(BUTTON_GPIO);

    esp_deep_sleep_start();
}
