// power test 03: awake and idle with UART2 installed, as V0 between measurements.
// Wiring: sensor VCC on 3V3. Default sdkconfig (160 MHz).

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PI_UART      UART_NUM_2
#define PI_TX_PIN    GPIO_NUM_17
#define PI_RX_PIN    GPIO_NUM_16
#define BUZZER_GPIO  GPIO_NUM_27
#define MARKER_GPIO  GPIO_NUM_32

void app_main(void)
{
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);

    gpio_reset_pin(MARKER_GPIO);
    gpio_set_direction(MARKER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MARKER_GPIO, 1);

    // UART2 installed exactly as in V0 (it stays installed all the time there)
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
    };
    ESP_ERROR_CHECK(uart_driver_install(PI_UART, 2048, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PI_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PI_UART, PI_TX_PIN, PI_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // idle like V0 between measurements: the task sleeps, the CPU does not
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
