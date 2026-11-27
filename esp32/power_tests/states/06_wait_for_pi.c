// power tests 06/07: awake with UART2 open, waiting for bytes from the Pi. 06 = default
// sdkconfig (160 MHz, V0/V1), 07 = V2 menuconfig (README).

#include <stdint.h>

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

    // same UART setup as link_open() in the firmware
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
    uart_flush_input(PI_UART);

    vTaskDelay(pdMS_TO_TICKS(2000));

    // same call as link_read_line(): wait for one byte with a timeout
    while (1) {
        uint8_t c;
        uart_read_bytes(PI_UART, &c, 1, pdMS_TO_TICKS(5000));
    }
}
