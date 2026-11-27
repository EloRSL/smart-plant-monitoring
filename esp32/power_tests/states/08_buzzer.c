// power test 08: buzzer 5 s on, 5 s off; the difference between the two levels is the
// buzzer. Wiring: buzzer on GPIO27.

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUZZER_GPIO  GPIO_NUM_27
#define MARKER_GPIO  GPIO_NUM_32

void app_main(void)
{
    gpio_reset_pin(BUZZER_GPIO);
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MARKER_GPIO);
    gpio_set_direction(MARKER_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(MARKER_GPIO, 1);
        gpio_set_level(BUZZER_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(5000));
        gpio_set_level(BUZZER_GPIO, 0);
        gpio_set_level(MARKER_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
