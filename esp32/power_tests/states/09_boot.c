// power test 09: boot from deep sleep every 2 s with an empty app, so each peak is one
// boot. 09a = default sdkconfig, 09b = V2 menuconfig (80 MHz, quiet log and
// bootloader).

#include "esp_sleep.h"

#define SLEEP_S  2

void app_main(void)
{
    // nothing else: go straight back to sleep, so each peak is one boot
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_S * 1000000);
    esp_deep_sleep_start();
}
