# Power tests

One ESP-IDF project for all the power measurements. Every file in `states/` puts the
ESP32 in one state from the firmware, so we can measure only that state (Monsoon HVPM,
5 kHz, board powered over USB). To run a test copy the file to `main/main.c`, build and
flash. By default `main/main.c` is test 01.

| test | file | state | used in |
| --- | --- | --- | --- |
| 01 | `01_deep_sleep_sensors_on.c` | deep sleep, sensors on 3V3, pins not held | V1 |
| 02 | `02_deep_sleep_sensors_off.c` | deep sleep, sensors off, pins held | V2, V3 |
| 03 | `03_awake_idle.c` | awake, UART2 installed | V0 |
| 04 | `04_measure.c` | one measurement every 3 s, sensors always on | V0, V1 |
| 05 | `05_measure_switched_sensors.c` | sensors on, 1 s warm-up, read, sensors off | V2, V3 |
| 06 / 07 | `06_wait_for_pi.c` | awake, waiting for the Pi on UART2 | 06: V0/V1, 07: V2/V3 |
| 08 | `08_buzzer.c` | buzzer 5 s on, 5 s off | all |
| 09a / 09b | `09_boot.c` | boot from deep sleep with an empty app | 09a: V1, 09b: V2/V3 |
| 10 | `10_light_sleep.c` | light sleep, sensors off and on | only tried, not used |

Setup:

- tests 01, 03, 04, 06, 08, 09a: sensor VCC on 3V3, default menuconfig (160 MHz, log Info)
- tests 02, 05, 07, 09b, 10: soil VCC on GPIO4, DHT11 VCC on GPIO33, and the same
  menuconfig as V2 (CPU 80 MHz, log level Warning, bootloader log Warning), see
  `../firmware/v2_lowpower/sdkconfig.defaults`

We exported every capture from PowerTool as csv and named it after the test, for example
`S04.csv`, `S04_80.csv` (80 MHz) or `S01_voda.csv` (probe in water). `analysis/` reads
these files.
