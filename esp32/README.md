# ESP32 part

This is the ESP32 side of the plant monitor (the Raspberry Pi part is in `pi/`).
The ESP32 reads the sensors, checks the rules from `configuration.txt`, beeps,
and wakes the Pi when it needs to send data.

## Folders

| folder | what's in it |
| --- | --- |
| `firmware/v0_baseline` | V0: loop with vTaskDelay, never sleeps, Pi always on |
| `firmware/v1_deepsleep` | V1: deep sleep between readings, readings saved in RTC memory, Pi woken only for sessions |
| `firmware/v2_lowpower` | V2: V1 + sensors powered from GPIO, pins held in sleep, quiet log, 80 MHz |
| `firmware/v3_adaptive` | V3: V2 + interval changes between 5/20/30 min, wakes the Pi when a rule is crossed |
| `power_tests` | small programs, one for each state we measured with the Monsoon |
| `analysis` | python script that gets the numbers out of the Monsoon csv files |

## Building

Every version is its own ESP-IDF project (we used ESP-IDF 6.1). Open the folder in
VS Code with the ESP-IDF extension, or:

```
cd firmware/v3_adaptive
idf.py build flash monitor
```

The DHT11 driver (`esp-idf-lib/dht`) and cJSON are downloaded by the component manager
(`main/idf_component.yml`). For V2 and V3 the CPU speed and log level come from
`sdkconfig.defaults`, so if you already have an old `sdkconfig` delete it once.

`DEMO_MODE` at the top of `main.c` is 0 (real intervals: reading every 20 min, Pi once
a day). Set it to 1 for testing, then it reads every 30 s (V3: 10/30/60 s).

## Wiring

| what | ESP32 pin |
| --- | --- |
| DHT11 data | GPIO26 |
| soil sensor (analog) | GPIO34 (ADC1 CH6) |
| buzzer | GPIO27 |
| wake line to Pi GPIO3 (pin 5) | GPIO25 (open drain) |
| UART to Pi | TX2 = GPIO17 -> Pi pin 10, RX2 = GPIO16 -> Pi pin 8 |
| marker for power measurements | GPIO32 (high while awake) |
| manual session | BOOT button (GPIO0) |
| sensor power (V2, V3 only) | soil VCC on GPIO4, DHT11 VCC on GPIO33 |

In V0 and V1 the sensors are on 3V3. Don't forget common GND with the Pi.

## Talking to the Pi

UART 115200 8N1, one JSON object per line. The Pi sends `ready`, the ESP32 sends its
`alarm` lines and then the readings in `batch` messages (max 50 per batch). Each batch
is deleted only after the Pi sends `ack` with the same `seq`. After that the Pi sends
`time`, `rules`, maybe `config` or `buzz`, and at the end `bye`.

## Analysis

```
python analysis/analyze_states.py --data <folder with S*.csv> --out results.json
```

The raw Monsoon captures are too big for git (about 20 MB each), we can send them if
needed.
