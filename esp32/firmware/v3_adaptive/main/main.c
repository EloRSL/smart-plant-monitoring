// Smart plant monitoring - ESP32 part, version V3
// V2 + adaptive interval (5 / 20 / 30 min depending on how fast values change)
// and it wakes the Pi right away when a rule gets crossed.
// PECD 2026, Vid Kramar

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "cJSON.h"
#include "dht.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ----- V2 switches: 1 = on, 0 = behave like V1 -----
// CPU 80 MHz and quiet boot log are set in sdkconfig.defaults.

#define V2_SWITCHED_SENSOR_POWER  1   // sensor VCC from GPIO4/GPIO33, on only while measuring
#define V2_HOLD_PINS_IN_SLEEP     1   // outputs held LOW in deep sleep: no leakage, no buzzer click
#define V2_QUIET_LOG              1   // log level Warning: printing costs awake time
#define V2_TRIM_MEASUREMENT       1   // fewer ADC reads, 1 DHT retry, shorter warm-up

// version string sent to the Pi, e.g. "v2-p1h1q1t1" = which switches were on
#define STR_(x) #x
#define STR(x)  STR_(x)
#define FW_VERSION "v3-p" STR(V2_SWITCHED_SENSOR_POWER) "h" STR(V2_HOLD_PINS_IN_SLEEP) \
                   "q" STR(V2_QUIET_LOG) "t" STR(V2_TRIM_MEASUREMENT)

// ----- settings -----

// 1 = short intervals for testing/demo, 0 = real intervals
#define DEMO_MODE 0

#if DEMO_MODE
#define INTERVAL_FAST_S      10          // something is happening
#define INTERVAL_NORMAL_S    30          // default
#define INTERVAL_MAX_S       60          // conditions stable
#define PI_WAKE_INTERVAL_S   (5 * 60)    // "daily" Pi wake-up
#define RETRY_AFTER_FAIL_S   (2 * 60)    // wait this long after a failed session
#else
#define INTERVAL_FAST_S      (5 * 60)
#define INTERVAL_NORMAL_S    (20 * 60)
#define INTERVAL_MAX_S       (30 * 60)
#define PI_WAKE_INTERVAL_S   (24 * 60 * 60)
#define RETRY_AFTER_FAIL_S   (60 * 60)
#endif

// adaptive scheduling (V3)
#define STABLE_TO_NORMAL     3           // FAST -> NORMAL after 3 stable readings
#define STABLE_TO_MAX        6           // NORMAL -> MAX after 6 stable readings
// "Large change" since the previous reading -> FAST
#define DELTA_AIR_TEMPERATURE  2         // C
#define DELTA_AIR_HUMIDITY     5         // %RH
#define DELTA_SOIL_HUMIDITY    50        // units of 0..1000
// "Close to a threshold" -> FAST
#define NEAR_AIR_TEMPERATURE   2
#define NEAR_AIR_HUMIDITY      5
#define NEAR_SOIL_HUMIDITY     50

// the "daily" Pi wake-up happens in daylight, so the photo shows the plant: the first
// wake-up between DAY_START_HOUR and DAY_END_HOUR (local time) on a day without a
// daytime session yet. This needs the real time from the Pi; before that, and in demo
// mode, PI_WAKE_INTERVAL_S is used instead.
#define DAY_START_HOUR       12
#define DAY_END_HOUR         16
#define LOCAL_TZ             "CET-1CEST,M3.5.0,M10.5.0/3"   // Slovenia, with summer time

// pins
#define WAKE_PIN          GPIO_NUM_25    // to Pi pin 5 (GPIO3), open drain
#define PI_UART           UART_NUM_2
#define PI_TX_PIN         GPIO_NUM_17    // board label TX2, to Pi pin 10
#define PI_RX_PIN         GPIO_NUM_16    // board label RX2, to Pi pin 8
#define DHT_GPIO          GPIO_NUM_26
#define BUZZER_GPIO       GPIO_NUM_27
#define MOISTURE_CHANNEL  ADC_CHANNEL_6  // ADC1 CH6 = GPIO34
#define MARKER_GPIO       GPIO_NUM_32    // HIGH while awake, for measurements
#define BUTTON_GPIO       GPIO_NUM_0     // BOOT button, LOW when pressed
#define SOIL_VCC_GPIO     GPIO_NUM_4     // V2: powers the soil sensor
#define DHT_VCC_GPIO      GPIO_NUM_33    // V2: powers the DHT11 module

// soil calibration, raw ADC 0..4095. Resistive probe: raw rises with moisture.
// soil_humidity 0 = probe in air, 1000 = probe in water.
#define SOIL_RAW_AIR      0
#define SOIL_RAW_WATER    2723
#if V2_TRIM_MEASUREMENT
#define SOIL_SAMPLES      4              // fewer reads: measured stable enough
#else
#define SOIL_SAMPLES      16             // ADC reads averaged per measurement
#endif

// DHT11
#define DHT_STARTUP_DELAY_MS  2000       // DHT11 is not ready right after power-on
#if V2_TRIM_MEASUREMENT
#define DHT_RETRIES           1          // extra attempts after a failed read
#define SENSOR_WARMUP_MS      1000       // DHT11 datasheet: ~1 s after power-on before the first read
#else
#define DHT_RETRIES           2
#define SENSOR_WARMUP_MS      DHT_STARTUP_DELAY_MS
#endif
#define DHT_RETRY_DELAY_MS    1100       // DHT11 needs >= 1 s between reads

// buzzer
#define BEEP_ON_MS        200
#define BEEP_OFF_MS       300
#define BEEP_GROUP_GAP_MS 1000           // pause between two different alarms
#define REMINDER_EVERY    6              // re-beep every N readings while a rule stays true

// hysteresis: an active alarm clears only when the value is this far back
#define HYST_AIR_TEMPERATURE  1
#define HYST_AIR_HUMIDITY     2
#define HYST_SOIL_HUMIDITY    20

// protocol timing
#define READY_TIMEOUT_MS  90000          // Pi boot + service start
#define ACK_TIMEOUT_MS    5000
#define ACK_RETRIES       3
#define BYE_TIMEOUT_MS    30000          // time, rules, config, buzz, then bye
#define CHUNK_SIZE        50             // max readings per batch message

// buffers
#define BUF_CAPACITY      400            // readings in RTC memory (8 B each)
#define BUF_FULL_LEVEL    (BUF_CAPACITY * 8 / 10)   // 80 % -> "full"
#define MAX_RULES         10
#define MAX_ALARMS        16
#define JSON_LINE_MAX          1536           // longest JSON line we accept
#define UART_RX_BUF       2048

// ----- types -----

static const char *TAG = "plant";

// sensor names are the ones from the project spec / configuration.txt
typedef enum {
    SENSOR_AIR_TEMPERATURE = 0,
    SENSOR_AIR_HUMIDITY,
    SENSOR_SOIL_HUMIDITY,
    SENSOR_COUNT
} sensor_t;

static const char *const SENSOR_NAMES[SENSOR_COUNT] = {
    "air_temperature", "air_humidity", "soil_humidity"
};
static const int SENSOR_HYST[SENSOR_COUNT] = {
    HYST_AIR_TEMPERATURE, HYST_AIR_HUMIDITY, HYST_SOIL_HUMIDITY
};
static const int SENSOR_DELTA[SENSOR_COUNT] = {
    DELTA_AIR_TEMPERATURE, DELTA_AIR_HUMIDITY, DELTA_SOIL_HUMIDITY
};
static const int SENSOR_NEAR[SENSOR_COUNT] = {
    NEAR_AIR_TEMPERATURE, NEAR_AIR_HUMIDITY, NEAR_SOIL_HUMIDITY
};

// measurement speed (V3)
typedef enum { SPEED_FAST = 0, SPEED_NORMAL, SPEED_MAX } sched_speed_t;
static const char *const SPEED_NAMES[] = { "fast", "normal", "max" };

typedef enum { OP_LT = 0, OP_GT, OP_LE, OP_GE, OP_COUNT } op_t;
static const char *const OP_NAMES[OP_COUNT] = { "<", ">", "<=", ">=" };

// one alarm rule, e.g. "3: soil_humidity < 350" -> beep 3 times
typedef struct {
    uint8_t buzz;      // number of beeps
    uint8_t sensor;    // sensor_t
    uint8_t op;        // op_t
    int16_t value;
} rule_t;

// default rules, used until the Pi sends its configuration.txt
static const rule_t DEFAULT_RULES[] = {
    { 3, SENSOR_SOIL_HUMIDITY,   OP_LT, 350 },
    { 2, SENSOR_AIR_TEMPERATURE, OP_GT, 25  },
    { 2, SENSOR_AIR_TEMPERATURE, OP_LT, 8   },
    { 1, SENSOR_SOIL_HUMIDITY,   OP_GT, 900 },
};

// one measurement, 8 bytes. Invalid values mean "sensor failed".
#define TEMP_INVALID  INT8_MIN
#define HUM_INVALID   0xFF
#define SOIL_INVALID  0xFFFF
typedef struct {
    uint32_t t;        // ESP clock in seconds (unix time once synced)
    int8_t   temp;     // air_temperature, C
    uint8_t  hum;      // air_humidity, %RH
    uint16_t soil;     // soil_humidity, 0..1000
} reading_t;
_Static_assert(sizeof(reading_t) == 8, "reading_t must stay 8 bytes");

// one beep event, reported to the Pi as an "alarm" message
typedef struct {
    uint32_t t;
    uint8_t  buzz;
    uint8_t  sensor;
    int16_t  value;
} alarm_t;

// everything that must survive deep sleep
#define STATE_MAGIC 0x504C4E54u   // "PLNT"
typedef struct {
    uint32_t magic;               // wrong after power-on -> initialise
    uint32_t wakes;               // number of wake-ups (measurements)
    uint64_t awake_us_total;      // sum of awake time of finished wake-ups
    uint32_t awake_count;         // number of finished wake-ups in that sum
    uint32_t seq;                 // next batch sequence number
    uint32_t dropped;             // readings lost because the buffer was full
    bool     time_synced;         // true after the Pi sent the time
    uint32_t last_session_t;      // ESP clock of the last successful Pi session
    uint32_t last_attempt_t;      // ESP clock of the last attempt (success or not)
    uint32_t last_daytime_t;      // last successful session during daylight (photo taken)
    uint32_t interval_s;          // current sleep interval (from the speed)
    uint32_t normal_interval_s;   // NORMAL interval (Pi can change it)
    uint32_t pi_wake_interval_s;  // "daily" interval (Pi can change it)

    uint8_t  speed;               // sched_speed_t: FAST, NORMAL or MAX
    uint8_t  stable_count;        // stable readings in a row at this speed
    uint32_t interval_changes;    // how often the speed changed, for stats
    bool     change_pending;      // a threshold was newly crossed: wake the Pi
    bool     have_prev;           // prev holds a reading
    reading_t prev;               // previous reading, for "large change"

    uint8_t  rule_count;
    rule_t   rules[MAX_RULES];
    bool     rule_active[MAX_RULES];     // condition currently true
    uint8_t  since_beep[MAX_RULES];      // readings since the last beep

    uint8_t  alarm_count;
    alarm_t  alarms[MAX_ALARMS];         // beeps not yet reported to the Pi

    uint16_t head;                       // ring buffer: oldest reading
    uint16_t count;
    reading_t buf[BUF_CAPACITY];
} node_state_t;

// RTC slow memory: keeps its content through deep sleep
RTC_DATA_ATTR static node_state_t s;

static char s_line[JSON_LINE_MAX];

// ----- small helpers, each used in several places -----

static void beep(int on_ms, int off_ms, int times)
{
    for (int i = 0; i < times; ++i) {
        ESP_ERROR_CHECK(gpio_set_level(BUZZER_GPIO, 1));
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        ESP_ERROR_CHECK(gpio_set_level(BUZZER_GPIO, 0));
        if (off_ms > 0) vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

// value of one sensor in a reading; false if that sensor failed
static bool reading_value(const reading_t *r, uint8_t sensor, int *out)
{
    switch (sensor) {
    case SENSOR_AIR_TEMPERATURE: *out = r->temp; return r->temp != TEMP_INVALID;
    case SENSOR_AIR_HUMIDITY:    *out = r->hum;  return r->hum != HUM_INVALID;
    case SENSOR_SOIL_HUMIDITY:   *out = r->soil; return r->soil != SOIL_INVALID;
    default:                     return false;
    }
}

static int find_name(const char *name, const char *const *names, int count)
{
    for (int i = 0; i < count; ++i) {
        if (strcmp(name, names[i]) == 0) return i;
    }
    return -1;
}

// read one '\n'-terminated line from the Pi into s_line. Returns its length, or -1 on
// timeout.
static int link_read_line(int64_t deadline_us)
{
    size_t n = 0;
    bool overflow = false;
    while (true) {
        int64_t left_ms = (deadline_us - esp_timer_get_time()) / 1000;
        if (left_ms <= 0) return -1;
        TickType_t ticks = pdMS_TO_TICKS(left_ms);
        if (ticks == 0) ticks = 1;

        uint8_t c;
        if (uart_read_bytes(PI_UART, &c, 1, ticks) <= 0) continue;
        if (c == '\r') continue;
        if (c == '\n') {
            if (overflow) {
                ESP_LOGW(TAG, "line too long, ignored");
                n = 0;
                overflow = false;
                continue;
            }
            if (n == 0) continue;
            s_line[n] = '\0';
            return (int)n;
        }
        if (n < sizeof(s_line) - 1) s_line[n++] = (char)c;
        else overflow = true;
    }
}

// send a JSON object to the Pi as one line, then free it
static void link_send(cJSON *obj)
{
    char *text = cJSON_PrintUnformatted(obj);
    if (text != NULL) {
        uart_write_bytes(PI_UART, text, strlen(text));
        uart_write_bytes(PI_UART, "\n", 1);
        ESP_LOGI(TAG, "-> %s", text);
        cJSON_free(text);
    }
    cJSON_Delete(obj);
}

static const char *msg_type(const cJSON *msg)
{
    const cJSON *type = cJSON_GetObjectItemCaseSensitive(msg, "type");
    return cJSON_IsString(type) ? type->valuestring : "";
}

// ----- main: runs once after every wake-up, from top to bottom -----

void app_main(void)
{
    // ----- 1. start-up: pins, log level, time zone -----

#if V2_HOLD_PINS_IN_SLEEP
    // release the pins held during sleep, otherwise we cannot change them
    gpio_hold_dis(BUZZER_GPIO);
    gpio_hold_dis(MARKER_GPIO);
#if V2_SWITCHED_SENSOR_POWER
    gpio_hold_dis(SOIL_VCC_GPIO);
    gpio_hold_dis(DHT_VCC_GPIO);
#endif
#endif
#if V2_QUIET_LOG
    // only warnings and errors from here on. Boot messages before app_main are silenced
    // by the log levels in sdkconfig.defaults.
    esp_log_level_set("*", ESP_LOG_WARN);
#endif

    // local time zone for the daily wake-up (environment is lost in deep sleep)
    setenv("TZ", LOCAL_TZ, 1);
    tzset();

    // marker pin: HIGH while awake, for the power measurements
    gpio_reset_pin(MARKER_GPIO);
    gpio_set_direction(MARKER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MARKER_GPIO, 1);

    // buzzer pin: output, off
    ESP_ERROR_CHECK(gpio_reset_pin(BUZZER_GPIO));
    ESP_ERROR_CHECK(gpio_set_level(BUZZER_GPIO, 0));
    ESP_ERROR_CHECK(gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT));

    // soil moisture: ADC1, 12 dB attenuation (full 0..3.3 V range), 12 bit
    adc_oneshot_unit_handle_t adc;
    adc_oneshot_unit_init_cfg_t adc_unit = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_unit, &adc));
    adc_oneshot_chan_cfg_t adc_channel = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, MOISTURE_CHANNEL, &adc_channel));

    // DHT11 data pin: the module has its own pull-up
    ESP_ERROR_CHECK(gpio_set_pull_mode(DHT_GPIO, GPIO_FLOATING));

#if V2_SWITCHED_SENSOR_POWER
    // sensor power pins: outputs, OFF until the measurement needs them
    gpio_reset_pin(SOIL_VCC_GPIO);
    gpio_reset_pin(DHT_VCC_GPIO);
    gpio_set_direction(SOIL_VCC_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(DHT_VCC_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(SOIL_VCC_GPIO, 0);
    gpio_set_level(DHT_VCC_GPIO, 0);
#endif

    // wake line to the Pi, open drain: 1 = released, 0 = pulls the Pi wake pin to ground
    gpio_set_level(WAKE_PIN, 1);
    gpio_config_t wake_config = {
        .pin_bit_mask = 1ULL << WAKE_PIN,
        .mode = GPIO_MODE_OUTPUT_OD,
    };
    gpio_config(&wake_config);

    // ----- 2. why did we start? -----

    // 0 = power-on or reset
    const uint32_t causes = esp_sleep_get_wakeup_causes();
    bool button = (causes & BIT(ESP_SLEEP_WAKEUP_EXT0)) != 0;
    bool cold = (causes == 0) || (s.magic != STATE_MAGIC);

    // power-on or reset: the clock restarts at 0, so old timestamps in RTC memory would
    // be wrong. Start clean (default rules until the Pi sends some).
    if (cold) {
        memset(&s, 0, sizeof(s));
        s.magic = STATE_MAGIC;
        s.normal_interval_s = INTERVAL_NORMAL_S;
        s.speed = SPEED_NORMAL;
        s.interval_s = s.normal_interval_s;
        s.pi_wake_interval_s = PI_WAKE_INTERVAL_S;
        s.rule_count = sizeof(DEFAULT_RULES) / sizeof(DEFAULT_RULES[0]);
        memcpy(s.rules, DEFAULT_RULES, sizeof(DEFAULT_RULES));
        ESP_LOGI(TAG, "state initialised, %u default rules", (unsigned)s.rule_count);
    }
    s.wakes++;

#if !V2_SWITCHED_SENSOR_POWER
    // after power-on the DHT11 needs time; after deep sleep it stayed powered. (With
    // switched power, the measurement does its own warm-up every time.)
    if (cold) vTaskDelay(pdMS_TO_TICKS(DHT_STARTUP_DELAY_MS));
#endif

    // ----- 3. measure -----

    reading_t r = {
        .t = (uint32_t)time(NULL), .temp = TEMP_INVALID, .hum = HUM_INVALID, .soil = SOIL_INVALID,
    };

#if V2_SWITCHED_SENSOR_POWER
    // sensors were unpowered during sleep: switch on and let them settle. The DHT11 is
    // the slow one; the soil sensor is ready long before.
    gpio_set_level(SOIL_VCC_GPIO, 1);
    gpio_set_level(DHT_VCC_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(SENSOR_WARMUP_MS));
#endif

    // air temperature and humidity, with retries
    float hum, temp;
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt <= DHT_RETRIES; ++attempt) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(DHT_RETRY_DELAY_MS));
        err = dht_read_float_data(DHT_TYPE_DHT11, DHT_GPIO, &hum, &temp);
        if (err == ESP_OK) break;
    }
    if (err == ESP_OK) {
        r.temp = (int8_t)(temp >= 0 ? temp + 0.5f : temp - 0.5f);
        r.hum = (uint8_t)(hum + 0.5f);
    } else {
        ESP_LOGW(TAG, "DHT11 read failed: %s", esp_err_to_name(err));
    }

    // soil moisture: average of SOIL_SAMPLES ADC reads, then raw ADC -> 0 (air) .. 1000
    // (water), linear between the two calibration points
    int raw = -1;
    int soil_sum = 0, soil_ok = 0;
    for (int i = 0; i < SOIL_SAMPLES; ++i) {
        int sample;
        if (adc_oneshot_read(adc, MOISTURE_CHANNEL, &sample) == ESP_OK) {
            soil_sum += sample;
            soil_ok++;
        }
    }
    if (soil_ok > 0) {
        raw = soil_sum / soil_ok;
        int scaled = (SOIL_RAW_AIR - raw) * 1000 / (SOIL_RAW_AIR - SOIL_RAW_WATER);
        if (scaled < 0) scaled = 0;
        if (scaled > 1000) scaled = 1000;
        r.soil = (uint16_t)scaled;
    } else {
        ESP_LOGW(TAG, "soil ADC read failed");
    }

#if V2_SWITCHED_SENSOR_POWER
    // off again. Data pin as input, so it does not feed the unpowered DHT.
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);
    gpio_set_level(SOIL_VCC_GPIO, 0);
    gpio_set_level(DHT_VCC_GPIO, 0);
#endif

    ESP_LOGI(TAG, "measured: air_temperature=%d air_humidity=%u soil_humidity=%u (raw %d)",
             r.temp, (unsigned)r.hum, (unsigned)r.soil, raw);

    // ----- 4. store the reading in the ring buffer -----

    if (s.count == BUF_CAPACITY) {       // full: overwrite the oldest
        s.head = (s.head + 1) % BUF_CAPACITY;
        s.count--;
        s.dropped++;
    }
    s.buf[(s.head + s.count) % BUF_CAPACITY] = r;
    s.count++;

    // ----- 5. check the rules and beep -----
    // beep when a rule BECOMES true, then again every REMINDER_EVERY readings while it
    // stays true (not on every reading, or it beeps all night). Hysteresis: once
    // active, the threshold moves by SENSOR_HYST so a value hovering at the threshold
    // does not switch the alarm on and off.
    bool beeped = false;
    for (int i = 0; i < s.rule_count; ++i) {
        const rule_t *rule = &s.rules[i];
        int v;
        if (!reading_value(&r, rule->sensor, &v)) continue;   // sensor failed: keep state

        int threshold = rule->value;
        if (s.rule_active[i]) {
            if (rule->op == OP_LT || rule->op == OP_LE) threshold += SENSOR_HYST[rule->sensor];
            else                                         threshold -= SENSOR_HYST[rule->sensor];
        }
        bool cond;
        switch (rule->op) {
        case OP_LT: cond = v < threshold;  break;
        case OP_GT: cond = v > threshold;  break;
        case OP_LE: cond = v <= threshold; break;
        case OP_GE: cond = v >= threshold; break;
        default:    cond = false;          break;
        }

        bool do_beep = false;
        if (cond && !s.rule_active[i]) {
            ESP_LOGI(TAG, "ALARM: %u: %s %s %d (value %d)", (unsigned)rule->buzz,
                     SENSOR_NAMES[rule->sensor], OP_NAMES[rule->op], rule->value, v);
            do_beep = true;
            s.change_pending = true;     // V3: tell the Pi now ("change")
        } else if (cond && ++s.since_beep[i] >= REMINDER_EVERY) {
            ESP_LOGI(TAG, "reminder: %s still %d", SENSOR_NAMES[rule->sensor], v);
            do_beep = true;
        } else if (!cond && s.rule_active[i]) {
            ESP_LOGI(TAG, "cleared: %s back to %d", SENSOR_NAMES[rule->sensor], v);
        }
        s.rule_active[i] = cond;

        if (do_beep) {
            if (beeped) vTaskDelay(pdMS_TO_TICKS(BEEP_GROUP_GAP_MS));
            beep(BEEP_ON_MS, BEEP_OFF_MS, rule->buzz);
            beeped = true;
            s.since_beep[i] = 0;

            // remember the beep so it can be reported to the Pi (events_log.txt)
            if (s.alarm_count == MAX_ALARMS) {   // full: drop the oldest
                memmove(&s.alarms[0], &s.alarms[1], sizeof(alarm_t) * (MAX_ALARMS - 1));
                s.alarm_count--;
            }
            s.alarms[s.alarm_count++] = (alarm_t){
                .t = (uint32_t)time(NULL), .buzz = rule->buzz, .sensor = rule->sensor, .value = (int16_t)v,
            };
        }
    }

    // ----- 6. V3: pick the next interval -----
    // FAST when something is happening, then step back to NORMAL and MAX only after
    // several stable readings in a row

    bool busy = false;

    // something is happening: any sensor changed by at least its DELTA since the previous reading
    if (s.have_prev) {
        for (int sn = 0; sn < SENSOR_COUNT; ++sn) {
            int now_v, prev_v;
            if (!reading_value(&r, (uint8_t)sn, &now_v) || !reading_value(&s.prev, (uint8_t)sn, &prev_v)) {
                continue;
            }
            int diff = now_v > prev_v ? now_v - prev_v : prev_v - now_v;
            if (diff >= SENSOR_DELTA[sn]) {
                busy = true;
                break;
            }
        }
    }

    // ... or a value within NEAR of a rule threshold, or a rule currently violated
    for (int i = 0; i < s.rule_count && !busy; ++i) {
        if (s.rule_active[i]) {
            busy = true;
            break;
        }
        int v;
        if (!reading_value(&r, s.rules[i].sensor, &v)) continue;
        int diff = v > s.rules[i].value ? v - s.rules[i].value : s.rules[i].value - v;
        if (diff <= SENSOR_NEAR[s.rules[i].sensor]) busy = true;
    }

    uint8_t old_speed = s.speed;
    if (busy) {
        s.speed = SPEED_FAST;
        s.stable_count = 0;
    } else if (s.speed != SPEED_MAX) {
        s.stable_count++;
        if (s.speed == SPEED_FAST && s.stable_count >= STABLE_TO_NORMAL) {
            s.speed = SPEED_NORMAL;
            s.stable_count = 0;
        } else if (s.speed == SPEED_NORMAL && s.stable_count >= STABLE_TO_MAX) {
            s.speed = SPEED_MAX;
            s.stable_count = 0;
        }
    }
    if (s.speed != old_speed) {
        s.interval_changes++;
        ESP_LOGI(TAG, "speed %s -> %s", SPEED_NAMES[old_speed], SPEED_NAMES[s.speed]);
    }
    s.interval_s = s.speed == SPEED_FAST ? INTERVAL_FAST_S
                 : s.speed == SPEED_MAX  ? INTERVAL_MAX_S
                 : s.normal_interval_s;
    s.prev = r;
    s.have_prev = true;

    // ----- 7. do we need the Pi? -----

    uint32_t now = (uint32_t)time(NULL);
    const char *reason = NULL;
    if (button || cold) {
        reason = "manual";               // button, or first start: get time + rules
    } else if (s.count >= BUF_FULL_LEVEL) {
        reason = "full";
    } else if (s.change_pending) {
        reason = "change";               // V3: a threshold was newly crossed
    } else {
        // the regular ("daily") wake-up
        bool daily_due;
#if !DEMO_MODE
        if (s.time_synced) {
            // real time known: once per day, in daylight
            // (DAY_START_HOUR..DAY_END_HOUR), so the photo shows the plant
            time_t a = (time_t)now;
            struct tm today;
            localtime_r(&a, &today);
            if (today.tm_hour < DAY_START_HOUR || today.tm_hour >= DAY_END_HOUR) {
                daily_due = false;
            } else if (s.last_daytime_t == 0) {
                daily_due = true;
            } else {
                time_t b = (time_t)s.last_daytime_t;
                struct tm last;
                localtime_r(&b, &last);
                daily_due = today.tm_yday != last.tm_yday || today.tm_year != last.tm_year;
            }
        } else
#endif
        {
            // demo mode, or no real time yet: fixed interval since the last session
            daily_due = now - s.last_session_t >= s.pi_wake_interval_s;
        }
        if (daily_due) reason = "daily";
    }

    // after a FAILED attempt, wait RETRY_AFTER_FAIL_S before trying again, so a broken
    // Pi does not keep us awake 90 s on every wake-up
    bool last_failed = s.last_attempt_t != s.last_session_t;
    bool retry_ok = !last_failed || now - s.last_attempt_t >= RETRY_AFTER_FAIL_S;

    // ----- 8. session with the Pi -----
    //   a. wake the Pi and open the UART
    //   b. wait for {"type":"ready"}       (the ESP sends nothing before it)
    //   c. send pending alarms
    //   d. send readings in chunks of CHUNK_SIZE; delete each chunk only after its ack
    //   e. handle time / rules / config / buzz until "bye"
    //   f. close the UART
    if (reason != NULL && (button || cold || retry_ok)) {
        ESP_LOGI(TAG, "Pi session, reason %s, %u readings", reason, (unsigned)s.count);
        s.last_attempt_t = now;

        // a. Wake pulse on the Pi's wake pin, then UART2 at 115200 8N1
        gpio_set_level(WAKE_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_set_level(WAKE_PIN, 1);

        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
        };
        ESP_ERROR_CHECK(uart_driver_install(PI_UART, UART_RX_BUF, 0, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(PI_UART, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(PI_UART, PI_TX_PIN, PI_RX_PIN,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        uart_flush_input(PI_UART);

        // ok = true once every reading was acknowledged. Any failure leaves the block
        // with "break" and keeps the readings for the next session.
        bool ok = false;
        do {
            // b. Wait for "ready"; other messages are ignored
            bool ready = false;
            int64_t ready_deadline = esp_timer_get_time() + (int64_t)READY_TIMEOUT_MS * 1000;
            while (!ready && link_read_line(ready_deadline) >= 0) {
                cJSON *msg = cJSON_Parse(s_line);
                if (msg != NULL && strcmp(msg_type(msg), "ready") == 0) ready = true;
                cJSON_Delete(msg);               // e.g. a repeated "ready"
            }
            if (!ready) {
                ESP_LOGW(TAG, "Pi not ready, keeping %u readings", (unsigned)s.count);
                break;
            }

            // c. Alarms first. They are deleted once the first ack proves the Pi listens.
            for (uint8_t i = 0; i < s.alarm_count; ++i) {
                cJSON *a = cJSON_CreateObject();
                cJSON_AddStringToObject(a, "type", "alarm");
                cJSON_AddNumberToObject(a, "t", s.alarms[i].t);
                cJSON_AddNumberToObject(a, "buzz", s.alarms[i].buzz);
                cJSON_AddStringToObject(a, "sensor", SENSOR_NAMES[s.alarms[i].sensor]);
                cJSON_AddNumberToObject(a, "value", s.alarms[i].value);
                link_send(a);
            }

            // d. Readings in chunks, each one deleted only after its ack
            bool first_chunk = true;
            bool aborted = false;
            do {
                uint16_t n = s.count < CHUNK_SIZE ? s.count : CHUNK_SIZE;
                bool last = (n == s.count);
                uint32_t seq = s.seq;

                bool acked = false;
                for (int attempt = 0; attempt <= ACK_RETRIES && !acked; ++attempt) {
                    cJSON *batch = cJSON_CreateObject();
                    cJSON_AddStringToObject(batch, "type", "batch");
                    cJSON_AddNumberToObject(batch, "seq", seq);
                    cJSON_AddStringToObject(batch, "reason", reason);
                    cJSON_AddNumberToObject(batch, "now", (uint32_t)time(NULL));
                    cJSON_AddBoolToObject(batch, "synced", s.time_synced);
                    cJSON *list = cJSON_AddArrayToObject(batch, "readings");
                    for (uint16_t i = 0; i < n; ++i) {
                        const reading_t *rd = &s.buf[(s.head + i) % BUF_CAPACITY];
                        cJSON *o = cJSON_CreateObject();
                        cJSON_AddNumberToObject(o, "t", rd->t);
                        if (rd->temp != TEMP_INVALID) cJSON_AddNumberToObject(o, "air_temperature", rd->temp);
                        if (rd->hum != HUM_INVALID)   cJSON_AddNumberToObject(o, "air_humidity", rd->hum);
                        if (rd->soil != SOIL_INVALID) cJSON_AddNumberToObject(o, "soil_humidity", rd->soil);
                        cJSON_AddItemToArray(list, o);
                    }
                    cJSON_AddBoolToObject(batch, "last", last);
                    if (last) {
                        // statistics for the Pi, with the last chunk
                        cJSON *stats = cJSON_AddObjectToObject(batch, "stats");
                        cJSON_AddNumberToObject(stats, "wakes", s.wakes);
                        cJSON_AddNumberToObject(stats, "avg_awake_ms",
                                                s.awake_count ? (double)(s.awake_us_total / s.awake_count / 1000) : 0);
                        cJSON_AddNumberToObject(stats, "dropped", s.dropped);
                        cJSON_AddNumberToObject(stats, "interval_s", s.interval_s);
                        cJSON_AddNumberToObject(stats, "interval_changes", s.interval_changes);
                        cJSON_AddStringToObject(stats, "fw", FW_VERSION);
                    }
                    link_send(batch);

                    // wait for the ack with OUR seq; ignore other messages
                    int64_t ack_deadline = esp_timer_get_time() + (int64_t)ACK_TIMEOUT_MS * 1000;
                    while (!acked && link_read_line(ack_deadline) >= 0) {
                        cJSON *reply = cJSON_Parse(s_line);
                        const cJSON *rs = cJSON_GetObjectItemCaseSensitive(reply, "seq");
                        if (strcmp(msg_type(reply), "ack") == 0 && cJSON_IsNumber(rs) &&
                            (uint32_t)rs->valuedouble == seq) {
                            acked = true;
                        }
                        cJSON_Delete(reply);
                    }
                    if (!acked) ESP_LOGW(TAG, "no ack for seq %" PRIu32 " (attempt %d)", seq, attempt + 1);
                }
                if (!acked) {
                    ESP_LOGW(TAG, "session aborted, keeping %u readings", (unsigned)s.count);
                    aborted = true;
                    break;
                }

                if (first_chunk) {
                    s.alarm_count = 0;
                    first_chunk = false;
                }
                // acknowledged: delete the chunk from the ring buffer
                if (n > s.count) n = s.count;
                s.head = (s.head + n) % BUF_CAPACITY;
                s.count -= n;
                s.seq++;
            } while (s.count > 0);
            if (aborted) break;

            // e. Downlink: time, rules, config, buzz ... until bye
            int64_t bye_deadline = esp_timer_get_time() + (int64_t)BYE_TIMEOUT_MS * 1000;
            while (link_read_line(bye_deadline) >= 0) {
                cJSON *m = cJSON_Parse(s_line);
                const char *type = msg_type(m);
                bool bye = false;

                if (strcmp(type, "time") == 0) {
                    // the Pi sent the real time: set our clock and shift everything
                    // that was stored with the old clock, so all timestamps stay
                    // consistent
                    const cJSON *unix_t = cJSON_GetObjectItemCaseSensitive(m, "unix");
                    if (cJSON_IsNumber(unix_t)) {
                        uint32_t unix_now = (uint32_t)unix_t->valuedouble;
                        int64_t delta = (int64_t)unix_now - (int64_t)(uint32_t)time(NULL);
                        struct timeval tv = { .tv_sec = (time_t)unix_now, .tv_usec = 0 };
                        settimeofday(&tv, NULL);

                        for (uint16_t i = 0; i < s.count; ++i) {
                            reading_t *rd = &s.buf[(s.head + i) % BUF_CAPACITY];
                            rd->t = (uint32_t)((int64_t)rd->t + delta);
                        }
                        for (uint8_t i = 0; i < s.alarm_count; ++i) {
                            s.alarms[i].t = (uint32_t)((int64_t)s.alarms[i].t + delta);
                        }
                        if (s.last_session_t != 0) s.last_session_t = (uint32_t)((int64_t)s.last_session_t + delta);
                        if (s.last_attempt_t != 0) s.last_attempt_t = (uint32_t)((int64_t)s.last_attempt_t + delta);
                        if (s.last_daytime_t != 0) s.last_daytime_t = (uint32_t)((int64_t)s.last_daytime_t + delta);
                        s.time_synced = true;
                        ESP_LOGI(TAG, "time set to %" PRIu32 " (shift %" PRId64 " s)", unix_now, delta);
                    }

                } else if (strcmp(type, "rules") == 0) {
                    // {"type":"rules","rules":[{"buzz":3,"sensor":"soil_humidity","op":"<","value":350},...]}
                    const cJSON *rule_list = cJSON_GetObjectItemCaseSensitive(m, "rules");
                    if (cJSON_IsArray(rule_list)) {
                        rule_t rules[MAX_RULES];
                        uint8_t nr = 0;
                        const cJSON *item;
                        cJSON_ArrayForEach(item, rule_list) {
                            const cJSON *buzz = cJSON_GetObjectItemCaseSensitive(item, "buzz");
                            const cJSON *sensor = cJSON_GetObjectItemCaseSensitive(item, "sensor");
                            const cJSON *op = cJSON_GetObjectItemCaseSensitive(item, "op");
                            const cJSON *value = cJSON_GetObjectItemCaseSensitive(item, "value");
                            if (!cJSON_IsNumber(buzz) || !cJSON_IsString(sensor) ||
                                !cJSON_IsString(op) || !cJSON_IsNumber(value)) {
                                ESP_LOGW(TAG, "bad rule ignored");
                                continue;
                            }
                            int sn = find_name(sensor->valuestring, SENSOR_NAMES, SENSOR_COUNT);
                            int on = find_name(op->valuestring, OP_NAMES, OP_COUNT);
                            if (sn < 0 || on < 0 || buzz->valueint < 1 || buzz->valueint > 10) {
                                ESP_LOGW(TAG, "bad rule ignored: %s %s", sensor->valuestring, op->valuestring);
                                continue;
                            }
                            if (nr == MAX_RULES) {
                                ESP_LOGW(TAG, "too many rules, max %d", MAX_RULES);
                                break;
                            }
                            rules[nr++] = (rule_t){
                                .buzz = (uint8_t)buzz->valueint, .sensor = (uint8_t)sn,
                                .op = (uint8_t)on, .value = (int16_t)value->valueint,
                            };
                        }

                        // only reset the alarm states if the rules really changed,
                        // otherwise every session would make all active alarms beep
                        // again
                        if (!(nr == s.rule_count && memcmp(rules, s.rules, sizeof(rule_t) * nr) == 0)) {
                            s.rule_count = nr;
                            memcpy(s.rules, rules, sizeof(rule_t) * nr);
                            memset(s.rule_active, 0, sizeof(s.rule_active));
                            memset(s.since_beep, 0, sizeof(s.since_beep));
                            ESP_LOGI(TAG, "new rules from the Pi: %u", (unsigned)nr);
                        }
                    }

                } else if (strcmp(type, "config") == 0) {
                    // {"type":"config","interval_s":1200,"pi_wake_interval_s":86400} -
                    // both optional
                    // In V3 interval_s sets the NORMAL interval; FAST and MAX stay
                    // fixed.
                    const cJSON *interval = cJSON_GetObjectItemCaseSensitive(m, "interval_s");
                    const cJSON *pi_wake = cJSON_GetObjectItemCaseSensitive(m, "pi_wake_interval_s");
                    if (cJSON_IsNumber(interval) && interval->valueint >= 10) {
                        s.normal_interval_s = (uint32_t)interval->valueint;
                        s.interval_s = s.speed == SPEED_FAST ? INTERVAL_FAST_S
                                     : s.speed == SPEED_MAX  ? INTERVAL_MAX_S
                                     : s.normal_interval_s;
                    }
                    if (cJSON_IsNumber(pi_wake) && pi_wake->valueint >= 60) {
                        s.pi_wake_interval_s = (uint32_t)pi_wake->valueint;
                    }
                    ESP_LOGI(TAG, "config: normal interval %" PRIu32 " s, pi wake %" PRIu32 " s",
                             s.normal_interval_s, s.pi_wake_interval_s);

                } else if (strcmp(type, "buzz") == 0) {
                    const cJSON *count = cJSON_GetObjectItemCaseSensitive(m, "count");
                    if (cJSON_IsNumber(count) && count->valueint >= 1 && count->valueint <= 10) {
                        ESP_LOGI(TAG, "Pi asks for %d beeps", count->valueint);
                        beep(BEEP_ON_MS, BEEP_OFF_MS, count->valueint);
                    }

                } else if (strcmp(type, "bye") == 0) {
                    bye = true;
                }
                cJSON_Delete(m);
                if (bye) break;
            }
            ESP_LOGI(TAG, "session done");
            ok = true;
        } while (0);

        // f. Remove the UART and make TX/RX plain inputs, so the ESP32 never
        // drives a voltage into a Pi that is switched off (back-powering).
        uart_driver_delete(PI_UART);
        gpio_config_t link_pins = {
            .pin_bit_mask = (1ULL << PI_TX_PIN) | (1ULL << PI_RX_PIN),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        gpio_config(&link_pins);

        if (ok) {
            s.last_session_t = (uint32_t)time(NULL);
            s.last_attempt_t = s.last_session_t;
            // A session in daylight (photo taken) counts as today's daily session
            if (s.time_synced) {
                time_t t = (time_t)s.last_session_t;
                struct tm lt;
                localtime_r(&t, &lt);
                if (lt.tm_hour >= DAY_START_HOUR && lt.tm_hour < DAY_END_HOUR) {
                    s.last_daytime_t = s.last_session_t;
                }
            }
            s.change_pending = false;    // the Pi knows now
        }
    }

    // ----- 9. deep sleep until the next measurement -----

    // awake time of this wake-up, for the stats
    int64_t awake_us = esp_timer_get_time();
    s.awake_us_total += (uint64_t)awake_us;
    s.awake_count++;

    // keep a regular schedule: subtract the time we were awake
    int64_t sleep_us = (int64_t)s.interval_s * 1000000 - awake_us;
    if (sleep_us < 1000000) sleep_us = 1000000;

    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup((uint64_t)sleep_us));
    // BOOT button = manual Pi wake. GPIO0 is LOW while pressed.
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(BUTTON_GPIO, 0));
    rtc_gpio_pullup_en(BUTTON_GPIO);
    rtc_gpio_pulldown_dis(BUTTON_GPIO);

    ESP_LOGI(TAG, "awake %" PRId64 " ms, sleeping %" PRId64 " s",
             awake_us / 1000, sleep_us / 1000000);
    gpio_set_level(MARKER_GPIO, 0);      // marker LOW = asleep

#if V2_HOLD_PINS_IN_SLEEP
    // without hold, outputs float in deep sleep: leakage, and the buzzer can click.
    // These are all RTC pins, so gpio_hold_en() keeps them LOW.
    gpio_set_level(BUZZER_GPIO, 0);
    gpio_hold_en(BUZZER_GPIO);
    gpio_hold_en(MARKER_GPIO);
#if V2_SWITCHED_SENSOR_POWER
    gpio_hold_en(SOIL_VCC_GPIO);
    gpio_hold_en(DHT_VCC_GPIO);
#endif
#endif

    fflush(stdout);
    esp_deep_sleep_start();
}
