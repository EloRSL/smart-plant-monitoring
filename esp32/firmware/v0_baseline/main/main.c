// Smart plant monitoring - ESP32 part, version V0 (baseline)
// Everything runs in one loop with vTaskDelay, no sleep at all.
// The Pi is always on and gets every reading right away.
// PECD 2026, Vid Kramar

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "cJSON.h"
#include "dht.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FW_VERSION "v0"

// ----- settings -----

// 1 = short intervals for testing/demo, 0 = real intervals
#define DEMO_MODE 0

#if DEMO_MODE
#define MEASURE_INTERVAL_S   30
#else
#define MEASURE_INTERVAL_S   (20 * 60)
#endif

// pins
#define WAKE_PIN          GPIO_NUM_25    // to Pi pin 5 (GPIO3), open drain
#define PI_UART           UART_NUM_2
#define PI_TX_PIN         GPIO_NUM_17    // board label TX2, to Pi pin 10
#define PI_RX_PIN         GPIO_NUM_16    // board label RX2, to Pi pin 8
#define DHT_GPIO          GPIO_NUM_26
#define BUZZER_GPIO       GPIO_NUM_27
#define MOISTURE_CHANNEL  ADC_CHANNEL_6  // ADC1 CH6 = GPIO34
#define MARKER_GPIO       GPIO_NUM_32    // HIGH while awake (always, in V0)

// soil calibration, raw ADC 0..4095. Resistive probe: raw rises with moisture.
// soil_humidity 0 = probe in air, 1000 = probe in water.
#define SOIL_RAW_AIR      0
#define SOIL_RAW_WATER    2723
#define SOIL_SAMPLES      16             // ADC reads averaged per measurement

// DHT11
#define DHT_STARTUP_DELAY_MS  2000       // DHT11 is not ready right after power-on
#define DHT_RETRIES           2          // extra attempts after a failed read
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
#define READY_TIMEOUT_MS  5000           // Pi is always on and repeats "ready" every 2 s
#define ACK_TIMEOUT_MS    5000
#define ACK_RETRIES       3
#define BYE_TIMEOUT_MS    30000          // time, rules, config, buzz, then bye
#define CHUNK_SIZE        50             // max readings per batch message

// buffers
#define BUF_CAPACITY      400            // unsent readings kept in RAM (8 B each)
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

// program state (normal RAM: V0 never sleeps)
typedef struct {
    uint32_t wakes;
    uint32_t seq;                 // next batch sequence number
    uint32_t dropped;             // readings lost because the buffer was full
    bool     time_synced;         // true after the Pi sent the time
    uint32_t interval_s;          // measurement interval (Pi can change it)

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

static node_state_t s;

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

// ----- main: set everything up once, then measure and send forever -----

void app_main(void)
{
    // ----- 1. start-up: pins, sensors, program state -----

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

    // wake line to the Pi, open drain: 1 = released, 0 = pulls the Pi wake pin to ground
    gpio_set_level(WAKE_PIN, 1);
    gpio_config_t wake_config = {
        .pin_bit_mask = 1ULL << WAKE_PIN,
        .mode = GPIO_MODE_OUTPUT_OD,
    };
    gpio_config(&wake_config);

    // program state: default rules until the Pi sends its configuration.txt
    memset(&s, 0, sizeof(s));
    s.interval_s = MEASURE_INTERVAL_S;
    s.rule_count = sizeof(DEFAULT_RULES) / sizeof(DEFAULT_RULES[0]);
    memcpy(s.rules, DEFAULT_RULES, sizeof(DEFAULT_RULES));
    ESP_LOGI(TAG, "state initialised, %u default rules", (unsigned)s.rule_count);

    // the Pi is assumed always on; one wake pulse at start-up is harmless if it already runs
    gpio_set_level(WAKE_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
    gpio_set_level(WAKE_PIN, 1);

    // the UART driver stays installed all the time in V0: 115200 8N1
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

    // after power-on the DHT11 needs time
    vTaskDelay(pdMS_TO_TICKS(DHT_STARTUP_DELAY_MS));

    while (1) {
        s.wakes++;

        // ----- 2. measure -----

        reading_t r = {
            .t = (uint32_t)time(NULL), .temp = TEMP_INVALID, .hum = HUM_INVALID, .soil = SOIL_INVALID,
        };

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

        // soil moisture: average of SOIL_SAMPLES ADC reads, then raw ADC -> 0 (air) ..
        // 1000 (water), linear between the two calibration points
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

        ESP_LOGI(TAG, "measured: air_temperature=%d air_humidity=%u soil_humidity=%u (raw %d)",
                 r.temp, (unsigned)r.hum, (unsigned)r.soil, raw);

        // ----- 3. store the reading in the ring buffer -----

        if (s.count == BUF_CAPACITY) {       // full: overwrite the oldest
            s.head = (s.head + 1) % BUF_CAPACITY;
            s.count--;
            s.dropped++;
        }
        s.buf[(s.head + s.count) % BUF_CAPACITY] = r;
        s.count++;

        // ----- 4. check the rules and beep -----
        // beep when a rule BECOMES true, then again every REMINDER_EVERY readings while
        // it stays true (not on every reading, or it beeps all night). Hysteresis: once
        // active, the threshold moves by SENSOR_HYST so a value hovering at the
        // threshold does not switch the alarm on and off.
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

        // ----- 5. send it to the Pi right away -----
        //   a. wait for {"type":"ready"}       (the ESP sends nothing before it)
        //   b. send pending alarms
        //   c. send readings in chunks of CHUNK_SIZE; delete each chunk only after its
        //   ack
        //   d. handle time / rules / config / buzz until "bye"
        // If the Pi does not answer, the reading stays in the buffer and goes out with
        // the next one.
        // any failure leaves the block with "break"; the readings stay in the buffer
        // and go out with the next session
        do {
            // a. Wait for "ready"; other messages are ignored
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

            // b. Alarms first. They are deleted once the first ack proves the Pi listens.
            for (uint8_t i = 0; i < s.alarm_count; ++i) {
                cJSON *a = cJSON_CreateObject();
                cJSON_AddStringToObject(a, "type", "alarm");
                cJSON_AddNumberToObject(a, "t", s.alarms[i].t);
                cJSON_AddNumberToObject(a, "buzz", s.alarms[i].buzz);
                cJSON_AddStringToObject(a, "sensor", SENSOR_NAMES[s.alarms[i].sensor]);
                cJSON_AddNumberToObject(a, "value", s.alarms[i].value);
                link_send(a);
            }

            // c. Readings in chunks, each one deleted only after its ack
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
                    cJSON_AddStringToObject(batch, "reason", "periodic");
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
                        // V0 never sleeps: it is awake the whole time, so awake time
                        // per measurement = uptime / number of measurements (about one
                        // interval)
                        cJSON_AddNumberToObject(stats, "avg_awake_ms",
                                                s.wakes ? (double)(esp_timer_get_time() / s.wakes / 1000) : 0);
                        cJSON_AddNumberToObject(stats, "dropped", s.dropped);
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

            // d. Downlink: time, rules, config, buzz ... until bye
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
                    // {"type":"config","interval_s":30} (optional)
                    const cJSON *interval = cJSON_GetObjectItemCaseSensitive(m, "interval_s");
                    if (cJSON_IsNumber(interval) && interval->valueint >= 10) {
                        s.interval_s = (uint32_t)interval->valueint;
                    }
                    ESP_LOGI(TAG, "config: interval %" PRIu32 " s", s.interval_s);

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
        } while (0);

        // ----- 6. idle until the next measurement (V0 never sleeps) -----
        vTaskDelay(pdMS_TO_TICKS(s.interval_s * 1000));
    }
}
