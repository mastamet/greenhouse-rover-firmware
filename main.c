/*
 * GREENHOUSE ROVER - firmware v3
 *   Step 1: Simulated detection layer (serial commands + buttons)
 *   Step 2: Mission state machine
 *           CRUISE > DETECTED > STOP > ALIGN > FIRE > COOLDOWN > RESUME   (weeds)
 *           CRUISE > DETECTED > STOP > INSPECT > RESUME                   (disease)
 *   Also:   signed odometry, fractional step counting, PWM actually stops at 0 Hz
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "sh1107.h"

/* ---------------- Pins ---------------- */
#define STEP_PIN             GPIO_NUM_2
#define DIR_PIN              GPIO_NUM_4
#define DIR_BUTTON_PIN       GPIO_NUM_5    // Red   : toggle direction
#define WEED_BUTTON_PIN      GPIO_NUM_19   // Blue  : inject WEED detection
#define DISEASE_BUTTON_PIN   GPIO_NUM_23   // Green : inject BLIGHT detection
#define LASER_LED_PIN        GPIO_NUM_18   // Red LED    : "laser" (on only in FIRE)
#define ALERT_LED_PIN        GPIO_NUM_27   // Yellow LED : on while engaging a target
#define POT_ADC_CHANNEL      ADC_CHANNEL_7 // Potentiometer on GPIO 35

/* ---------------- PWM (step pulses) ---------------- */
#define LEDC_TIMER           LEDC_TIMER_0
#define LEDC_MODE            LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL         LEDC_CHANNEL_0
#define LEDC_DUTY_RES        LEDC_TIMER_8_BIT
#define LEDC_DUTY_VAL        128

/* ---------------- I2C ---------------- */
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_SDA_IO    GPIO_NUM_21
#define I2C_MASTER_SCL_IO    GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ   100000

/* ---------------- Wi-Fi ---------------- */
#define WIFI_SSID            "Wokwi-GUEST"
#define WIFI_PASS            ""

/* ---------------- Tuning ---------------- */
#define STEPS_PER_METER      200.0f
#define LOOP_MS              30
#define RAMP_STEP_HZ         40
#define TELEMETRY_LOOPS      67            // ~2 s
#define DISPLAY_LOOPS        4

#define CONF_THRESHOLD       60            // % - below this a detection is ignored
#define DETECTED_CONFIRM_MS  200
#define ALIGN_BASE_MS        300
#define ALIGN_PER_MM_MS      8             // extra aim time per mm of offset
#define FIRE_MS              600
#define COOLDOWN_MS          1000
#define INSPECT_MS           2000
#define RESUME_HOLDOFF_MS    1500          // ignore detections right after resuming

static const char *TAG = "ROVER";

/* ================================================================
 *  DETECTION LAYER  (swap the producer for a real camera later)
 * ================================================================ */
typedef enum {
    DET_NONE = 0,
    DET_HEALTHY,
    DET_WEED,
    DET_BLIGHT,
    DET_FUNGAL,
    DET_NUTRIENT,
} det_type_t;

static const char *DET_NAMES[] = { "NONE", "HEALTHY", "WEED", "BLIGHT", "FUNGAL", "NUTRIENT" };

typedef struct {
    det_type_t type;
    uint8_t    confidence;      // 0-100 %
    int16_t    offset_mm;       // horizontal offset of target from camera centre (-100..100)
    int32_t    position_steps;  // rover position when it was seen
} detection_t;

static QueueHandle_t det_queue;

/* ================================================================
 *  STATE MACHINE
 * ================================================================ */
typedef enum {
    ST_CRUISE = 0,
    ST_DETECTED,
    ST_STOP,
    ST_ALIGN,
    ST_FIRE,
    ST_COOLDOWN,
    ST_INSPECT,
    ST_RESUME,
    ST_ESTOP,
} rover_state_t;

static const char *STATE_NAMES[] = {
    "CRUISE", "DETECTED", "STOP", "ALIGN", "FIRE", "COOLDOWN", "INSPECT", "RESUME", "ESTOP"
};

/* ---------------- Shared state ---------------- */
static volatile rover_state_t rover_state = ST_CRUISE;
static volatile uint32_t current_freq = 0;
static volatile int      current_dir = 1;                 // 1 = FWD, 0 = REV
static volatile uint32_t total_steps_traveled = 0;        // odometer (always increases)
static volatile int32_t  rover_position_steps = 0;        // signed position along the row

static volatile uint32_t cnt_weeds_detected = 0;
static volatile uint32_t cnt_weeds_zapped   = 0;
static volatile uint32_t cnt_diseases       = 0;
static volatile uint32_t cnt_healthy        = 0;
static volatile uint32_t cnt_low_conf       = 0;
static volatile uint32_t cnt_dropped        = 0;

static detection_t active_det;          // target currently being engaged
static detection_t last_det;            // last accepted detection (for telemetry)
static bool        have_last_det = false;
static uint32_t    state_enter_ms = 0;

/* Requests coming from the serial task, handled in the main loop */
static volatile bool req_estop = false;
static volatile bool req_reset = false;
static volatile bool req_abort = false;
static volatile bool req_status = false;

static bool pwm_running = false;
static uint32_t pwm_last_freq = 0;

/* ---------------- Helpers ---------------- */
static uint32_t now_ms(void) { return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS); }

static void enter_state(rover_state_t s) {
    rover_state = s;
    state_enter_ms = now_ms();

    gpio_set_level(LASER_LED_PIN, s == ST_FIRE ? 1 : 0);
    bool engaged = (s == ST_DETECTED || s == ST_STOP || s == ST_ALIGN || s == ST_FIRE ||
                    s == ST_COOLDOWN || s == ST_INSPECT || s == ST_ESTOP);
    gpio_set_level(ALERT_LED_PIN, engaged ? 1 : 0);

    ESP_LOGI(TAG, ">> STATE: %s  (pos=%.2f m)", STATE_NAMES[s],
             (float)rover_position_steps / STEPS_PER_METER);
}

static uint32_t state_elapsed_ms(void) { return now_ms() - state_enter_ms; }

static void motor_set_freq(uint32_t f) {
    if (f == 0) {
        if (pwm_running) {
            ledc_stop(LEDC_MODE, LEDC_CHANNEL, 0);   // really stop the step pulses
            pwm_running = false;
            pwm_last_freq = 0;
        }
        return;
    }
    if (f != pwm_last_freq) {
        ledc_set_freq(LEDC_MODE, LEDC_TIMER, f);
        pwm_last_freq = f;
    }
    if (!pwm_running) {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_VAL);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        pwm_running = true;
    }
}

/* ---------------- Detection injection ---------------- */
static bool inject_detection(det_type_t type, int conf, int offset_mm, const char *src) {
    detection_t d = {
        .type = type,
        .confidence = (uint8_t)conf,
        .offset_mm = (int16_t)offset_mm,
        .position_steps = rover_position_steps,
    };
    if (xQueueSend(det_queue, &d, 0) != pdTRUE) {
        ESP_LOGW(TAG, "[%s] detection queue full, dropped", src);
        return false;
    }
    ESP_LOGI(TAG, "[%s] camera sees %s conf=%d%% offset=%dmm", src, DET_NAMES[type], conf, offset_mm);
    return true;
}

/* Returns true if this detection should start an engagement */
static bool process_detection(const detection_t *d) {
    if (d->confidence < CONF_THRESHOLD) {
        cnt_low_conf++;
        ESP_LOGW(TAG, "Ignored %s: confidence %d%% < %d%%", DET_NAMES[d->type], d->confidence, CONF_THRESHOLD);
        return false;
    }
    last_det = *d;
    have_last_det = true;

    switch (d->type) {
        case DET_HEALTHY:
            cnt_healthy++;
            ESP_LOGI(TAG, "Healthy leaf logged, continuing.");
            return false;
        case DET_WEED:
            cnt_weeds_detected++;
            active_det = *d;
            return true;
        case DET_BLIGHT:
        case DET_FUNGAL:
        case DET_NUTRIENT:
            cnt_diseases++;
            active_det = *d;
            return true;
        default:
            return false;
    }
}

static void drain_queue_dropped(const char *why) {
    detection_t d;
    while (xQueueReceive(det_queue, &d, 0) == pdTRUE) {
        cnt_dropped++;
        ESP_LOGW(TAG, "Dropped %s (%s)", DET_NAMES[d.type], why);
    }
}

/* ---------------- Telemetry ---------------- */
static void build_telemetry(char *buf, size_t n) {
    float pos_m  = (float)rover_position_steps / STEPS_PER_METER;
    float odo_m  = (float)total_steps_traveled / STEPS_PER_METER;
    const char *ld = have_last_det ? DET_NAMES[last_det.type] : "NONE";
    snprintf(buf, n,
        "{\"status\":\"online\",\"state\":\"%s\",\"dir\":\"%s\",\"freq_hz\":%lu,"
        "\"position_m\":%.2f,\"odometer_m\":%.2f,"
        "\"weeds_detected\":%lu,\"weeds_zapped\":%lu,\"diseases\":%lu,\"healthy\":%lu,"
        "\"low_conf\":%lu,\"dropped\":%lu,\"last_det\":\"%s\"}",
        STATE_NAMES[rover_state], current_dir ? "FWD" : "REV", (unsigned long)current_freq,
        pos_m, odo_m,
        (unsigned long)cnt_weeds_detected, (unsigned long)cnt_weeds_zapped,
        (unsigned long)cnt_diseases, (unsigned long)cnt_healthy,
        (unsigned long)cnt_low_conf, (unsigned long)cnt_dropped, ld);
}

/* ================================================================
 *  SERIAL COMMAND TASK  (type into the Wokwi serial monitor)
 * ================================================================ */
static void print_help(void) {
    printf("\n--- ROVER COMMANDS -------------------------------------\n");
    printf(" weed     [conf] [offset_mm]   inject weed detection\n");
    printf(" blight   [conf] [offset_mm]   inject blight detection\n");
    printf(" fungal   [conf] [offset_mm]   inject fungal detection\n");
    printf(" nutrient [conf] [offset_mm]   inject nutrient deficiency\n");
    printf(" healthy  [conf]               inject healthy leaf\n");
    printf(" abort                         cancel current engagement\n");
    printf(" estop / reset                 emergency stop / clear it\n");
    printf(" status                        print telemetry now\n");
    printf(" (conf default 90, offset default random -60..60)\n");
    printf("--------------------------------------------------------\n");
}

static void handle_command(char *line) {
    char cmd[16] = {0};
    int a = 0, b = 0;
    int n = sscanf(line, "%15s %d %d", cmd, &a, &b);
    if (n < 1) return;
    for (char *p = cmd; *p; p++) *p = (char)tolower((unsigned char)*p);

    int conf = (n >= 2 && a >= 0 && a <= 100) ? a : 90;
    int off  = (n >= 3) ? b : ((int)(esp_random() % 121) - 60);
    if (off > 100) off = 100;
    if (off < -100) off = -100;

    if      (!strcmp(cmd, "weed"))     inject_detection(DET_WEED, conf, off, "SERIAL");
    else if (!strcmp(cmd, "blight"))   inject_detection(DET_BLIGHT, conf, off, "SERIAL");
    else if (!strcmp(cmd, "fungal"))   inject_detection(DET_FUNGAL, conf, off, "SERIAL");
    else if (!strcmp(cmd, "nutrient")) inject_detection(DET_NUTRIENT, conf, off, "SERIAL");
    else if (!strcmp(cmd, "healthy"))  inject_detection(DET_HEALTHY, conf, 0, "SERIAL");
    else if (!strcmp(cmd, "abort"))    req_abort = true;
    else if (!strcmp(cmd, "estop"))    req_estop = true;
    else if (!strcmp(cmd, "reset"))    req_reset = true;
    else if (!strcmp(cmd, "status"))   req_status = true;
    else if (!strcmp(cmd, "help"))     print_help();
    else printf("Unknown command '%s' (type help)\n", cmd);
}

static void serial_cmd_task(void *arg) {
    char line[64];
    int len = 0;
    uint8_t ch;
    print_help();
    while (true) {
        int n = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(50));
        if (n > 0) {
            if (ch == '\r' || ch == '\n') {
                if (len > 0) {
                    line[len] = 0;
                    handle_command(line);
                    len = 0;
                }
            } else if (len < (int)sizeof(line) - 1) {
                line[len++] = (char)ch;
            }
        }
    }
}

/* ================================================================
 *  Wi-Fi + web server
 * ================================================================ */
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t inst_any, inst_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &inst_any));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &inst_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static esp_err_t telemetry_get_handler(httpd_req_t *req) {
    char resp[448];
    build_telemetry(resp, sizeof(resp));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri = { .uri = "/telemetry", .method = HTTP_GET, .handler = telemetry_get_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &uri);
        ESP_LOGI(TAG, "HTTP: /telemetry registered");
    }
    return server;
}

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/* ================================================================
 *  OLED
 * ================================================================ */
static void oled_line(uint8_t page, const char *fmt, ...) {
    char buf[24];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    sh1107_draw_string(page, 0, buf);          // ~21 chars fit per line
}

/* Short names so every OLED line stays within 16 characters */
static const char *OLED_NAMES[] = { "NONE", "HEALTHY", "WEED", "BLIGHT", "FUNGAL", "NUTR" };

static void render_display(void) {
    sh1107_clear();
    /* Page 0 is partly hidden by the bezel, so the layout uses pages 2-14. Max 16 chars per line. */
    oled_line(2, "%s %s", STATE_NAMES[rover_state], current_dir ? "FWD" : "REV");
    oled_line(4, "SPEED %lu HZ", (unsigned long)current_freq);
    oled_line(6, "POS %.2f M", (float)rover_position_steps / STEPS_PER_METER);
    oled_line(8, "DIST %.1f M", (float)total_steps_traveled / STEPS_PER_METER);

    switch (rover_state) {
        case ST_DETECTED: case ST_STOP: case ST_ALIGN: case ST_FIRE:
        case ST_COOLDOWN: case ST_INSPECT:
            oled_line(10, "%s %d%% %dMM", OLED_NAMES[active_det.type], active_det.confidence, active_det.offset_mm);
            break;
        case ST_ESTOP:
            oled_line(10, "TYPE RESET");
            break;
        default:
            if (have_last_det) oled_line(10, "LAST %s %d%%", OLED_NAMES[last_det.type], last_det.confidence);
            else               oled_line(10, "LAST NONE");
            break;
    }
    oled_line(12, "W%lu Z%lu D%lu",
              (unsigned long)cnt_weeds_detected, (unsigned long)cnt_weeds_zapped, (unsigned long)cnt_diseases);
    oled_line(14, "OK %lu SKIP %lu",
              (unsigned long)cnt_healthy, (unsigned long)(cnt_low_conf + cnt_dropped));
    sh1107_refresh();
}

/* ================================================================
 *  MAIN
 * ================================================================ */
void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Outputs */
    gpio_config_t out_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN) | (1ULL << LASER_LED_PIN) | (1ULL << ALERT_LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_conf);
    gpio_set_level(DIR_PIN, 1);
    gpio_set_level(LASER_LED_PIN, 0);
    gpio_set_level(ALERT_LED_PIN, 0);

    /* Buttons */
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << DIR_BUTTON_PIN) | (1ULL << WEED_BUTTON_PIN) | (1ULL << DISEASE_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    /* PWM for STEP pin (starts stopped) */
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = STEP_PIN,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    /* ADC */
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_control = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_control, &adc1_handle));
    adc_oneshot_chan_cfg_t adc_config = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_12 };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &adc_config));

    /* OLED */
    ESP_ERROR_CHECK(i2c_master_init());
    sh1107_init(I2C_NUM_0);

    /* Detection queue + serial console input */
    det_queue = xQueueCreate(8, sizeof(detection_t));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    xTaskCreate(serial_cmd_task, "serial_cmd", 4096, NULL, 4, NULL);

    /* Network */
    wifi_init_sta();
    start_webserver();

    enter_state(ST_CRUISE);

    int target_dir = 1;
    int last_dir_btn = 0, last_weed_btn = 0, last_dis_btn = 0;
    int adc_raw = 0;
    uint32_t step_accum = 0;
    int display_counter = 0, telemetry_counter = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        /* ---------- Inputs ---------- */
        adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &adc_raw);
        uint32_t pot_freq = 50 + (adc_raw / 2);

        int b = gpio_get_level(DIR_BUTTON_PIN);
        if (b == 1 && last_dir_btn == 0) {
            target_dir = !target_dir;
            ESP_LOGI(TAG, "Direction reversal requested.");
        }
        last_dir_btn = b;

        b = gpio_get_level(WEED_BUTTON_PIN);
        if (b == 1 && last_weed_btn == 0)
            inject_detection(DET_WEED, 90, (int)(esp_random() % 121) - 60, "BTN");
        last_weed_btn = b;

        b = gpio_get_level(DISEASE_BUTTON_PIN);
        if (b == 1 && last_dis_btn == 0)
            inject_detection(DET_BLIGHT, 85, (int)(esp_random() % 121) - 60, "BTN");
        last_dis_btn = b;

        /* ---------- Serial requests ---------- */
        if (req_status) {
            req_status = false;
            char tbuf[448];
            build_telemetry(tbuf, sizeof(tbuf));
            printf("STATUS: %s\n", tbuf);
        }
        if (req_estop) {
            req_estop = false;
            current_freq = 0;
            motor_set_freq(0);
            enter_state(ST_ESTOP);
        }
        if (req_reset) {
            req_reset = false;
            if (rover_state == ST_ESTOP) enter_state(ST_CRUISE);
            else ESP_LOGW(TAG, "reset ignored (not in ESTOP)");
        }
        if (req_abort) {
            req_abort = false;
            if (rover_state != ST_CRUISE && rover_state != ST_RESUME && rover_state != ST_ESTOP) {
                ESP_LOGW(TAG, "Engagement aborted by operator.");
                enter_state(ST_RESUME);
            }
        }

        /* ---------- State machine ---------- */
        detection_t d;
        switch (rover_state) {
            case ST_CRUISE:
                if (xQueueReceive(det_queue, &d, 0) == pdTRUE) {
                    if (process_detection(&d)) {
                        ESP_LOGI(TAG, "TARGET LOCKED: %s %d%% offset %d mm at %.2f m",
                                 DET_NAMES[active_det.type], active_det.confidence, active_det.offset_mm,
                                 (float)active_det.position_steps / STEPS_PER_METER);
                        enter_state(ST_DETECTED);
                    }
                }
                break;

            case ST_DETECTED:
                drain_queue_dropped("busy");
                if (state_elapsed_ms() >= DETECTED_CONFIRM_MS) enter_state(ST_STOP);
                break;

            case ST_STOP:
                drain_queue_dropped("busy");
                if (current_freq == 0) {
                    enter_state(active_det.type == DET_WEED ? ST_ALIGN : ST_INSPECT);
                }
                break;

            case ST_ALIGN: {
                drain_queue_dropped("busy");
                int mm = active_det.offset_mm < 0 ? -active_det.offset_mm : active_det.offset_mm;
                if (state_elapsed_ms() >= (uint32_t)(ALIGN_BASE_MS + mm * ALIGN_PER_MM_MS)) {
                    ESP_LOGI(TAG, "Aimed at offset %d mm. LASER ON (simulated)", active_det.offset_mm);
                    enter_state(ST_FIRE);
                }
                break;
            }

            case ST_FIRE:
                drain_queue_dropped("busy");
                if (state_elapsed_ms() >= FIRE_MS) {
                    cnt_weeds_zapped++;
                    ESP_LOGI(TAG, "LASER OFF. Weed destroyed (total %lu).", (unsigned long)cnt_weeds_zapped);
                    enter_state(ST_COOLDOWN);
                }
                break;

            case ST_COOLDOWN:
                drain_queue_dropped("busy");
                if (state_elapsed_ms() >= COOLDOWN_MS) enter_state(ST_RESUME);
                break;

            case ST_INSPECT:
                drain_queue_dropped("busy");
                if (state_elapsed_ms() >= INSPECT_MS) {
                    ESP_LOGW(TAG, "DISEASE FLAGGED: %s %d%% at %.2f m - needs human inspection",
                             DET_NAMES[active_det.type], active_det.confidence,
                             (float)active_det.position_steps / STEPS_PER_METER);
                    enter_state(ST_RESUME);
                }
                break;

            case ST_RESUME:
                drain_queue_dropped("holdoff");
                if (state_elapsed_ms() >= RESUME_HOLDOFF_MS) enter_state(ST_CRUISE);
                break;

            case ST_ESTOP:
                drain_queue_dropped("estop");
                break;
        }

        /* ---------- Motor control ---------- */
        bool drive = (rover_state == ST_CRUISE || rover_state == ST_RESUME);
        uint32_t target_freq = drive ? pot_freq : 0;
        if (current_dir != target_dir) target_freq = 0;      // stop before reversing

        if (current_freq < target_freq) {
            current_freq += RAMP_STEP_HZ;
            if (current_freq > target_freq) current_freq = target_freq;
        } else if (current_freq > target_freq) {
            current_freq = (current_freq > RAMP_STEP_HZ) ? current_freq - RAMP_STEP_HZ : 0;
            if (current_freq < target_freq) current_freq = target_freq;
        }

        if (current_freq == 0 && current_dir != target_dir) {
            current_dir = target_dir;
            gpio_set_level(DIR_PIN, current_dir);
            ESP_LOGI(TAG, "DIR pin flipped to %d.", current_dir);
        }

        motor_set_freq(current_freq);

        /* ---------- Odometry (signed, no truncation loss) ---------- */
        if (current_freq > 0) {
            step_accum += current_freq * LOOP_MS;
            uint32_t steps = step_accum / 1000;
            step_accum %= 1000;
            total_steps_traveled += steps;
            if (current_dir) rover_position_steps += (int32_t)steps;
            else             rover_position_steps -= (int32_t)steps;
        }

        /* ---------- Telemetry + display ---------- */
        if (++telemetry_counter >= TELEMETRY_LOOPS) {
            telemetry_counter = 0;
            char tbuf[448];
            build_telemetry(tbuf, sizeof(tbuf));
            printf("TELEMETRY: %s\n", tbuf);
        }
        if (++display_counter >= DISPLAY_LOOPS) {
            display_counter = 0;
            render_display();
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOOP_MS));
    }
}
