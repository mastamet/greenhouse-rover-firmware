#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "sh1107.h"

#define STEP_PIN             GPIO_NUM_2    
#define DIR_PIN              GPIO_NUM_4    
#define DIR_BUTTON_PIN       GPIO_NUM_5    // Button 1 (Red): Toggles Direction
#define MOVE_BUTTON_PIN      GPIO_NUM_19   // Button 2 (Blue): 500-Step Targeted Move
#define SENSOR_BUTTON_PIN    GPIO_NUM_23   // Button 3 (Green): Weed Check / Slowdown
#define POT_ADC_CHANNEL      ADC_CHANNEL_7 // Potentiometer on GPIO 35

#define LEDC_TIMER           LEDC_TIMER_0
#define LEDC_MODE            LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO       (STEP_PIN)
#define LEDC_CHANNEL         LEDC_CHANNEL_0
#define LEDC_DUTY_RES        LEDC_TIMER_8_BIT  
#define LEDC_DUTY_VAL        128              

#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_SDA_IO    GPIO_NUM_21
#define I2C_MASTER_SCL_IO    GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ   100000

#define STEPS_PER_METER      200.0f        // Odometer calibration scale

#define WIFI_SSID            "Wokwi-GUEST"
#define WIFI_PASS            ""

static const char *TAG = "ROVER_FIRMWARE";

static volatile uint32_t current_freq = 50;
static volatile uint32_t total_steps_traveled = 0;
static volatile int current_dir = 1;
static volatile bool targeted_mode_active = false;
static volatile bool sensor_slowdown_active = false;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        esp_rom_printf("Wi-Fi Event: Connecting to virtual Wi-Fi AP...\n");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        esp_rom_printf("Wi-Fi Event: Disconnected. Retrying connection...\n");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        esp_rom_printf("IP Address: " IPSTR "\n", IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

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
    char resp_str[256];
    float distance_meters = (float)total_steps_traveled / STEPS_PER_METER;
    const char *mode_str = "AUTO_CRUISE";
    if (targeted_mode_active) mode_str = "TARGETED_MOVE";
    else if (sensor_slowdown_active) mode_str = "SENSOR_CREEP";

    snprintf(resp_str, sizeof(resp_str),
        "{\"status\":\"online\",\"mode\":\"%s\",\"dir\":\"%s\",\"freq_hz\":%lu,\"distance_m\":%.2f,\"steps\":%lu}",
        mode_str,
        current_dir ? "FWD" : "REV",
        (unsigned long)current_freq,
        distance_meters,
        (unsigned long)total_steps_traveled);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t telemetry_uri = {
            .uri       = "/telemetry",
            .method    = HTTP_GET,
            .handler   = telemetry_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &telemetry_uri);
        esp_rom_printf("HTTP Server: /telemetry endpoint registered\n");
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

void app_main(void) {
    esp_rom_printf("1. APP_MAIN STARTED\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    esp_rom_printf("2. NVS Flash Initialized\n");

    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&dir_conf);
    gpio_set_level(DIR_PIN, 1); 

    uint64_t pin_mask = (1ULL << DIR_BUTTON_PIN) | (1ULL << MOVE_BUTTON_PIN) | (1ULL << SENSOR_BUTTON_PIN);
    gpio_config_t btn_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);
    esp_rom_printf("3. GPIOs Configured\n");

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 50,             
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = STEP_PIN,
        .duty           = LEDC_DUTY_VAL,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    esp_rom_printf("4. PWM Configured\n");

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_control = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_control, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &adc_config));
    esp_rom_printf("5. ADC Configured\n");

    ESP_ERROR_CHECK(i2c_master_init());
    sh1107_init(I2C_NUM_0);
    esp_rom_printf("6. I2C & OLED Initialized\n");

    wifi_init_sta();
    start_webserver();
    esp_rom_printf("7. Wi-Fi & Webserver Initialized\n");

    int target_dir = 1;         
    int last_dir_btn_state = 0;
    int last_move_btn_state = 0;
    int last_sensor_btn_state = 0;
    int adc_raw = 0;
    
    int ramp_step = 40;         
    int32_t target_steps_remaining = 0; 
    int display_refresh_counter = 0;
    int telemetry_print_counter = 0;

    while (true) {
        adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &adc_raw);
        uint32_t potentiometer_target_freq = 50 + (adc_raw / 2); 

        int dir_btn_state = gpio_get_level(DIR_BUTTON_PIN);
        if (dir_btn_state == 1 && last_dir_btn_state == 0) {
            target_dir = !target_dir;            
            ESP_LOGI(TAG, "Direction reversal requested.");
        }
        last_dir_btn_state = dir_btn_state;

        int move_btn_state = gpio_get_level(MOVE_BUTTON_PIN);
        if (move_btn_state == 1 && last_move_btn_state == 0) {
            if (!targeted_mode_active) {
                target_steps_remaining = 500; 
                targeted_mode_active = true;
                ESP_LOGI(TAG, "Targeted weed move initiated: 500 steps.");
            }
        }
        last_move_btn_state = move_btn_state;

        int sensor_btn_state = gpio_get_level(SENSOR_BUTTON_PIN);
        if (sensor_btn_state == 1 && last_sensor_btn_state == 0) {
            sensor_slowdown_active = !sensor_slowdown_active;
            ESP_LOGI(TAG, "Sensor weed check mode toggled: %s", sensor_slowdown_active ? "ACTIVE" : "CLEARED");
        }
        last_sensor_btn_state = sensor_btn_state;

        uint32_t effective_target_freq = potentiometer_target_freq;

        if (current_dir != target_dir) {
            effective_target_freq = 0; 
        } else if (sensor_slowdown_active) {
            effective_target_freq = 40; 
        } else if (targeted_mode_active && target_steps_remaining <= 0) {
            effective_target_freq = 0;
            if (current_freq == 0) {
                targeted_mode_active = false; 
                ESP_LOGI(TAG, "Target reached. Resuming cruise.");
            }
        }

        if (current_freq < effective_target_freq) {
            current_freq += ramp_step;
            if (current_freq > effective_target_freq) current_freq = effective_target_freq;
        } else if (current_freq > effective_target_freq) {
            if (current_freq >= ramp_step) {
                current_freq -= ramp_step;
            } else {
                current_freq = 0;
            }
            if (current_freq < effective_target_freq) current_freq = effective_target_freq;
        }

        if (current_freq == 0 && current_dir != target_dir) {
            current_dir = target_dir;               
            gpio_set_level(DIR_PIN, current_dir);   
            ESP_LOGI(TAG, "DIR pin flipped to %d.", current_dir);
        }

        if (current_freq > 0) {
            ledc_set_freq(LEDC_MODE, LEDC_TIMER, current_freq);

            uint32_t steps_this_cycle = (current_freq * 30) / 1000;
            total_steps_traveled += steps_this_cycle;

            if (targeted_mode_active) {
                target_steps_remaining -= steps_this_cycle;
                if (target_steps_remaining < 0) target_steps_remaining = 0;
            }
        }

        // Print Telemetry JSON to Serial every ~1 second (33 loops * 30ms)
        telemetry_print_counter++;
        if (telemetry_print_counter >= 33) {
            telemetry_print_counter = 0;
            float distance_meters = (float)total_steps_traveled / STEPS_PER_METER;
            const char *mode_str = "AUTO_CRUISE";
            if (targeted_mode_active) mode_str = "TARGETED_MOVE";
            else if (sensor_slowdown_active) mode_str = "SENSOR_CREEP";

            esp_rom_printf("TELEMETRY: {\"status\":\"online\",\"mode\":\"%s\",\"dir\":\"%s\",\"freq_hz\":%lu,\"distance_m\":%.2f,\"steps\":%lu}\n",
                mode_str,
                current_dir ? "FWD" : "REV",
                (unsigned long)current_freq,
                distance_meters,
                (unsigned long)total_steps_traveled);
        }

        display_refresh_counter++;
        if (display_refresh_counter >= 4) {
            display_refresh_counter = 0;

            char line_buf[32];
            sh1107_clear();

            if (targeted_mode_active) {
                sh1107_draw_string(0, 0, "=== ACTION / WEED ===");
                sh1107_draw_string(2, 0, "STATUS: TARGETED MOVE");
                sh1107_draw_string(4, 0, "TARGET: 500 STEPS");
                
                snprintf(line_buf, sizeof(line_buf), "REMAIN: %ld STEPS", (long)target_steps_remaining);
                sh1107_draw_string(6, 0, line_buf);
            }
            else if (sensor_slowdown_active) {
                sh1107_draw_string(0, 0, "=== SENSOR / ALERT ===");
                sh1107_draw_string(2, 0, "STATUS: LEAF/WEED CHECK");
                
                snprintf(line_buf, sizeof(line_buf), "POT SPD: %lu HZ", (unsigned long)potentiometer_target_freq);
                sh1107_draw_string(4, 0, line_buf);
                
                sh1107_draw_string(6, 0, "ACTION: CREEP SLOWDOWN");
            }
            else {
                sh1107_draw_string(0, 0, "GREENHOUSE ROVER");
                sh1107_draw_string(2, 0, "MODE: AUTO CRUISE");
                
                snprintf(line_buf, sizeof(line_buf), "DIR:  %s | S:%luHz", current_dir ? "FWD" : "REV", (unsigned long)current_freq);
                sh1107_draw_string(4, 0, line_buf);

                float distance_meters = (float)total_steps_traveled / STEPS_PER_METER;
                snprintf(line_buf, sizeof(line_buf), "DIST:%.1fm S:%lu", distance_meters, (unsigned long)total_steps_traveled);
                sh1107_draw_string(6, 0, line_buf);
            }

            sh1107_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
