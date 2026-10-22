#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
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

static const char *TAG = "ROVER_FIRMWARE";

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
    // 1. Configure Hardware Peripherals
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

    // 2. Configure LEDC PWM Timer & Channel
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

    // 3. Initialize ADC for Potentiometer (GPIO 35 / ADC1 Channel 7)
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_control = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_control, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &adc_config));

    // 4. Initialize I2C and SH1107 OLED Display
    ESP_ERROR_CHECK(i2c_master_init());
    sh1107_init(I2C_NUM_0);

    ESP_LOGI(TAG, "Greenhouse Rover Context-Aware Firmware Booted.");

    int current_dir = 1;        
    int target_dir = 1;         
    int last_dir_btn_state = 0;
    int last_move_btn_state = 0;
    int last_sensor_btn_state = 0;
    int adc_raw = 0;
    
    uint32_t current_freq = 50; 
    int ramp_step = 40;         

    int32_t target_steps_remaining = 0; 
    bool targeted_mode_active = false;  
    bool sensor_slowdown_active = false;
    int display_refresh_counter = 0;

    while (true) {
        // Read potentiometer speed scaling (GPIO 35)
        adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &adc_raw);
        uint32_t potentiometer_target_freq = 50 + (adc_raw / 2); 

        // --- BUTTON 1: Direction Toggle / Reversing (GPIO 5 - Red) ---
        int dir_btn_state = gpio_get_level(DIR_BUTTON_PIN);
        if (dir_btn_state == 1 && last_dir_btn_state == 0) {
            target_dir = !target_dir;            
            ESP_LOGI(TAG, "Direction reversal requested.");
        }
        last_dir_btn_state = dir_btn_state;

        // --- BUTTON 2: Targeted Countdown Trigger (GPIO 19 - Blue) ---
        int move_btn_state = gpio_get_level(MOVE_BUTTON_PIN);
        if (move_btn_state == 1 && last_move_btn_state == 0) {
            if (!targeted_mode_active) {
                target_steps_remaining = 500; 
                targeted_mode_active = true;
                ESP_LOGI(TAG, "Targeted weed move initiated: 500 steps.");
            }
        }
        last_move_btn_state = move_btn_state;

        // --- BUTTON 3: Sensor Weed Check Toggle (GPIO 23 - Green) ---
        int sensor_btn_state = gpio_get_level(SENSOR_BUTTON_PIN);
        if (sensor_btn_state == 1 && last_sensor_btn_state == 0) {
            sensor_slowdown_active = !sensor_slowdown_active;
            ESP_LOGI(TAG, "Sensor weed check mode toggled: %s", sensor_slowdown_active ? "ACTIVE" : "CLEARED");
        }
        last_sensor_btn_state = sensor_btn_state;

        // --- FREQUENCY TARGET CALCULATION & OVERRIDES ---
        uint32_t effective_target_freq = potentiometer_target_freq;

        if (current_dir != target_dir) {
            effective_target_freq = 0; // Zero-crossing brake for reversing
        } else if (sensor_slowdown_active) {
            effective_target_freq = 40; // Creep crawl speed for leaf check/weeding
        } else if (targeted_mode_active && target_steps_remaining <= 0) {
            effective_target_freq = 0;
            if (current_freq == 0) {
                targeted_mode_active = false; 
                ESP_LOGI(TAG, "Target reached. Resuming cruise.");
            }
        }

        // --- ACCELERATION RAMPING ---
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

        // --- ZERO-CROSSING SWAP ---
        if (current_freq == 0 && current_dir != target_dir) {
            current_dir = target_dir;               
            gpio_set_level(DIR_PIN, current_dir);   
            ESP_LOGI(TAG, "DIR pin flipped to %d.", current_dir);
        }

        // --- PWM UPDATE & STEP TRACKING ---
        if (current_freq > 0) {
            ledc_set_freq(LEDC_MODE, LEDC_TIMER, current_freq);

            if (targeted_mode_active) {
                int32_t steps_this_cycle = (current_freq * 30) / 1000;
                target_steps_remaining -= steps_this_cycle;
                if (target_steps_remaining < 0) target_steps_remaining = 0;
            }
        }

        // --- CONTEXT-DRIVEN MULTI-PAGE OLED UI RENDERING (~120ms rate) ---
        display_refresh_counter++;
        if (display_refresh_counter >= 4) {
            display_refresh_counter = 0;

            char line_buf[32];
            sh1107_clear();

            // --- UI PAGE 2: ACTION / TARGETED MOVE OVERRIDE ---
            if (targeted_mode_active) {
                sh1107_draw_string(0, 0, "=== ACTION / WEED ===");
                sh1107_draw_string(2, 0, "STATUS: TARGETED MOVE");
                sh1107_draw_string(4, 0, "TARGET: 500 STEPS");
                
                snprintf(line_buf, sizeof(line_buf), "REMAIN: %ld STEPS", (long)target_steps_remaining);
                sh1107_draw_string(6, 0, line_buf);
            }
            // --- UI PAGE 3: SENSOR / WARNING OVERRIDE ---
            else if (sensor_slowdown_active) {
                sh1107_draw_string(0, 0, "=== SENSOR / ALERT ===");
                sh1107_draw_string(2, 0, "STATUS: LEAF/WEED CHECK");
                
                snprintf(line_buf, sizeof(line_buf), "POT SPD: %lu HZ", (unsigned long)potentiometer_target_freq);
                sh1107_draw_string(4, 0, line_buf);
                
                sh1107_draw_string(6, 0, "ACTION: CREEP SLOWDOWN");
            }
            // --- UI PAGE 1: DEFAULT / CORE DASHBOARD ---
            else {
                sh1107_draw_string(0, 0, "GREENHOUSE ROVER");
                sh1107_draw_string(2, 0, "MODE: AUTO CRUISE");
                
                snprintf(line_buf, sizeof(line_buf), "DIR:  %s", current_dir ? "FWD" : "REV");
                sh1107_draw_string(4, 0, line_buf);

                snprintf(line_buf, sizeof(line_buf), "FREQ: %lu HZ", (unsigned long)current_freq);
                sh1107_draw_string(6, 0, line_buf);
            }

            sh1107_refresh();
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
