#include <stdio.h>
<<<<<<< HEAD
#include <string.h>
=======
>>>>>>> c31a64b7c0a5075f5808fa7a79710590e8c9d2d5
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
<<<<<<< HEAD
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "sh1107.h"

#define STEP_PIN          GPIO_NUM_2    
#define DIR_PIN           GPIO_NUM_4    
#define DIR_BUTTON_PIN    GPIO_NUM_5    // Button 1: Toggles Direction
#define MOVE_BUTTON_PIN   GPIO_NUM_19   // Button 2: Triggers Targeted Step Count
#define POT_ADC_CHANNEL   ADC_CHANNEL_7 

#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO    (STEP_PIN)
#define LEDC_CHANNEL      LEDC_CHANNEL_0
#define LEDC_DUTY_RES     LEDC_TIMER_8_BIT  
#define LEDC_DUTY_VAL     128               

#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ 100000

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
=======
#include "esp_adc/adc_oneshot.h"

#define STEP_PIN        GPIO_NUM_2    
#define DIR_PIN         GPIO_NUM_4    
#define BUTTON_PIN      GPIO_NUM_5    
#define POT_ADC_CHANNEL ADC_CHANNEL_7 

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO  (STEP_PIN)
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_8_BIT  
#define LEDC_DUTY_VAL   128               

void app_main(void) {
    // 1. Configure Direction Pin
>>>>>>> c31a64b7c0a5075f5808fa7a79710590e8c9d2d5
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&dir_conf);
    gpio_set_level(DIR_PIN, 1); 

<<<<<<< HEAD
    uint64_t pin_mask = (1ULL << DIR_BUTTON_PIN) | (1ULL << MOVE_BUTTON_PIN);
    gpio_config_t btn_conf = {
        .pin_bit_mask = pin_mask,
=======
    // 2. Configure Button Pin with Internal Pull-Down
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
>>>>>>> c31a64b7c0a5075f5808fa7a79710590e8c9d2d5
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

<<<<<<< HEAD
    // 2. Configure LEDC PWM Timer & Channel
=======
    // 3. Configure LEDC Timer
>>>>>>> c31a64b7c0a5075f5808fa7a79710590e8c9d2d5
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 50,             
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

<<<<<<< HEAD
=======
    // 4. Configure LEDC Channel
>>>>>>> c31a64b7c0a5075f5808fa7a79710590e8c9d2d5
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
<<<<<<< HEAD
        .gpio_num       = STEP_PIN,
=======
        .gpio_num       = LEDC_OUTPUT_IO,
>>>>>>> c31a64b7c0a5075f5808fa7a79710590e8c9d2d5
        .duty           = LEDC_DUTY_VAL,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

<<<<<<< HEAD
    // 3. Initialize ADC for Potentiometer
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_control = { .unit_id = ADC_UNIT_1 };
=======
    // 5. Initialize ADC
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_control = {
        .unit_id = ADC_UNIT_1,
    };
>>>>>>> c31a64b7c0a5075f5808fa7a79710590e8c9d2d5
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_control, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &adc_config));

<<<<<<< HEAD
    // 4. Initialize I2C and SH1107 OLED Display
    ESP_ERROR_CHECK(i2c_master_init());
    sh1107_init(I2C_NUM_0);

    ESP_LOGI(TAG, "Greenhouse Rover Firmware Booted Successfully.");

    int current_dir = 1;       
    int target_dir = 1;        
    int last_dir_btn_state = 0;
    int last_move_btn_state = 0;
    int adc_raw = 0;
    
    uint32_t current_freq = 50; 
    int ramp_step = 40;        

    int32_t target_steps_remaining = 0; 
    bool targeted_mode_active = false;  
    int display_refresh_counter = 0;

    while (true) {
        // Read potentiometer speed scaling smoothly
        adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &adc_raw);
        uint32_t potentiometer_target_freq = 50 + (adc_raw / 2); 

        // --- BUTTON 1: Direction Toggle (GPIO 5) ---
        int dir_btn_state = gpio_get_level(DIR_BUTTON_PIN);
        if (dir_btn_state == 1 && last_dir_btn_state == 0) {
            target_dir = !target_dir;             
            ESP_LOGI(TAG, "Direction change requested.");
        }
        last_dir_btn_state = dir_btn_state;

        // --- BUTTON 2: Targeted Countdown Trigger (GPIO 19) ---
        int move_btn_state = gpio_get_level(MOVE_BUTTON_PIN);
        if (move_btn_state == 1 && last_move_btn_state == 0) {
            if (!targeted_mode_active) {
                target_steps_remaining = 500; 
                targeted_mode_active = true;
                ESP_LOGI(TAG, "Targeted countdown initiated: 500 steps.");
            }
        }
        last_move_btn_state = move_btn_state;

        // --- FREQUENCY TARGET CALCULATION ---
        uint32_t effective_target_freq = potentiometer_target_freq;

        if (current_dir != target_dir) {
            effective_target_freq = 0; // Zero-crossing brake
        } else if (targeted_mode_active && target_steps_remaining <= 0) {
            effective_target_freq = 0;
            if (current_freq == 0) {
                targeted_mode_active = false; 
                ESP_LOGI(TAG, "Target reached. Resuming free-run.");
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

        // --- THROTTLED OLED UI RENDERING (~120ms rate) ---
        display_refresh_counter++;
        if (display_refresh_counter >= 4) {
            display_refresh_counter = 0;

            char line_buf[32];
            sh1107_clear();

            sh1107_draw_string(0, 0, targeted_mode_active ? "MODE: TARGETED" : "MODE: FREE-RUN");
            
            snprintf(line_buf, sizeof(line_buf), "DIR: %s", current_dir ? "FWD" : "REV");
            sh1107_draw_string(2, 0, line_buf);

            snprintf(line_buf, sizeof(line_buf), "FREQ: %lu HZ", (unsigned long)current_freq);
            sh1107_draw_string(4, 0, line_buf);

            snprintf(line_buf, sizeof(line_buf), "STEPS: %ld", (long)target_steps_remaining);
            sh1107_draw_string(6, 0, line_buf);

            sh1107_refresh();
=======
    printf("Rover Stepper Initialized with Graceful Reversal Ramping.\n");

    int current_dir = 1;       
    int target_dir = 1;        // Desired direction based on button clicks
    int last_btn_state = 0;    
    int adc_raw = 0;
    
    uint32_t current_freq = 50; 
    int ramp_step = 40;        // Acceleration/Deceleration rate

    while (true) {
        if (adc_oneshot_read(adc1_handle, POT_ADC_CHANNEL, &adc_raw) == ESP_OK) {
            uint32_t potentiometer_target_freq = 50 + (adc_raw / 2); 

            // Handle Latching Direction Button (Registers the command change)
            int btn_state = gpio_get_level(BUTTON_PIN);
            if (btn_state == 1 && last_btn_state == 0) {
                target_dir = !target_dir;             
                printf("Reverse command received! Decelerating to 0 before flipping direction...\n");
            }
            last_btn_state = btn_state;

            // --- REVERSAL RAMP STATE MACHINE ---
            uint32_t effective_target_freq = potentiometer_target_freq;

            if (current_dir != target_dir) {
                // If we want a different direction than what is currently running,
                // we must force our intermediate target frequency down to 0 first!
                effective_target_freq = 0;
            }

            // Standard Ramping Logic toward the effective target
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

            // If we have successfully ramped down to 0 while a direction change was pending:
            if (current_freq == 0 && current_dir != target_dir) {
                current_dir = target_dir;               // Update current running direction
                gpio_set_level(DIR_PIN, current_dir);   // Safely flip the physical pin at zero speed
                printf("Motor stopped. Flipping DIR pin to %d and accelerating...\n", current_dir);
            }

            // Prevent setting LEDC frequency to 0 (which can throw errors or lock the timer), 
            // so we set a minimum floor of 1 Hz during full stop, or update normally.
            if (current_freq > 0) {
                ledc_set_freq(LEDC_MODE, LEDC_TIMER, current_freq);
            }

            printf("Target Dir: %d | Run Dir: %d | Freq: %lu Hz\n", target_dir, current_dir, current_freq);
>>>>>>> c31a64b7c0a5075f5808fa7a79710590e8c9d2d5
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}
