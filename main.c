#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
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
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << DIR_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&dir_conf);
    gpio_set_level(DIR_PIN, 1); 

    // 2. Configure Button Pin with Internal Pull-Down
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&btn_conf);

    // 3. Configure LEDC Timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = 50,             
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 4. Configure LEDC Channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = LEDC_DUTY_VAL,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // 5. Initialize ADC
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_control = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_control, &adc1_handle));

    adc_oneshot_chan_cfg_t adc_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, POT_ADC_CHANNEL, &adc_config));

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
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}