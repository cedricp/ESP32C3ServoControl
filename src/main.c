#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" 
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "gyro_task.h"

#include "server.h"
#include "utils.h"
#include "crsf_task.h"
#include "types.h"

#include "pid.h"

#define ONBOARD_LED_PIN     8
#define PAIRING_BUTTON_PIN  9

// #define DEBUG_STACK 1

const int servo_gpios[NUM_SERVOS] = {5, 6, 7, 10, 2, 0};

const uint32_t test_sequence_us_1[NUM_SERVOS] = {990, 1000, 2000, 1000, 1000, 1000};
const uint32_t test_sequence_us_2[NUM_SERVOS] = {1500, 1500, 2000, 1500, 1000, 1500};
const uint32_t test_sequence_us_3[NUM_SERVOS] = {2000, 2000, 2000, 2000, 1000, 2000};

PID_Config_t pidRoll  = { .Kp = 0.5f, .Ki = 0.0f, .Kd = 0.0001f, 250.f, .integralAcc = 0.0, .prevMeasuredRate = 0.0, .invert = 0 };
PID_Config_t pidPitch = { .Kp = 0.6f, .Ki = 0.0f, .Kd = 0.0001f, 150.f, .integralAcc = 0.0, .prevMeasuredRate = 0.0, .invert = 0 };
PID_Config_t pidYaw   = { .Kp = 0.7f, .Ki = 0.0f, .Kd = 0.0002f, 120.f, .integralAcc = 0.0, .prevMeasuredRate = 0.0, .invert = 0 };

PID_Config_t* get_pid_roll(void) { return &pidRoll; }
PID_Config_t* get_pid_pitch(void) { return &pidPitch; }
PID_Config_t* get_pid_yaw(void) { return &pidYaw; }

int g_master_gain_channel = 5;
int g_ouput_mapping[NUM_SERVOS] = {0, 1, 2, 3, 4, 5};
uint32_t g_failsafe_us[NUM_SERVOS] = {1500, 1500, 1000, 1500, 1500, 1500};
bool g_invert_channel[NUM_SERVOS] = {0, 0, 0, 0, 0, 0};

TaskHandle_t servo_task_handle = NULL;
TaskHandle_t crsf_task_handle = NULL;
TaskHandle_t slow_button_task_handle = NULL;
TaskHandle_t gyro_task_handle = NULL;

void writeFailsafeToNVS(char* start_byte, char* checksum_byte) {
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t size = checksum_byte - start_byte + 1;

    checksum_byte[0] = crsf_crc8((uint8_t*)start_byte, size-1);

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;

    err = nvs_set_blob(my_handle, "fs_config", start_byte, size);
    if (err == ESP_OK) {
        nvs_commit(my_handle); // Validation de la gravure en Flash
        printf("Failsafe configuration reset to defaults and saved to NVS.\n");
    } else {
        printf("Error writing failsafe configuration to NVS: %s\n", esp_err_to_name(err));
    }

    nvs_close(my_handle);
}

void readFailsafeFromNVS(char* start_byte, char* checksum_byte) {
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t required_size = checksum_byte - start_byte + 1;

    failsafe_config_t config;

    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        err = nvs_get_blob(my_handle, "fs_config", &config, &required_size);
        uint8_t calculated_crc = crsf_crc8((uint8_t*)&config.channels, required_size-1);

        if (err == ESP_OK && config.checksum == calculated_crc) {
            printf("Failsafe configuration loaded from NVS.\n");
        } else {
            printf("No failsafe configuration found in NVS, using defaults.\n");
        }
        nvs_close(my_handle);
    } else {
        printf("Error opening NVS handle, creating defaults: %s\n", esp_err_to_name(err));
        writeFailsafeToNVS(start_byte, checksum_byte);
    }
}

void blink_led(int times, int delay_ms, bool finish_lit) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(ONBOARD_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(ONBOARD_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    if (finish_lit) {
        gpio_set_level(ONBOARD_LED_PIN, 0);
    } else {
        gpio_set_level(ONBOARD_LED_PIN, 1);
    }
}

// ==========================================
// Failsafe button task
// ==========================================
void slow_button_task(void *pvParameters) {
    // 1. Configure the input pullup
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PAIRING_BUTTON_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE // No interrupts needed!
    };
    gpio_config(&io_conf);

   gpio_config_t io_conf2 = {
        .pin_bit_mask = (1ULL << ONBOARD_LED_PIN),
        .mode         = GPIO_MODE_OUTPUT,        // Set as output
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf2); 

    int stable_count = 0;
    bool button_pressed = false;

    while (1) {
        if (gpio_get_level(PAIRING_BUTTON_PIN) == 0) {
            stable_count++;
            if (stable_count >= 2) { // Must be low for 2 consecutive reads (approx 60ms)
                if (!button_pressed) {
                    button_pressed = true;
                    printf("Button pressed! Entering pairing mode...\n");
                    if (!server_is_running()) {
                        start_webserver();
                        blink_led(4, 200, true); // Visual feedback for button press
                    } else {
                        stop_webserver();
                        blink_led(4, 200, false); // Visual feedback for button press
                    }
                }
            }
        } else {
            stable_count = 0;
            button_pressed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void test_sequence(const uint32_t *sequence) {
    for (int i = 0; i < NUM_SERVOS; i++) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(sequence[i]));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

uint16_t computeAxis(PID_Config_t* pid, int16_t stick_us, float gyro_value, float master_gain, float dt)
{
    float targetRate = mapStickToRate(stick_us, pid->maxRateDegs, 5);
    float stickInput = nomaliseStick(stick_us);
    float axis_correction = computeAxisPID(stickInput, targetRate, gyro_value, dt, master_gain, pid->maxRateDegs, pid);
    return (axis_correction + 1) * 500. + 1000.;
}

// ==========================================
// Servo managemenent task
// ==========================================
void servo_update_task(void *pvParameters) {
    servo_data_t rx_data;
    gyro_data_t gyro_data;
    bool radio_init = false;

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_14_BIT,
        .freq_hz          = 50,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < NUM_SERVOS; i++) {
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = (ledc_channel_t)i,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = servo_gpios[i],
            .duty           = us_to_ledc_duty(1500), // Neutral at boot
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    test_sequence(test_sequence_us_1);
    vTaskDelay(pdMS_TO_TICKS(500));
    test_sequence(test_sequence_us_2);
    vTaskDelay(pdMS_TO_TICKS(500));
    test_sequence(test_sequence_us_3);
    vTaskDelay(pdMS_TO_TICKS(500));

    static int64_t last_time = 0;
    int64_t servo_timer = esp_timer_get_time();
    float master_gain = 0.3f;
    int64_t output_timer = esp_timer_get_time();


    while (1) {
        get_servo_data(&rx_data);
        get_gyro_data(&gyro_data);

        
        if (rx_data.valid && g_master_gain_channel >= 0 && g_master_gain_channel < 8) {
            // Update master gain
            master_gain = ((float)rx_data.us_values[g_master_gain_channel] - 1000.0f) / 1000.0f;
        }

        if (gyro_data.valid) {
            int64_t now = esp_timer_get_time();
            if (last_time == 0) {
                last_time = now; // First iteration safety
            }

            float dt = (float)(now - last_time) / 1000000.0f;
            last_time = now;

            if (dt <= 0.0005f || dt > 0.020f) {
                dt = 0.02f; // Fallback nominal à 20 ms (1/50 Hz)
            }
            rx_data.us_values[0] = computeAxis(&pidRoll, rx_data.us_values[0], gyro_data.x, master_gain, dt);
            rx_data.us_values[1] = computeAxis(&pidPitch, rx_data.us_values[1], gyro_data.y, master_gain, dt);
            rx_data.us_values[3] = computeAxis(&pidYaw, rx_data.us_values[3], gyro_data.z, master_gain, dt);
        }
        
        int64_t delta = esp_timer_get_time() - servo_timer;
        if (delta > 20000) {
            // 50 Hz refresh (200 ms)
            servo_timer = esp_timer_get_time();
            if (rx_data.valid) {
                for (int i = 0; i < NUM_SERVOS; i++) {
                    uint16_t us = rx_data.us_values[g_ouput_mapping[i]];
                    
                    // Clamp to 1000 µs - 2000 µs
                    us = us < 1000 ? 1000 : us;
                    us = us > 2000 ? 2000 : us;

                    if (g_invert_channel[i]) {
                        us = 3000 - us;
                    }
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(us));
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
                }
                radio_init = true;
            } else if (radio_init) {
                // --- FAILSAFE MODE ---
                // No data since 250ms, set all servos to failsafe values
                for (int i = 0; i < NUM_SERVOS; i++) {
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(g_failsafe_us[i]));
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
                }
            }
        }
        if (esp_timer_get_time() - output_timer > 500000) {
            output_timer = esp_timer_get_time();
            printf("Servo values : %d %d %d %d %d %d\n", rx_data.us_values[0], rx_data.us_values[1], rx_data.us_values[2], rx_data.us_values[3], rx_data.us_values[4], rx_data.us_values[5]);
            printf("Gyro values : %f %f %f\n", gyro_data.x, gyro_data.y, gyro_data.z);
            printf("Master gain : %f channel : %d\n", master_gain, g_master_gain_channel);
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void app_main(void) {
    // Check EEPROM initialization
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    xTaskCreate(slow_button_task, "slow_button", 1024, NULL, 5, &slow_button_task_handle);
    xTaskCreate(crsf_rx_task, "crsf_rx", 2048, NULL, 15, &crsf_task_handle);
    xTaskCreate(servo_update_task, "servo_ctrl", 4096, NULL, 20, &servo_task_handle);
    xTaskCreate(gyro_control_task, "gyro", 4096, NULL, 21, &gyro_task_handle);

    blink_led(4, 200, false);

    start_webserver();

    // /!\ No delay here, we must send commands as soon as possible for ESC config

    vTaskDelay(3000);

#ifdef DEBUG_STACK
    while (1) {
        if (servo_task_handle != NULL) {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(servo_task_handle);
            
            printf("Remaining stack for Servo Task: %u words\n", (unsigned int)remaining_stack);
        }

        if (crsf_task_handle != NULL) {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(crsf_task_handle);
            
            printf("Remaining stack for Crsf Task: %u words\n", (unsigned int)remaining_stack);
        }

        if (slow_button_task_handle != NULL) {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(slow_button_task_handle);
            
            printf("Remaining stack for Slow Button Task: %u words\n", (unsigned int)remaining_stack);
        }

        if (gyro_task_handle != NULL) {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(gyro_task_handle);
            
            printf("Remaining stack for Gyro Button Task: %u words\n", (unsigned int)remaining_stack);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
#endif
}