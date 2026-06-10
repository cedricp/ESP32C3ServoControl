#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" 
#include "driver/uart.h"
#include "driver/ledc.h"

#include "nvs_flash.h"
#include "nvs.h"


#define CRSF_UART_PORT      UART_NUM_1
#define CRSF_RX_PIN         3        
#define CRSF_TX_PIN         4         
#define CRSF_BAUD_RATE      420000
#define NUM_SERVOS          6
#define ONBOARD_LED_PIN     8
#define PAIRING_BUTTON_PIN  9
#define CRSF_TIMEOUT_MS     250

const int servo_gpios[NUM_SERVOS] = {5, 6, 7, 10, 2, 0};
const uint32_t failsafe_us[NUM_SERVOS] = {1500, 1500, 990, 1500, 1500, 1500};
QueueHandle_t servo_queue = NULL;

struct FailsafePayload {
    int values[NUM_SERVOS];
    uint16_t checksum;
} failsafeData;

typedef struct {
    uint16_t us_values[NUM_SERVOS];
} servo_data_t;

typedef struct {
    uint32_t channels[NUM_SERVOS];
    uint8_t checksum;
} failsafe_config_t;

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}

void writeFailsafeToNVS() {
    nvs_handle_t my_handle;
    esp_err_t err;

    // TODO: Replace with actual failsafe values from your application
    failsafe_config_t config;
    for (int i = 0; i < NUM_SERVOS; i++) {
        config.channels[i] = failsafe_us[i];
    }

    config.checksum = crsf_crc8((uint8_t*)&config.channels, NUM_SERVOS * sizeof(uint32_t));

    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return;

    err = nvs_set_blob(my_handle, "fs_config", &config, sizeof(failsafe_config_t));
    if (err == ESP_OK) {
        nvs_commit(my_handle); // Validation de la gravure en Flash
        printf("Failsafe configuration reset to defaults and saved to NVS.\n");
    } else {
        printf("Error writing failsafe configuration to NVS: %s\n", esp_err_to_name(err));
    }

    nvs_close(my_handle);
}


void readFailsafeFromNVS() {
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t required_size = sizeof(failsafe_config_t);

    failsafe_config_t config;

    err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err == ESP_OK) {
        err = nvs_get_blob(my_handle, "fs_config", &config, &required_size);
        uint8_t calculated_crc = crsf_crc8((uint8_t*)&config.channels, NUM_SERVOS * sizeof(uint32_t));

        if (err == ESP_OK && config.checksum == calculated_crc) {
            printf("Failsafe configuration loaded from NVS.\n");
        } else {
            printf("No failsafe configuration found in NVS, using defaults.\n");
        }
        nvs_close(my_handle);
    }
}

uint32_t us_to_ledc_duty(uint32_t us) {
    return (us * 16384) / 20000;
}

void blink_led(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        gpio_set_level(ONBOARD_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(ONBOARD_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
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
        // 2. Read the active-low button (0 = pressed, 1 = released)
        if (gpio_get_level(PAIRING_BUTTON_PIN) == 0) {
            stable_count++;
            if (stable_count >= 2) { // Must be low for 2 consecutive reads (approx 60ms)
                if (!button_pressed) {
                    button_pressed = true;
                    printf("Button pressed! Entering pairing mode...\n");
                    blink_led(5, 200); // Visual feedback for button press
                }
            }
        } else {
            stable_count = 0;
            button_pressed = false;
        }

        // 3. The secret sauce: Sleep for 30ms. 
        // This yields ALL CPU execution time back to your CRSF and Servo tasks.
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// ==========================================
// CROSSFIRE UART task
// ==========================================
void crsf_rx_task(void *pvParameters) {
    uint8_t buffer[64];
    uint16_t crsf_channels[8];
    servo_data_t tx_data;

    // Init UART for CRSF reception
    uart_config_t uart_config = {
        .baud_rate = CRSF_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(CRSF_UART_PORT, &uart_config);
    uart_set_pin(CRSF_UART_PORT, CRSF_TX_PIN, CRSF_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(CRSF_UART_PORT, 1024, 0, 0, NULL, 0);

    while (1) {
        int len = uart_read_bytes(CRSF_UART_PORT, buffer, 1, pdMS_TO_TICKS(20));
        
        if (len > 0 && buffer[0] == 0xC8) {
            uart_read_bytes(CRSF_UART_PORT, &buffer[1], 1, pdMS_TO_TICKS(10));
            uint8_t packet_len = buffer[1];

            if (packet_len <= 62) {
                uart_read_bytes(CRSF_UART_PORT, &buffer[2], packet_len, pdMS_TO_TICKS(20));

                if (buffer[2] == 0x16) {
                    uint8_t computed_crc = crsf_crc8(&buffer[2], packet_len - 1);
                    if (computed_crc == buffer[packet_len + 1]) {
                        
                        // Channel decoding (11 bits)
                        uint8_t *payload = &buffer[3];
                        crsf_channels[0] = ((uint16_t)payload[0]        | (uint16_t)payload[1] << 8)                                    & 0x07FF;
                        crsf_channels[1] = ((uint16_t)payload[1] >> 3   | (uint16_t)payload[2] << 5)                                    & 0x07FF;
                        crsf_channels[2] = ((uint16_t)payload[2] >> 6   | (uint16_t)payload[3] << 2  | (uint32_t)payload[4] << 10)     & 0x07FF;
                        crsf_channels[3] = ((uint16_t)payload[4] >> 1   | (uint16_t)payload[5] << 7)                                    & 0x07FF;
                        crsf_channels[4] = ((uint16_t)payload[5] >> 4   | (uint16_t)payload[6] << 4)                                    & 0x07FF;
                        crsf_channels[5] = ((uint16_t)payload[6] >> 7   | (uint16_t)payload[7] << 1  | (uint32_t)payload[8] << 9)      & 0x07FF;
                        crsf_channels[6] = ((uint16_t)payload[8] >> 2   | (uint16_t)payload[9] << 6)                                    & 0x07FF;
                        crsf_channels[7] = ((uint16_t)payload[9] >> 5   | (uint32_t)payload[10] << 3 | (uint32_t)payload[11] << 11)    & 0x07FF;

                        
                        for (int i = 0; i < NUM_SERVOS; i++) {
                            tx_data.us_values[i] = ((crsf_channels[i] - 992) * 5) / 8 + 1500;
                        }
                        
                        // Send to servo queue
                        xQueueSend(servo_queue, &tx_data, 0);

                    }
                }
            }
        }
    }
}

// ==========================================
// Servo managemenent task
// ==========================================
void servo_update_task(void *pvParameters) {
    servo_data_t rx_data;

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
            .duty           = us_to_ledc_duty(1500), // Neutre au boot
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
    }

    while (1) {
        if (xQueueReceive(servo_queue, &rx_data, pdMS_TO_TICKS(CRSF_TIMEOUT_MS)) == pdTRUE) {
            
            for (int i = 0; i < NUM_SERVOS; i++) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(rx_data.us_values[i]));
                ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
            }
        } else {
            // --- FAILSAFE MODE ---
            // No data since 250ms, set all servos to failsafe values
            for (int i = 0; i < NUM_SERVOS; i++) {
                ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(failsafe_us[i]));
                ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
            }
        }
    }
}

TaskHandle_t servo_task_handle = NULL;
TaskHandle_t crsf_task_handle = NULL;
TaskHandle_t slow_button_task_handle = NULL;

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    servo_queue = xQueueCreate(2, sizeof(servo_data_t));

    if (servo_queue != NULL) {
        xTaskCreate(crsf_rx_task, "crsf_rx", 3072, NULL, 10, &crsf_task_handle);
        xTaskCreate(servo_update_task, "servo_ctrl", 2048, NULL, 9, &servo_task_handle);
    }

    xTaskCreate(slow_button_task, "slow_button", 1024, NULL, 1, &slow_button_task_handle);

    bool led_state = false;
    for(int i = 0; i < 10; i++) {
        gpio_set_level(ONBOARD_LED_PIN, led_state);
        led_state = !led_state;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    while (1) {
        if (servo_task_handle != NULL) {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(servo_task_handle);
            
            printf("Remaining stack for Servo Task: %u words\n", (unsigned int)remaining_stack);
        }

        if (crsf_task_handle != NULL) {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(crsf_task_handle);
            
            printf("Remaining stack for Crsf Task: %u words\n", (unsigned int)remaining_stack);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}