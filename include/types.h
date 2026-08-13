#pragma once

#include <stdint.h>

#define NUM_SERVOS          6
#define NUM_CHANNELS        8

#define CRSF_UART_PORT      UART_NUM_0
#define CRSF_RX_PIN         3        
#define CRSF_TX_PIN         4         
#define CRSF_BAUD_RATE      420000
#define CRSF_TIMEOUT_MS     250

typedef struct {
    uint16_t us_values[NUM_CHANNELS];
    char valid;
} servo_data_t;

typedef struct {
    uint32_t channels[NUM_SERVOS];
    uint8_t checksum;
} failsafe_config_t;
