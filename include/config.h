#pragma once

#include <hal/adc_types.h>

// CRSF UART configuration
#define CRSF_UART_PORT      UART_NUM_0
#define CRSF_RX_PIN         GPIO_NUM_3
#define CRSF_TX_PIN         GPIO_NUM_4
#define CRSF_BAUD_RATE      420000
#define CRSF_TIMEOUT_MS     250
#define NUM_CRSF_CHANNELS   8

// GPS UART configuration
#define GPS_UART_PORT      UART_NUM_1
#define GPS_TX_PIN         GPIO_NUM_NC
#define GPS_RX_PIN         GPIO_NUM_5
#define GPS_BAUD_RATE      115200

// PWM configuration
#define NUM_PWM_OUPUTS      5
#define PWM_OUTPUT_PINS     GPIO_NUM_6, GPIO_NUM_7, GPIO_NUM_8, GPIO_NUM_9, GPIO_NUM_10

// I2C configuration for MPU6500
#define I2C_SDA_PIN         GPIO_NUM_20
#define I2C_SCL_PIN         GPIO_NUM_21
#define I2C_INT_PIN         GPIO_NUM_1
#define I2C_POWER_PIN       GPIO_NUM_2

// Voltmeter configuration (GPIO_NUM_0 is ADC1 channel 0)
#define ADC_VOLTMETER_PIN   ADC_CHANNEL_0

// ESC Software serial RX pin
#define ESC_RX_GPIO         GPIO_NUM_0
#define ESC_BITRATE         115200