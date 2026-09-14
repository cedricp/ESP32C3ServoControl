#pragma once

#include <stdint.h>
#include "esp_adc/adc_oneshot.h"

typedef enum  {
    BATT_MAIN_ADC_NUM = ADC_CHANNEL_0,
    BATT_NUM
} battery_type_t;

typedef struct {
    uint32_t main_batt_voltage_mv;
    uint8_t main_batt_percent;
} batt_stat_t;


void power_task(void *pvParameters);
void calibrateVoltmeterSensor(uint8_t channel, float actualValue);
void power_init();