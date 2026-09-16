#pragma once

typedef struct {
    uint8_t  temperature;       // °C
    uint32_t voltage_mv;        // cVolts
    uint32_t current_ma;        // cAmpères
    uint16_t mah;               // mAh
    uint32_t erpm;              // Tours/min électriques
} esc_telemetry_t;

void esc_telemetry_task(void *pvParameters);