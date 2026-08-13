#pragma once


typedef struct {
    float x, y, z;  // deg/s
    char valid;
} gyro_data_t;

void gyro_control_task(void *pvParameters);
void get_gyro_data(gyro_data_t *data);

