#pragma once


typedef struct {
    float rot_x, rot_y, rot_z;  // deg/s
    float rot_x_low, rot_y_low, rot_z_low;  // deg/s
    float ax, ay, az; // m/s^2
    float raw_ax, raw_ay, raw_az; // m/s^2
    char valid;
} gyro_data_t;

void gyro_control_task(void *pvParameters);
void get_gyro_data(gyro_data_t *data);

