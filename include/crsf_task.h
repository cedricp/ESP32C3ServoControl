#pragma once

#include "utils.h"

void crsf_task_rx(void *pvParameters);
void crsf_task_tx(void *pvParameters);
void get_servo_data(servo_data_t *data);
void crsf_init();