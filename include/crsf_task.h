#pragma once

#include "utils.h"
#include "crsf_types.h"
#include "telemetry_task.h"

void crsf_task_rx(void *pvParameters);
void crsf_task_tx(void *pvParameters);
void get_servo_data(servo_data_t *data);
void crsf_init();

void crsf_send_battery_packet(uint16_t voltage_v_times_10, uint16_t current_a_times_10, uint32_t fuel_mah, uint8_t percent);
void crsf_send_temp(int16_t temp_celsius);
void crsf_send_rpm(uint16_t rpm);
void crsf_send_attitude(int16_t pitch_deg, int16_t roll_deg, int16_t yaw_deg);
void crsf_send_gps_packet(const crsf_telemetry_gps_t *gps_payload);
void forward_gps_to_elrs(const ubx_nav_pvt_t *u_blox_data, crsf_telemetry_gps_t *elrs_data);
