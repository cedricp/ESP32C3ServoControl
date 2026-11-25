#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "gyro_task.h"

#include "server.h"
#include "utils.h"
#include "crsf_task.h"

#include "pid.h"

// #define DEBUG_GYRO
// #define DEBUG_STACK

/*
 * MPU 6500 Axis reminder
 * Accel +X -> front
 * Accel +Y -> left
 * Accel +Z -> up
 * Gyro +X -> roll right
 * Gyro +Y -> pitch down
 * Gyro +Z -> yaw left
 */


struct system_data_t sysdata = {
    .pid_roll = {0},
    .pid_pitch = {0},
    .pid_yaw = {0},
    .servo_gpios = {PWM_OUTPUT_PINS},
    .attitude_correction_rp = {0.f, 0.f},
    .motor_magnets_count = 14,
    .crash_reasons = {0, 0, 0, 0},
    .attitude = {0.f, 0.f, 0.f},
    .master_kp_gain_channel = DEFAULT_CHANNEL_KP_GAIN,
    .master_kd_gain_channel = DEFAULT_CHANNEL_KD_GAIN,
    .instant_trim_channel   = DEFAULT_CHANNEL_INSTANT_TRIM,
    .flightmode_channel     = DEFAULT_CHANNEL_FLIGHTMODE,
    .flightmode = 0,
    .elrs_armed = false,
    .elrs_data_valid = false,
    .crash_g_threshold = 16.f,
    .level_mode_gain = 5.0f,
    .tpa_value = 0.0f,
    .esc_temperature = 0,
    .gyro_temperature = 0,
    .gyro_calibrated = 0,
    .gyrotask_launched = 0
};  

TaskHandle_t servo_task_handle     = NULL;
TaskHandle_t crsf_rx_task_handle   = NULL;
TaskHandle_t actions_task_handle   = NULL;
TaskHandle_t telemetry_task_handle = NULL;
TaskHandle_t gyro_sv_task_handle   = NULL;


PID_Config_t *get_pid_roll(void)
{
     return &sysdata.pid_roll;
}

PID_Config_t *get_pid_pitch(void)
{
     return &sysdata.pid_pitch;
}

PID_Config_t *get_pid_yaw(void)
{
    return &sysdata.pid_yaw;
}

static inline uint32_t __attribute__((always_inline)) us_to_ledc_duty(uint32_t us)
{
    // return (us * 16384) / 20000;
    return (us * ((1 << LEDC_TIMER_14_BIT) - 1)) / LEDC_PERIOD_US;
}


void calibrate_roll(void)
{
    sysdata.attitude_correction_rp[0] = -sysdata.attitude.roll_deg;
    nvs_save_struct("attitude_corr", &sysdata.attitude_correction_rp, sizeof(sysdata.attitude_correction_rp));
}

void calibrate_pitch(void)
{
    sysdata.attitude_correction_rp[1] = -sysdata.attitude.pitch_deg;
    nvs_save_struct("attitude_corr", &sysdata.attitude_correction_rp, sizeof(sysdata.attitude_correction_rp));
}

static inline bool is_elrs_armed()
{
    return sysdata.elrs_armed;
}

static void init_pid(PID_Config_t *pid, float Kp, float Ki, float Kd, float max_rate_degs)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->max_rate_degs = max_rate_degs;
    pid->integral_acc = 0.0f;
    pid->prev_measured_rate = 0.0f;
}

void save_pid_config()
{
    if (nvs_save_struct("pid_roll", &sysdata.pid_roll, sizeof(sysdata.pid_roll)) != ESP_OK)
    {
        printf("Error saving pid_roll\n");
    }
    if (nvs_save_struct("pid_pitch", &sysdata.pid_pitch, sizeof(sysdata.pid_pitch)) != ESP_OK)
    {
        printf("Error saving pid_pitch\n");
    }
    if (nvs_save_struct("pid_yaw", &sysdata.pid_yaw, sizeof(sysdata.pid_yaw)) != ESP_OK)
    {
        printf("Error saving pid_yaw\n");
    }
    if (nvs_save_struct("kp_channel", &sysdata.master_kp_gain_channel, sizeof(sysdata.master_kp_gain_channel)) != ESP_OK)
    {
        printf("Error saving master_kp_gain_channel\n");
    }
    if (nvs_save_struct("kd_channel", &sysdata.master_kd_gain_channel, sizeof(sysdata.master_kd_gain_channel)) != ESP_OK)
    {
        printf("Error saving master_kd_gain_channel\n");
    }
    if (nvs_save_struct("fmode_channel", &sysdata.flightmode_channel, sizeof(sysdata.flightmode_channel)) != ESP_OK)
    {
        printf("Error saving flightmode_channel\n");
    }
    if (nvs_save_struct("accel_invert", &sysdata.invert_accel, sizeof(sysdata.invert_accel)) != ESP_OK)
    {
        printf("Error saving accel_invert\n");
    }
    if (nvs_save_struct("crashthreshold", &sysdata.crash_g_threshold, sizeof(sysdata.crash_g_threshold)) != ESP_OK){
        printf("Error saving crash_g_threshold\n");
    } 
    if (nvs_save_struct("tpa", &sysdata.tpa_value, sizeof(sysdata.tpa_value)) != ESP_OK){
        printf("Error saving tpa_value\n");
    } 
    if (nvs_save_struct("level_gain", &sysdata.level_mode_gain, sizeof(sysdata.level_mode_gain)) != ESP_OK){
        printf("Error saving level_mode_gain\n");
    } 
    printf("Saved config\n");
}

void load_pid_config()
{
    if (nvs_load_struct("pid_roll", &sysdata.pid_roll, sizeof(sysdata.pid_roll)) != ESP_OK)
    {
        printf("Error loading pid_roll\n");
    }
    if (nvs_load_struct("pid_pitch", &sysdata.pid_pitch, sizeof(sysdata.pid_pitch)) != ESP_OK)
    {
        printf("Error loading pid_pitch\n");
    }
    if (nvs_load_struct("pid_yaw", &sysdata.pid_yaw, sizeof(sysdata.pid_yaw)) != ESP_OK)
    {
        printf("Error loading pid_yaw\n");
    }
    if (nvs_load_struct("kp_channel", &sysdata.master_kp_gain_channel, sizeof(sysdata.master_kp_gain_channel)) != ESP_OK)
    {
        printf("Error loading master_kp_gain_channel\n");
    }
    if (nvs_load_struct("kd_channel", &sysdata.master_kd_gain_channel, sizeof(sysdata.master_kd_gain_channel)) != ESP_OK)
    {
        printf("Error loading master_kd_gain_channel\n");
    }
    if (nvs_load_struct("fmode_channel", &sysdata.flightmode_channel, sizeof(sysdata.flightmode_channel)) != ESP_OK)
    {
        printf("Error loading flightmode_channel\n");
    }
    if (nvs_load_struct("accel_invert", &sysdata.invert_accel, sizeof(sysdata.invert_accel)) != ESP_OK)
    {
        printf("Error loading accel_invert\n");
    }
    if (nvs_load_struct("crashthreshold", &sysdata.crash_g_threshold, sizeof(sysdata.crash_g_threshold)) != ESP_OK)
    {
        printf("Error loading crash_g_threshold\n");
    }
    if (nvs_load_struct("tpa", &sysdata.tpa_value, sizeof(sysdata.tpa_value)) != ESP_OK){
        printf("Error loading tpa_value\n");
    }
    if (nvs_load_struct("level_gain", &sysdata.level_mode_gain, sizeof(sysdata.level_mode_gain)) != ESP_OK){
        printf("Error loading level_mode_gain\n");
    }
}

void save_pwm_config()
{
    nvs_save_struct("pwm_mapping",   sysdata.ouput_mapping,  sizeof(sysdata.ouput_mapping));
    nvs_save_struct("pwm_invert",    sysdata.invert_channel, sizeof(sysdata.invert_channel));
    nvs_save_struct("pwm_failsafe",  sysdata.failsafe_us,    sizeof(sysdata.failsafe_us));
    nvs_save_struct("magnets_count", &sysdata.motor_magnets_count, sizeof(sysdata.motor_magnets_count));
}

void load_pwm_config()
{
    nvs_load_struct("pwm_mapping",   sysdata.ouput_mapping,  sizeof(sysdata.ouput_mapping));
    nvs_load_struct("pwm_invert",    sysdata.invert_channel, sizeof(sysdata.invert_channel));
    nvs_load_struct("pwm_failsafe",  sysdata.failsafe_us,    sizeof(sysdata.failsafe_us));
    nvs_load_struct("magnets_count", &sysdata.motor_magnets_count, sizeof(sysdata.motor_magnets_count));
}

void load_attitude_config()
{
    nvs_load_struct("attitude_corr", &sysdata.attitude_correction_rp, sizeof(sysdata.attitude_correction_rp));
}

void init_pid_factory()
{
    sysdata.master_kp_gain_channel = DEFAULT_CHANNEL_KP_GAIN;
    sysdata.master_kd_gain_channel = DEFAULT_CHANNEL_KD_GAIN;
    sysdata.flightmode_channel     = DEFAULT_CHANNEL_FLIGHTMODE;
    sysdata.instant_trim_channel   = DEFAULT_CHANNEL_INSTANT_TRIM;
    init_pid(&sysdata.pid_roll,  1.0f, 0.0f, 0.0001f, 250.f);
    init_pid(&sysdata.pid_pitch, 1.0f, 0.0f, 0.0001f, 150.f);
    init_pid(&sysdata.pid_yaw,   1.0f, 0.0f, 0.0002f, 120.f);
    sysdata.invert_accel[0] = false;
    sysdata.invert_accel[1] = false;
    sysdata.invert_accel[2] = false;
}

void init_pwm_factory()
{
    memcpy(sysdata.ouput_mapping,  (int[]){0, 1, 2, 3, 4}, sizeof(sysdata.ouput_mapping));
    memcpy(sysdata.invert_channel, (bool[]){0, 0, 0, 0, 0}, sizeof(sysdata.invert_channel));
    memcpy(sysdata.failsafe_us,    (int[]){1500, 1500, 1000, 1500, 1500}, sizeof(sysdata.failsafe_us));
}

void reset_crash(void)
{
    for (int i = 0; i < 4; i++)
    {
        sysdata.crash_reasons[i] = 0;
    }
    nvs_save_struct("crash", &sysdata.crash_reasons, sizeof(sysdata.crash_reasons));
}

void actions_task(void *pvParameters)
{
    nvs_load_struct("crash", &sysdata.crash_reasons, sizeof(sysdata.crash_reasons));

    uint8_t reboot_reason = (uint8_t)esp_reset_reason();

    if (reboot_reason > (uint8_t)ESP_RST_POWERON)
    {
        sysdata.crash_reasons[3] = sysdata.crash_reasons[2];
        sysdata.crash_reasons[2] = sysdata.crash_reasons[1];
        sysdata.crash_reasons[1] = sysdata.crash_reasons[0];
        sysdata.crash_reasons[0] = reboot_reason;
        nvs_save_struct("crash", &sysdata.crash_reasons, sizeof(sysdata.crash_reasons));
    }

    vTaskDelay(3000);

    for (int i = 0; i < 4; i++)
    {
        printf("Reset reason (recent to old) %d: %s\n", i, reset_reason_to_str(sysdata.crash_reasons[i]));
    }

    while (1)
    {
        if (sysdata.elrs_data_valid && is_elrs_armed() && server_is_running())
        {
            // Deactivate server when ELRS is armed
            stop_webserver();
            printf("Server stopped\n");
            vTaskDelay(3000);
        }
        else  if (sysdata.elrs_data_valid && !is_elrs_armed() && !server_is_running())
        {
            printf("Server started\n");
            start_webserver();
            vTaskDelay(3000);
        }
        else if (!sysdata.elrs_data_valid && !server_is_running())
        {
            printf("Server started\n");
            start_webserver();
            vTaskDelay(3000);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static inline uint16_t compute_axis_pwm(PID_Config_t *pid, float stick_normalized, float gyro_value, float gyro_value_low, float master_kp_gain, float master_kd_gain, float dt)
{
    float target_rate       = stick_normalized *  pid->max_rate_degs;
    float axis_correction   = compute_axis_pid(stick_normalized, target_rate, gyro_value, gyro_value_low, dt, master_kp_gain, master_kd_gain, pid, 1);
    
    return map_to_pwm(axis_correction);
}

inline static uint32_t process_motor_safety(uint32_t current_throttle_us, bool shock_detected) {
    static   motor_safety_state_t current_state = MOTOR_STATE_NORMAL;
    uint32_t pwm_us = 0;

    switch (current_state) {
        
        case MOTOR_STATE_NORMAL:
            if (shock_detected) {
                pwm_us = 1000;
                current_state = MOTOR_STATE_EMERGENCY_CUT;
            } else {
                pwm_us = current_throttle_us;
            }
            break;

        case MOTOR_STATE_EMERGENCY_CUT:
            pwm_us = 1000;
            
            if (current_throttle_us < 1020) {
                current_state = MOTOR_STATE_WAIT_THROTTLE_ZERO;
            }
            break;

        case MOTOR_STATE_WAIT_THROTTLE_ZERO:
            pwm_us = 1000;
            
            if (current_throttle_us > 1025) {
                current_state = MOTOR_STATE_NORMAL;
            }
            break;
    }
    return pwm_us;
}

inline static uint16_t apply_motor_thermal_protection(uint16_t pwm_value)
{
    if (sysdata.esc_temperature > 95)
    {
        return clampui(pwm_value, 1000, 1500);
    }
    else if (sysdata.esc_temperature > 85)
    {
        return clampui(pwm_value, 1000, 1800);
    }
    return pwm_value;
}

static void servo_pwm_init()
{
    // Init ledc timer for servos PWM generation
    ledc_timer_config_t ledc_timer = 
    {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz         = 50,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < NUM_PWM_OUPUTS; i++)
    {
        ledc_channel_config_t ledc_channel = 
        {
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = (ledc_channel_t)i,
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = sysdata.servo_gpios[i],
            .duty           = us_to_ledc_duty(1500), // Neutral at boot
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
        gpio_set_drive_capability(sysdata.servo_gpios[i], GPIO_DRIVE_CAP_2);
    }
}

static inline bool shock_detector(gyro_data_t* gyro_data)
{
    static float prev_ax = 0.0f, prev_ay = 0.0f, prev_az = 0.0f;

    // 1. Acceleration delta
    const float d_ax = gyro_data->ax - prev_ax;
    const float d_ay = gyro_data->ay - prev_ay;
    const float d_az = (gyro_data->az - prev_az) * 0.7f; // Lower Z for landing protection

    // Update for next cycle
    prev_ax = gyro_data->ax;
    prev_ay = gyro_data->ay;
    prev_az = gyro_data->az;

    const float jerk_factor = (d_ax * d_ax) + (d_ay * d_ay) + (d_az * d_az);

    const float gyro_sq = (gyro_data->ax * gyro_data->ax) + 
                          (gyro_data->ay * gyro_data->ay) + 
                          (gyro_data->az * gyro_data->az);

    return (jerk_factor > sysdata.crash_g_threshold) || (jerk_factor > (sysdata.crash_g_threshold * 0.4f) && gyro_sq > (480.0f*480.0f));
}

// ==========================================
// Servo managemenent task
// ==========================================
void servo_update_task(void *pvParameters)
{
    servo_data_t rx_data;
    gyro_data_t gyro_data;
    bool radio_init = false;

    static int64_t last_time = 0;
    int64_t servo_timer = esp_timer_get_time();
#ifdef DEBUG_GYRO
    int64_t output_timer = esp_timer_get_time();
#endif
    float master_kp_gain = 1.0f;
    float master_kd_gain = 1.0f;

#ifndef LEVEL_MODE_MAHONY
    get_gyro_data(&gyro_data);
    init_attitude(&attitude, gyro_data.raw_ax, gyro_data.raw_ay, gyro_data.raw_az);
#endif

    uint32_t ul_notified_value;

    while (1)
    {
        int64_t now = esp_timer_get_time();
        bool    gyro_failsafe = false;
        float   throttle_pid_attenuation = 1.0f;

        if (last_time == 0)
        {
            last_time = now; // First iteration safety
        }

        float dt = (now - last_time) * 1e-6f;
        last_time = now;

        if (dt <= 0.0005f || dt > 0.020f)
        {
            dt = 0.01f; // Fallback nominal à 20 ms (1/50 Hz)
        }

        BaseType_t xResult = xTaskNotifyWait(0x00, ULONG_MAX, &ul_notified_value, pdMS_TO_TICKS(5));
        if (xResult == pdTRUE)
        {
            get_gyro_data(&gyro_data);
        } else {
            gyro_data.valid = false;
        }

        get_servo_data(&rx_data);

        if (gyro_data.valid)
        {
#ifdef LEVEL_MODE_MAHONY
            // ~70us execution time
            mahony_update(gyro_data.rot_x, gyro_data.rot_y, gyro_data.rot_z, gyro_data.ax, gyro_data.ay, gyro_data.az, dt);
            mahony_get_euler(&sysdata.attitude);
#else
            compute_attitude(&sysdata.attitude, gyro_data.ax, gyro_data.ay, gyro_data.az, gyro_data.rot_x, gyro_data.rot_y, dt);
#endif
            sysdata.gyro_temperature = gyro_data.temp;

            const float throttle_normalized = rx_data.norm_values[DEFAULT_CHANNEL_THROTTLE];
            if (throttle_normalized > 0.0f)
            {
                // Attenuation of PID gains based on throttle position to reduce oscillations at high throttle
                // 0-50% no attenuation, 50-100% linear attenuation down to {sysdata.tpa_value x 100}% of original gain
                throttle_pid_attenuation = 1.0f - (throttle_normalized * sysdata.tpa_value);
            }
        }

        sysdata.elrs_data_valid = rx_data.valid;

        if (rx_data.valid)
        {
            if (sysdata.instant_trim_channel >= 0 && sysdata.instant_trim_channel < 12 && (rx_data.us_values[sysdata.instant_trim_channel] > 1600) )
            {
                if (sysdata.elrs_armed){
                    calibrate_pitch();
                } else {
                    gyro_calib();
                }
            }

            if (sysdata.master_kp_gain_channel >= 0 && sysdata.master_kp_gain_channel < 12)
            {
                // Update master kp gain
                master_kp_gain = (rx_data.norm_values[sysdata.master_kp_gain_channel] + 1.0f) * 0.5f; // Map from [-1, 1] to [0, 1]
                master_kp_gain *= throttle_pid_attenuation;
            }
            if (sysdata.master_kd_gain_channel >= 0 && sysdata.master_kd_gain_channel < 12)
            {
                // Update master kd gain
                master_kd_gain = (rx_data.norm_values[sysdata.master_kd_gain_channel] + 1.0f) * 0.5f; // Map from [-1, 1] to [0, 1]
                master_kd_gain *= throttle_pid_attenuation;
            }
            if (sysdata.flightmode_channel >= 0 && sysdata.flightmode_channel < 12 && sysdata.gyro_calibrated)
            {
                int fmv = rx_data.us_values[sysdata.flightmode_channel];
                if (fmv > 1700)
                {
                    sysdata.flightmode = FLIGHTMODE_LEVEL;
                }
                else if (fmv > 1300)
                {
                    sysdata.flightmode = FLIGHTMODE_STAB;
                }
                else
                {
                    sysdata.flightmode = FLIGHTMODE_FREE;
                }
            } else {
                sysdata.flightmode = FLIGHTMODE_FREE;
            }
            sysdata.elrs_armed = rx_data.us_values[4] > 1600;
        }

        // Gyro Pitch : -up +down
        // Gyro Roll : -left +right
        // Gyro Yaw : -right +left

        if (gyro_data.valid && rx_data.valid)
        {
            if (sysdata.flightmode == FLIGHTMODE_LEVEL)
            {
                const float attitude_pitch = sysdata.attitude.pitch_deg + sysdata.attitude_correction_rp[1];
                const float attitude_roll  = sysdata.attitude.roll_deg  + sysdata.attitude_correction_rp[0];

                float stick_input_roll  = rx_data.norm_values[DEFAULT_CHANNEL_AILERON];
                float target_angle_roll = stick_input_roll * 45.0f; // -45° à +45°
                float target_rate_roll  = sysdata.level_mode_gain * (target_angle_roll - attitude_roll);
                target_rate_roll        = clampf(target_rate_roll, -sysdata.pid_roll.max_rate_degs, sysdata.pid_roll.max_rate_degs);

                // We don't want pitch bump when the plane is inverted, so we only apply it when the roll angle is between -60° and +60°
                float pitch_bump = (attitude_roll < 60 && attitude_roll > -60) ? (1.0f - fast_cosf(deg_to_rad(attitude_roll))) * 20.0f : 0.0f;

                float stick_input_pitch  = rx_data.norm_values[DEFAULT_CHANNEL_ELEVATOR];
                float target_angle_pitch = (stick_input_pitch * 35.0f) - pitch_bump; // -35° à +35°
                float target_rate_pitch  = sysdata.level_mode_gain * (target_angle_pitch - attitude_pitch);
                target_rate_pitch        = clampf(target_rate_pitch, -sysdata.pid_pitch.max_rate_degs, sysdata.pid_pitch.max_rate_degs);

                rx_data.us_values[DEFAULT_CHANNEL_AILERON]  = map_to_pwm(compute_axis_pid(0, target_rate_roll, gyro_data.rot_x,  gyro_data.rot_x_low, dt, master_kp_gain, master_kd_gain, &sysdata.pid_roll, 0));
                rx_data.us_values[DEFAULT_CHANNEL_ELEVATOR] = map_to_pwm(compute_axis_pid(0, target_rate_pitch, gyro_data.rot_y, gyro_data.rot_y_low, dt, master_kp_gain, master_kd_gain, &sysdata.pid_pitch, 0));
                rx_data.us_values[DEFAULT_CHANNEL_RUDDER]   = compute_axis_pwm(&sysdata.pid_yaw, rx_data.norm_values[DEFAULT_CHANNEL_RUDDER], -gyro_data.rot_z, -gyro_data.rot_z_low, master_kp_gain, master_kd_gain, dt);
            }
        }
        else if (gyro_data.valid && !rx_data.valid)
        {
            // No RX data but gyro OK, attempt to save plane
            const float attitude_pitch = sysdata.attitude.pitch_deg + sysdata.attitude_correction_rp[1];
            const float attitude_roll  = sysdata.attitude.roll_deg  + sysdata.attitude_correction_rp[0];
            // Failsafe mode: no RX data, but gyro is valid. Try to keep plane flat and turning
            float target_angle_roll = 20.f;
            float target_rate_roll  = sysdata.level_mode_gain * (target_angle_roll - attitude_roll);
            target_rate_roll        = clampf(target_rate_roll, -sysdata.pid_roll.max_rate_degs, sysdata.pid_roll.max_rate_degs);
            
            float target_angle_pitch = -10.f;
            float target_rate_pitch  = sysdata.level_mode_gain * (target_angle_pitch - attitude_pitch);
            target_rate_pitch        = clampf(target_rate_pitch, -sysdata.pid_pitch.max_rate_degs, sysdata.pid_pitch.max_rate_degs);

            rx_data.us_values[DEFAULT_CHANNEL_AILERON]  = map_to_pwm(compute_axis_pid(0.f, target_rate_roll,  gyro_data.rot_x, gyro_data.rot_x_low, dt, master_kp_gain, master_kd_gain, &sysdata.pid_roll, 0));
            rx_data.us_values[DEFAULT_CHANNEL_ELEVATOR] = map_to_pwm(compute_axis_pid(0.f, target_rate_pitch, gyro_data.rot_y, gyro_data.rot_y_low, dt, master_kp_gain, master_kd_gain, &sysdata.pid_pitch, 0));
            rx_data.us_values[DEFAULT_CHANNEL_THROTTLE] = 1000; // Motor off
            rx_data.us_values[DEFAULT_CHANNEL_RUDDER]   = 1500; // Yaw neutral
            gyro_failsafe = true;
        }

        // Apply motor thermal protection based on ESC temperature
        rx_data.us_values[DEFAULT_CHANNEL_THROTTLE] = apply_motor_thermal_protection(rx_data.us_values[DEFAULT_CHANNEL_THROTTLE]);
        
        // Check ESC temperature and apply thermal protection if necessary
        rx_data.us_values[DEFAULT_CHANNEL_THROTTLE] = process_motor_safety(rx_data.us_values[DEFAULT_CHANNEL_THROTTLE], shock_detector(&gyro_data));

        if (!sysdata.elrs_armed)
        {
            // If not armed, force throttle to 0%
            rx_data.us_values[DEFAULT_CHANNEL_THROTTLE] = 1000;
        }

        if ((esp_timer_get_time() - servo_timer) > 20000)
        {
            // 50 Hz refresh (20 ms)
            servo_timer = esp_timer_get_time();
            if (rx_data.valid || gyro_failsafe)
            {
                for (int i = 0; i < NUM_PWM_OUPUTS; i++)
                {
                    uint16_t us = rx_data.us_values[sysdata.ouput_mapping[i]];

                    // Clamp to 1000 µs - 2000 µs
                    us = clampui(us, 1000, 2000);

                    if (sysdata.invert_channel[i])
                    {
                        us = 3000 - us;
                    }
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(us));
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
                }
                radio_init = true;
            }
            else if (radio_init)
            {
                // --- WORST CASE SCENARIO : FAILSAFE MODE IF GYRO AND RADIO ARE NOT WORKING ---
                for (int i = 0; i < NUM_PWM_OUPUTS; i++)
                {
                    uint16_t us = sysdata.failsafe_us[i];
                    if (sysdata.invert_channel[i])
                    {
                        us = 3000 - us;
                    }
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(us));
                    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
                }
            }
        }
#ifdef DEBUG_GYRO
        if (esp_timer_get_time() - output_timer > 500000)
        {
            output_timer = esp_timer_get_time();
            printf("Gyro values : %f %f %f\n", gyro_data.rot_x, gyro_data.rot_y, gyro_data.rot_z);
            printf("Accel values : %f %f %f\n", gyro_data.ax, gyro_data.ay, gyro_data.az);
            printf("Attitude : roll %f pitch %f dt %f\n", sysdata.g_attitude.roll_deg, sysdata.g_attitude.pitch_deg, dt);
            fflush(stdout);
        }
#endif
    }
}

void app_main(void)
{
    // Check EEPROM initialization
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        printf("Erasing NVS\n");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_pid_factory();
    init_pwm_factory();

    load_pid_config();
    load_pwm_config();
    load_attitude_config();

    // Init tasks
    gps_init();
    crsf_init();
    gyro_init();
    servo_pwm_init();

    xTaskCreate(actions_task,         "action",      8192, NULL, 5,  &actions_task_handle);
    xTaskCreate(crsf_task_rx,         "crsf_rx",     2048, NULL, 15, &crsf_rx_task_handle);
    xTaskCreate(servo_update_task,    "servo_ctrl",  4096, NULL, 20, &servo_task_handle);
    xTaskCreate(gyro_supervisor_task, "gyro_sv",     2048, NULL, 21, &gyro_sv_task_handle);
    xTaskCreate(telemetry_task,       "telemetry",   4096, NULL, 10, &telemetry_task_handle);

#ifdef DEBUG_STACK
    while (1)
    {
        if (servo_task_handle != NULL)
        {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(servo_task_handle);

            printf("Remaining stack for Servo Task: %u words\n", (unsigned int)remaining_stack);
        }

        if (crsf_task_handle != NULL)
        {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(crsf_task_handle);

            printf("Remaining stack for Crsf Task: %u words\n", (unsigned int)remaining_stack);
        }

        if (actions_task_handle != NULL)
        {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(actions_task_handle);

            printf("Remaining stack for Slow Button Task: %u words\n", (unsigned int)remaining_stack);
        }

        if (gyro_sv_task_handle != NULL)
        {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(gyro_supervisor_task);

            printf("Remaining stack for Gyro Button Task: %u words\n", (unsigned int)remaining_stack);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif
}