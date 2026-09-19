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

// #define DEBUG_GYRO 1
// #define DEBUG_STACK 1
// #define LEVEL_MODE_MAHONY 1

/*
 * MPU 6500 Axis reminder
 * Accel +X -> front
 * Accel +Y -> left
 * Accel +Z -> up
 * Gyro +X -> roll right
 * Gyro +Y -> pitch down
 * Gyro +Z -> yaw left
 */


PID_Config_t g_pid_roll;
PID_Config_t g_pid_pitch;
PID_Config_t g_pid_yaw;

const int   g_servo_gpios[NUM_PWM_OUPUTS] = {PWM_OUTPUT_PINS};
float       g_attitude_correction_rp[2] = {0.f, 0.f};
uint16_t    g_motor_magnets_count = 14;
uint8_t     g_crash_reasons[4] = {0, 0, 0, 0};
attitude_t  g_attitude;
int         g_master_kp_gain_channel = 5;
int         g_master_kd_gain_channel = 7;
int         g_flightmode_channel = 6;
int         g_flightmode = 0;
int         g_ouput_mapping[NUM_PWM_OUPUTS];
uint32_t    g_failsafe_us[NUM_PWM_OUPUTS];
bool        g_invert_channel[NUM_PWM_OUPUTS];
bool        g_invert_accel[3];
bool        g_elrs_armed = false;
bool        g_elrs_data_valid = false;
float       g_crash_g_threshold = 16.f;
motor_safety_state_t g_current_state = MOTOR_STATE_NORMAL;
volatile uint32_t g_esc_temperature = 0;


TaskHandle_t servo_task_handle = NULL;
TaskHandle_t crsf_rx_task_handle = NULL;
TaskHandle_t crsf_tx_task_handle = NULL;
TaskHandle_t actions_task_handle = NULL;
TaskHandle_t power_task_handle = NULL;
TaskHandle_t telemetry_task_handle = NULL;
TaskHandle_t esc_task_handle = NULL;
TaskHandle_t gyro_sv_task_handle = NULL;


PID_Config_t *get_pid_roll(void)
{
     return &g_pid_roll;
}

PID_Config_t *get_pid_pitch(void)
{
     return &g_pid_pitch;
}

PID_Config_t *get_pid_yaw(void)
{
    return &g_pid_yaw;
}

static inline uint32_t __attribute__((always_inline)) us_to_ledc_duty(uint32_t us)
{
    // return (us * 16384) / 20000;
    return (us * ((1 << LEDC_TIMER_14_BIT) - 1)) / LEDC_PERIOD_US;
}

void calibrate_roll(void)
{
    g_attitude_correction_rp[0] = -g_attitude.rollDeg;
    nvs_save_struct("attitude_corr", &g_attitude_correction_rp, sizeof(g_attitude_correction_rp));
}

void calibrate_pitch(void)
{
    g_attitude_correction_rp[1] = -g_attitude.pitchDeg;
    nvs_save_struct("attitude_corr", &g_attitude_correction_rp, sizeof(g_attitude_correction_rp));
}

static inline bool is_elrs_armed()
{
    return g_elrs_armed;
}

static void init_pid(PID_Config_t *pid, float Kp, float Ki, float Kd, float maxRateDegs, bool invert)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->maxRateDegs = maxRateDegs;
    pid->invert      = invert;
    pid->integralAcc = 0.0f;
    pid->prevMeasuredRate = 0.0f;
}

void save_pid_config()
{
    if (nvs_save_struct("pid_roll", &g_pid_roll, sizeof(g_pid_roll)) != ESP_OK)
    {
        printf("Error saving pid_roll\n");
    }
    if (nvs_save_struct("pid_pitch", &g_pid_pitch, sizeof(g_pid_pitch)) != ESP_OK)
    {
        printf("Error saving pid_pitch\n");
    }
    if (nvs_save_struct("pid_yaw", &g_pid_yaw, sizeof(g_pid_yaw)) != ESP_OK)
    {
        printf("Error saving pid_yaw\n");
    }
    if (nvs_save_struct("mgain_channel", &g_master_kp_gain_channel, sizeof(g_master_kp_gain_channel)) != ESP_OK)
    {
        printf("Error saving master_gain_channel\n");
    }
    if (nvs_save_struct("fmode_channel", &g_flightmode_channel, sizeof(g_flightmode_channel)) != ESP_OK)
    {
        printf("Error saving flightmode_channel\n");
    }
    if (nvs_save_struct("accel_invert", &g_invert_accel, sizeof(g_invert_accel)) != ESP_OK)
    {
        printf("Error saving accel_invert\n");
    }
    if (nvs_save_struct("crashthreshold", &g_crash_g_threshold, sizeof(g_crash_g_threshold)) != ESP_OK){
        printf("Error saving crash_g_threshold\n");
    } 
    printf("Saved config\n");
}

void load_pid_config()
{
    if (nvs_load_struct("pid_roll", &g_pid_roll, sizeof(g_pid_roll)) != ESP_OK)
    {
        printf("Error loading pid_roll\n");
    }
    if (nvs_load_struct("pid_pitch", &g_pid_pitch, sizeof(g_pid_pitch)) != ESP_OK)
    {
        printf("Error loading pid_pitch\n");
    }
    if (nvs_load_struct("pid_yaw", &g_pid_yaw, sizeof(g_pid_yaw)) != ESP_OK)
    {
        printf("Error loading pid_yaw\n");
    }
    if (nvs_load_struct("mgain_channel", &g_master_kp_gain_channel, sizeof(g_master_kp_gain_channel)) != ESP_OK)
    {
        printf("Error loading master_gain_channel\n");
    }
    if (nvs_load_struct("fmode_channel", &g_flightmode_channel, sizeof(g_flightmode_channel)) != ESP_OK)
    {
        printf("Error loading flightmode_channel\n");
    }
    if (nvs_load_struct("accel_invert", &g_invert_accel, sizeof(g_invert_accel)) != ESP_OK)
    {
        printf("Error loading accel_invert\n");
    }
    if (nvs_load_struct("crashthreshold", &g_crash_g_threshold, sizeof(g_crash_g_threshold)) != ESP_OK)
    {
        printf("Error loading crash_g_threshold\n");
    }
}

void save_pwm_config()
{
    nvs_save_struct("pwm_mapping",  g_ouput_mapping,  sizeof(g_ouput_mapping));
    nvs_save_struct("pwm_invert",   g_invert_channel, sizeof(g_invert_channel));
    nvs_save_struct("pwm_failsafe", g_failsafe_us,    sizeof(g_failsafe_us));
}

void load_pwm_config()
{
    nvs_load_struct("pwm_mapping",  g_ouput_mapping,  sizeof(g_ouput_mapping));
    nvs_load_struct("pwm_invert",   g_invert_channel, sizeof(g_invert_channel));
    nvs_load_struct("pwm_failsafe", g_failsafe_us,    sizeof(g_failsafe_us));
}

void load_attitude_correction()
{
    nvs_load_struct("attitude_corr", &g_attitude_correction_rp, sizeof(g_attitude_correction_rp));
}

void init_pid_factory()
{
    g_master_kp_gain_channel = 5;
    g_flightmode_channel = 6;
    init_pid(&g_pid_roll,  0.5f, 0.0f, 0.0001f, 250.f, false);
    init_pid(&g_pid_pitch, 0.6f, 0.0f, 0.0001f, 150.f, false);
    init_pid(&g_pid_yaw,   0.7f, 0.0f, 0.0002f, 120.f, false);
    g_invert_accel[0] = false;
    g_invert_accel[1] = false;
    g_invert_accel[2] = false;
}

void init_pwm_factory()
{
    memcpy(g_ouput_mapping,  (int[]){0, 1, 2, 3, 4, 5}, sizeof(g_ouput_mapping));
    memcpy(g_invert_channel, (bool[]){0, 0, 0, 0, 0, 0}, sizeof(g_invert_channel));
    memcpy(g_failsafe_us,    (int[]){1500, 1500, 1000, 1500, 1500, 1500}, sizeof(g_failsafe_us));
}

void reset_crash(void)
{
    for (int i = 0; i < 4; i++)
    {
        g_crash_reasons[i] = 0;
    }
    nvs_save_struct("crash", &g_crash_reasons, sizeof(g_crash_reasons));
}

// ==========================================
// Failsafe button task
// ==========================================
void actions_task(void *pvParameters)
{
    nvs_load_struct("crash", &g_crash_reasons, sizeof(g_crash_reasons));

    uint8_t reboot_reason = (uint8_t)esp_reset_reason();

    if (reboot_reason > (uint8_t)ESP_RST_POWERON)
    {
        g_crash_reasons[3] = g_crash_reasons[2];
        g_crash_reasons[2] = g_crash_reasons[1];
        g_crash_reasons[1] = g_crash_reasons[0];
        g_crash_reasons[0] = reboot_reason;
        nvs_save_struct("crash", &g_crash_reasons, sizeof(g_crash_reasons));
    }

    vTaskDelay(3000);

    for (int i = 0; i < 4; i++)
    {
        printf("Reset reason (recent to old) %d: %s\n", i, reset_reason_to_str(g_crash_reasons[i]));
    }

    while (1)
    {
        if (g_elrs_data_valid && is_elrs_armed() && server_is_running())
        {
            // Deactivate server when ELRS is armed
            stop_webserver();
            printf("Server stopped\n");
            vTaskDelay(3000);
        }
        else  if (g_elrs_data_valid && !is_elrs_armed() && !server_is_running())
        {
            printf("Server started\n");
            start_webserver();
            vTaskDelay(3000);
        }
        else if (!g_elrs_data_valid && !server_is_running())
        {
            printf("Server started\n");
            start_webserver();
            vTaskDelay(3000);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static inline uint16_t compute_axis_pwm(PID_Config_t *pid, int16_t stick_us, float gyro_value, float gyro_value_low, float master_kp_gain, float master_kd_gain, float dt)
{
    float targetRate        = mapStickToRate(stick_us, pid->maxRateDegs, 0);
    float stickInput        = nomalise_stick(stick_us);
    float axis_correction   = compute_axis_pid(stickInput, targetRate, gyro_value, gyro_value_low, dt, master_kp_gain, master_kd_gain, pid, 1);
    
    return map_to_pwm(axis_correction);
}

#ifndef LEVEL_MODE_MAHONY
static void init_attitude(attitude_t *attitude, float ax, float ay, float az)
{
    float accelNorm = fast_sqrtf(ay * ay + az * az);

    // Initialisation directe basée sur la gravité au sol
    attitude->rollDeg = fast_atan2f(ay, az) * RAD_TO_DEG;
    attitude->pitchDeg = (accelNorm > 0.001f)
                             ? fast_atan2f(-ax, accelNorm) * RAD_TO_DEG
                             : 0.0f;
}
#endif

inline static uint32_t process_motor_safety(uint32_t current_throttle_us, bool shock_detected) {
    uint32_t pwm_us = 0;

    switch (g_current_state) {
        
        case MOTOR_STATE_NORMAL:
            if (shock_detected) {
                pwm_us = 1000;
                g_current_state = MOTOR_STATE_EMERGENCY_CUT;
            } else {
                pwm_us = current_throttle_us;
            }
            break;

        case MOTOR_STATE_EMERGENCY_CUT:
            pwm_us = 1000;
            
            if (current_throttle_us < 1020) {
                g_current_state = MOTOR_STATE_WAIT_THROTTLE_ZERO;
            }
            break;

        case MOTOR_STATE_WAIT_THROTTLE_ZERO:
            pwm_us = 1000;
            
            if (current_throttle_us > 1025) {
                g_current_state = MOTOR_STATE_NORMAL;
            }
            break;
    }
    return pwm_us;
}

inline static uint16_t apply_motor_thermal_protection(uint16_t pwm_value)
{
    if (g_esc_temperature > 80)
    {
        return clampui(pwm_value, 1500, 2000);
    } else if (g_esc_temperature > 90)
    {
        return clampui(pwm_value, 1300, 2000);
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
            .gpio_num       = g_servo_gpios[i],
            .duty           = us_to_ledc_duty(1500), // Neutral at boot
            .hpoint         = 0
        };
        ledc_channel_config(&ledc_channel);
        gpio_set_drive_capability(g_servo_gpios[i], GPIO_DRIVE_CAP_2);
    }
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
    init_attitude(&g_attitude, gyro_data.raw_ax, gyro_data.raw_ay, gyro_data.raw_az);
#endif

    uint32_t ul_notified_value;

    while (1)
    {
        int64_t now = esp_timer_get_time();
        bool gyro_failsafe = false;

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

        g_elrs_data_valid = rx_data.valid;

        if (rx_data.valid)
        {
            if (g_master_kp_gain_channel >= 0 && g_master_kp_gain_channel < 8)
            {
                // Update master kp gain
                master_kp_gain = ((float)rx_data.us_values[g_master_kp_gain_channel] - 1000.0f) * 1e-3f;
            }
            if (g_master_kd_gain_channel >= 0 && g_master_kd_gain_channel < 8)
            {
                // Update master kd gain
                master_kd_gain = ((float)rx_data.us_values[g_master_kd_gain_channel] - 1000.0f) * 1e-3f;
            }
            if (g_flightmode_channel >= 0 && g_flightmode_channel < 8)
            {
                int fmv = rx_data.us_values[g_flightmode_channel];
                if (fmv > 1700)
                {
                    g_flightmode = FLIGHTMODE_LEVEL;
                }
                else if (fmv > 1300)
                {
                    g_flightmode = FLIGHTMODE_STAB;
                }
                else
                {
                    g_flightmode = FLIGHTMODE_FREE;
                }
            }
            g_elrs_armed = rx_data.us_values[4] > 1600;
        }

        if (gyro_data.valid)
        {
#ifdef LEVEL_MODE_MAHONY
            // ~70us execution time
            mahony_update(gyro_data.rot_x * DEG_TO_RAD, gyro_data.rot_y * DEG_TO_RAD, gyro_data.rot_z * DEG_TO_RAD, gyro_data.ax, gyro_data.ay, gyro_data.az, dt);
#else
            compute_attitude(&g_attitude, gyro_data.ax, gyro_data.ay, gyro_data.az, gyro_data.rot_x, gyro_data.rot_y, dt);
#endif
        }

        // Gyro Pitch : -up +down
        // Gyro Roll : -left +right
        // Gyro Yaw : -right +left

        if (gyro_data.valid && rx_data.valid)
        {
            if (g_flightmode == FLIGHTMODE_LEVEL)
            {
#ifdef LEVEL_MODE_MAHONY
                mahony_get_euler(&g_attitude);
#endif
                const float attitude_pitch = g_attitude.pitchDeg + g_attitude_correction_rp[1];
                const float attitude_roll  = g_attitude.rollDeg + g_attitude_correction_rp[0];

                float stickInputRoll  = nomalise_stick(rx_data.us_values[CHANNEL_AILERON]);
                float targetAngleRoll = stickInputRoll * 45.0f; // -45° à +45°
                float targetRateRoll  = 4.f * (targetAngleRoll - attitude_roll);
                targetRateRoll        = clampf(targetRateRoll, -g_pid_roll.maxRateDegs, g_pid_roll.maxRateDegs);

                float stickInputPitch  = nomalise_stick(rx_data.us_values[CHANNEL_ELEVATOR]);
                float targetAnglePitch = stickInputPitch * 35.0f; // -35° à +35°
                float targetRatePitch  = 3.5f * (targetAnglePitch - attitude_pitch);
                targetRatePitch        = clampf(targetRatePitch, -g_pid_pitch.maxRateDegs, g_pid_pitch.maxRateDegs);

                rx_data.us_values[CHANNEL_AILERON]  = map_to_pwm(compute_axis_pid(0, targetRateRoll, gyro_data.rot_x,  gyro_data.rot_x_low, dt, master_kp_gain, master_kd_gain, &g_pid_roll, 0));
                rx_data.us_values[CHANNEL_ELEVATOR] = map_to_pwm(compute_axis_pid(0, targetRatePitch, gyro_data.rot_y, gyro_data.rot_y_low, dt, master_kp_gain, master_kd_gain, &g_pid_pitch, 0));
                rx_data.us_values[CHANNEL_RUDDER]   = compute_axis_pwm(&g_pid_yaw, rx_data.us_values[CHANNEL_RUDDER], -gyro_data.rot_z, -gyro_data.rot_z_low, master_kp_gain, master_kd_gain, dt);
            }
            else if (g_flightmode == FLIGHTMODE_STAB)
            {
                rx_data.us_values[CHANNEL_AILERON]  = compute_axis_pwm(&g_pid_roll,  rx_data.us_values[CHANNEL_AILERON],  gyro_data.rot_x, gyro_data.rot_x_low, master_kp_gain, master_kd_gain, dt);
                rx_data.us_values[CHANNEL_ELEVATOR] = compute_axis_pwm(&g_pid_pitch, rx_data.us_values[CHANNEL_ELEVATOR], gyro_data.rot_y, gyro_data.rot_y_low, master_kp_gain, master_kd_gain, dt);
                rx_data.us_values[CHANNEL_RUDDER]   = compute_axis_pwm(&g_pid_yaw,   rx_data.us_values[CHANNEL_RUDDER], -gyro_data.rot_z, -gyro_data.rot_z_low, master_kp_gain, master_kd_gain, dt);
            }
        }
        else if (gyro_data.valid && !rx_data.valid)
        {
            // No RX data but gyro OK, attempt to save plane
#ifdef LEVEL_MODE_MAHONY
            mahony_get_euler(&g_attitude);
#endif
            const float attitude_pitch = g_attitude.pitchDeg + g_attitude_correction_rp[1];
            const float attitude_roll  = g_attitude.rollDeg + g_attitude_correction_rp[0];
            // Failsafe mode: no RX data, but gyro is valid. Try to keep plane flat and turning
            float targetAngleRoll = 20.f;
            float targetRateRoll  = 4.0f * (targetAngleRoll - attitude_roll);
            targetRateRoll        = clampf(targetRateRoll, -g_pid_roll.maxRateDegs, g_pid_roll.maxRateDegs);
            
            float targetAnglePitch = -10.f;
            float targetRatePitch  = 3.5f * (targetAnglePitch - attitude_pitch);
            targetRatePitch        = clampf(targetRatePitch, -g_pid_pitch.maxRateDegs, g_pid_pitch.maxRateDegs);

            rx_data.us_values[CHANNEL_AILERON]  = map_to_pwm(compute_axis_pid(0.f, targetRateRoll,  gyro_data.rot_x, gyro_data.rot_x_low, dt, master_kp_gain, master_kd_gain, &g_pid_roll, 0));
            rx_data.us_values[CHANNEL_ELEVATOR] = map_to_pwm(compute_axis_pid(0.f, targetRatePitch, gyro_data.rot_y, gyro_data.rot_y_low, dt, master_kp_gain, master_kd_gain, &g_pid_pitch, 0));
            rx_data.us_values[CHANNEL_THROTTLE] = 1000; // Motor off
            rx_data.us_values[CHANNEL_RUDDER]   = 1500; // Yaw neutral
            gyro_failsafe = true;
        }

        // Shock detection, if something is hit (propeller maybe?), cut off the motor to avoid further damage
        const float az            = gyro_data.raw_az * 0.7f; // decrease Z axis sensitivity to avoid false positives on landing
        const float shock_factor  = gyro_data.raw_ax * gyro_data.raw_ax + gyro_data.raw_ay * gyro_data.raw_ay + az * az;
        const bool shock_detected = shock_factor > g_crash_g_threshold;
        rx_data.us_values[CHANNEL_THROTTLE] = apply_motor_thermal_protection(rx_data.us_values[CHANNEL_THROTTLE]);
        
        // Check ESC temperature and apply thermal protection if necessary
        rx_data.us_values[CHANNEL_THROTTLE] = process_motor_safety(rx_data.us_values[CHANNEL_THROTTLE], shock_detected);

        if (!g_elrs_armed)
        {
            // If not armed, force throttle to 0%
            rx_data.us_values[CHANNEL_THROTTLE] = 1000;
        }

        if ((esp_timer_get_time() - servo_timer) > 20000)
        {
            // 50 Hz refresh (20 ms)
            servo_timer = esp_timer_get_time();
            if (rx_data.valid || gyro_failsafe)
            {
                for (int i = 0; i < NUM_PWM_OUPUTS; i++)
                {
                    uint16_t us = rx_data.us_values[g_ouput_mapping[i]];

                    // Clamp to 1000 µs - 2000 µs
                    us = clampui(us, 1000, 2000);

                    if (g_invert_channel[i])
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
                    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(g_failsafe_us[i]));
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
            printf("Attitude : roll %f pitch %f dt %f\n", g_attitude.rollDeg, g_attitude.pitchDeg, dt);
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
    load_attitude_correction();

    // Init tasks
    gps_init();
    crsf_init();
    gyro_init();
    servo_pwm_init();

    xTaskCreate(actions_task,         "action_task", 8192, NULL, 5,  &actions_task_handle);
    xTaskCreate(crsf_task_rx,         "crsf_rx",     2048, NULL, 15, &crsf_rx_task_handle);
    xTaskCreate(servo_update_task,    "servo_ctrl",  4096, NULL, 20, &servo_task_handle);
    xTaskCreate(gyro_supervisor_task, "gyro_sv",     2048, NULL, 21, &gyro_sv_task_handle);
    xTaskCreate(telemetry_task,             "tlm_task",    4096, NULL, 10, &telemetry_task_handle);

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