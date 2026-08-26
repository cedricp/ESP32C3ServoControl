#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

// #define DEBUG_STACK 1
#define DEBUG_GYRO 1
#define LEVEL_MODE_MAHONY 1

/*
 * MPU 6500 Axis reminder
 * Accel +X -> front
 * Accel +Y -> left
 * Accel +Z -> up
 * Gyro +X -> roll right
 * Gyro +Y -> pitch down
 * Gyro +Z -> yaw left
 */

#define ONBOARD_LED_PIN 8
#define PAIRING_BUTTON_PIN 9
#define LEDC_FREQUENCY_HZ 50
#define LEDC_PERIOD_US (1000000 / LEDC_FREQUENCY_HZ) // 20000 us

enum
{
    FLIGHTMODE_FREE = 0,
    FLIGHTMODE_STAB = 1,
    FLIGHTMODE_LEVEL = 2
} flightmode_t;

enum
{
    CHANNEL_AILERON,
    CHANNEL_ELEVATOR,
    CHANNEL_THROTTLE,
    CHANNEL_RUDDER,
    CHANNEL_ARM
} channels_t;

const int servo_gpios[NUM_PWM_OUPUTS] = {5, 6, 7, 10, 2, 0};
uint8_t g_crash_reasons[4] = {0, 0, 0, 0};
float attitude_correction_rp[2] = {0.f, 0.f};

PID_Config_t pidRoll;
PID_Config_t pidPitch;
PID_Config_t pidYaw;

PID_Config_t *get_pid_roll(void) { return &pidRoll; }
PID_Config_t *get_pid_pitch(void) { return &pidPitch; }
PID_Config_t *get_pid_yaw(void) { return &pidYaw; }

attitude_t g_attitude;

int g_master_gain_channel = 5;
int g_flightmode_channel = 6;
int g_flightmode = 0;
int g_ouput_mapping[NUM_PWM_OUPUTS];
uint32_t g_failsafe_us[NUM_PWM_OUPUTS];
bool g_invert_channel[NUM_PWM_OUPUTS];
bool g_invert_accel[3];
bool g_elrs_armed = false;
bool g_elrs_data_valid = false;

TaskHandle_t servo_task_handle = NULL;
TaskHandle_t crsf_task_handle = NULL;
TaskHandle_t actions_task_handle = NULL;
TaskHandle_t gyro_task_handle = NULL;

QueueHandle_t gyro_queue = NULL;

static inline uint32_t __attribute__((always_inline)) us_to_ledc_duty(uint32_t us)
{
    // return (us * 16384) / 20000;
    return (us * ((1 << LEDC_TIMER_14_BIT) - 1)) / LEDC_PERIOD_US;
}

void calibrate_roll(void)
{
    attitude_correction_rp[0] = -g_attitude.rollDeg;
    nvs_save_struct("attitude_corr", &attitude_correction_rp, sizeof(attitude_correction_rp));
}

void calibrate_pitch(void)
{
    attitude_correction_rp[1] = -g_attitude.pitchDeg;
    nvs_save_struct("attitude_corr", &attitude_correction_rp, sizeof(attitude_correction_rp));
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
    pid->invert = invert;

    pid->integralAcc = 0.0f;
    pid->prevMeasuredRate = 0.0f;
}

void save_pid_config()
{
    if (nvs_save_struct("pid_roll", &pidRoll, sizeof(pidRoll)) != ESP_OK)
    {
        printf("Error saving pid_roll\n");
    }
    if (nvs_save_struct("pid_pitch", &pidPitch, sizeof(pidPitch)) != ESP_OK)
    {
        printf("Error saving pid_pitch\n");
    }
    if (nvs_save_struct("pid_yaw", &pidYaw, sizeof(pidYaw)) != ESP_OK)
    {
        printf("Error saving pid_yaw\n");
    }
    if (nvs_save_struct("mgain_channel", &g_master_gain_channel, sizeof(g_master_gain_channel)) != ESP_OK)
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
    printf("Saved config\n");
}

void load_pid_config()
{
    if (nvs_load_struct("pid_roll", &pidRoll, sizeof(pidRoll)) != ESP_OK)
    {
        printf("Error loading pid_roll\n");
    }
    if (nvs_load_struct("pid_pitch", &pidPitch, sizeof(pidPitch)) != ESP_OK)
    {
        printf("Error loading pid_pitch\n");
    }
    if (nvs_load_struct("pid_yaw", &pidYaw, sizeof(pidYaw)) != ESP_OK)
    {
        printf("Error loading pid_yaw\n");
    }
    if (nvs_load_struct("mgain_channel", &g_master_gain_channel, sizeof(g_master_gain_channel)) != ESP_OK)
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
}

void save_pwm_config()
{
    nvs_save_struct("pwm_mapping", g_ouput_mapping, sizeof(g_ouput_mapping));
    nvs_save_struct("pwm_invert", g_invert_channel, sizeof(g_invert_channel));
    nvs_save_struct("pwm_failsafe", g_failsafe_us, sizeof(g_failsafe_us));
}

void load_pwm_config()
{
    nvs_load_struct("pwm_mapping", g_ouput_mapping, sizeof(g_ouput_mapping));
    nvs_load_struct("pwm_invert", g_invert_channel, sizeof(g_invert_channel));
    nvs_load_struct("pwm_failsafe", g_failsafe_us, sizeof(g_failsafe_us));
}

void load_attitude_correction()
{
    nvs_load_struct("attitude_corr", &attitude_correction_rp, sizeof(attitude_correction_rp));
}

void init_pid_factory()
{
    g_master_gain_channel = 5;
    g_flightmode_channel = 6;
    init_pid(&pidRoll, 0.5f, 0.0f, 0.0001f, 250.f, false);
    init_pid(&pidPitch, 0.6f, 0.0f, 0.0001f, 150.f, false);
    init_pid(&pidYaw, 0.7f, 0.0f, 0.0002f, 120.f, false);
    g_invert_accel[0] = false;
    g_invert_accel[1] = false;
    g_invert_accel[2] = false;
}

void erase_nvs(void)
{
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
}

void init_pwm_factory()
{
    memcpy(g_ouput_mapping, (int[]){0, 1, 2, 3, 4, 5}, sizeof(g_ouput_mapping));
    memcpy(g_invert_channel, (bool[]){0, 0, 0, 0, 0, 0}, sizeof(g_invert_channel));
    memcpy(g_failsafe_us, (int[]){1500, 1500, 1000, 1500, 1500, 1500}, sizeof(g_failsafe_us));
}

static void blink_led(int times, int delay_ms, bool finish_lit)
{
    for (int i = 0; i < times; i++)
    {
        gpio_set_level(ONBOARD_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(ONBOARD_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    if (finish_lit)
    {
        gpio_set_level(ONBOARD_LED_PIN, 0);
    }
    else
    {
        gpio_set_level(ONBOARD_LED_PIN, 1);
    }
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
    // 1. Configure the input pullup
    //     gpio_config_t io_conf = {
    //         .pin_bit_mask = (1ULL << PAIRING_BUTTON_PIN),
    //         .mode         = GPIO_MODE_INPUT,
    //         .pull_up_en   = GPIO_PULLUP_ENABLE,
    //         .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //         .intr_type    = GPIO_INTR_DISABLE // No interrupts needed!
    //     };
    //     gpio_config(&io_conf);

    gpio_config_t io_conf2 = {
        .pin_bit_mask = (1ULL << ONBOARD_LED_PIN),
        .mode = GPIO_MODE_OUTPUT, // Set as output
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf2);
    // Start LED off
    gpio_set_level(ONBOARD_LED_PIN, 1);

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
        printf("Reset reason (receny to old) %d: %s\n", i, reset_reason_to_str(g_crash_reasons[i]));
    }

    while (1)
    {
        if (g_elrs_data_valid && is_elrs_armed() && server_is_running())
        {
            // Deactivate server when ELRS is armed
            stop_webserver();
            printf("Server stopped\n");
            blink_led(4, 200, false); // Visual feedback for server off
            vTaskDelay(3000);
        }

        if (g_elrs_data_valid && !is_elrs_armed() && !server_is_running())
        {
            printf("Server started\n");
            start_webserver();
            blink_led(4, 200, true); // Visual feedback for server on
            vTaskDelay(3000);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void test_sequence(const uint32_t *sequence)
{
    for (int i = 0; i < NUM_PWM_OUPUTS; i++)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_ledc_duty(sequence[i]));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

static inline uint16_t compute_axis(PID_Config_t *pid, int16_t stick_us, float gyro_value, float gyro_value_low, float master_gain, float dt)
{
    float targetRate = mapStickToRate(stick_us, pid->maxRateDegs, 0);
    float stickInput = nomalise_stick(stick_us);
    float axis_correction = compute_axis_pid(stickInput, targetRate, gyro_value, gyro_value_low, dt, master_gain, pid, 1);
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

static inline float constrain_angle_deg(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

void init_flight_watchdog(void)
{
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 5000,         // Timeout de 2 secondes
        .idle_core_mask = (1 << 0), // Surveille aussi la tâche Idle du Coeur 0
        .trigger_panic = true       // Forcer la panique/reset en cas de timeout
    };

    // Initialisation ou re-configuration du TWDT
    esp_task_wdt_reconfigure(&twdt_config);

    // Ajouter la tâche courante (ex: la boucle de contrôle) au Watchdog
    esp_task_wdt_add(NULL);
}

// ==========================================
// Servo managemenent task
// ==========================================
void servo_update_task(void *pvParameters)
{
    servo_data_t rx_data;
    gyro_data_t gyro_data;
    bool radio_init = false;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < NUM_PWM_OUPUTS; i++)
    {
        ledc_channel_config_t ledc_channel = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = servo_gpios[i],
            .duty = us_to_ledc_duty(1500), // Neutral at boot
            .hpoint = 0};
        ledc_channel_config(&ledc_channel);
        gpio_set_drive_capability(servo_gpios[i], GPIO_DRIVE_CAP_0);
    }

    static int64_t last_time = 0;
    int64_t servo_timer = esp_timer_get_time();
#ifdef DEBUG_GYRO
    int64_t output_timer = esp_timer_get_time();
#endif
    float master_gain = 0.3f;

#ifndef LEVEL_MODE_MAHONY
    get_gyro_data(&gyro_data);
    init_attitude(&g_attitude, gyro_data.raw_ax, gyro_data.raw_ay, gyro_data.raw_az);
#endif

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

        if (xQueueReceive(gyro_queue, &gyro_data, pdMS_TO_TICKS(5)) != pdTRUE)
        {
            // Timeout or no data available, mark gyro data as invalid
            // 20 ms is enough for 50 Hz update rate, if we don't get data in that time, something is wrong
            gyro_data.valid = false;
            //ESP_LOGW("MPU", "Gyro data timeout");
        }
        get_servo_data(&rx_data);

        g_elrs_data_valid = rx_data.valid;

        if (rx_data.valid)
        {
            if (g_master_gain_channel >= 0 && g_master_gain_channel < 8)
            {
                // Update master gain
                master_gain = ((float)rx_data.us_values[g_master_gain_channel] - 1000.0f) * 1e-3f;
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
            mahony_update(gyro_data.rot_x * DEG_TO_RAD, gyro_data.rot_y * DEG_TO_RAD, gyro_data.rot_z * DEG_TO_RAD, gyro_data.ax, gyro_data.ay, gyro_data.az, dt);
#else
            compute_attitude(&g_attitude, gyro_data.ax, gyro_data.ay, gyro_data.az, gyro_data.rot_x, gyro_data.rot_y, dt);
#endif
        }

        if (gyro_data.valid && rx_data.valid)
        {
            if (g_flightmode == FLIGHTMODE_LEVEL)
            {
#ifdef LEVEL_MODE_MAHONY
                // 70 us execution time
                mahony_get_euler(&g_attitude);
#endif
                float attitude_pitch = g_attitude.pitchDeg + attitude_correction_rp[1];
                float attitude_roll = g_attitude.rollDeg + attitude_correction_rp[0];

                float stickInputRoll = nomalise_stick(rx_data.us_values[CHANNEL_AILERON]);
                float targetAngleRoll = stickInputRoll * 45.0f; // -45° à +45°
                float targetRateRoll = 4.f * (targetAngleRoll - attitude_roll);
                targetRateRoll = clampf(targetRateRoll, -pidRoll.maxRateDegs, pidRoll.maxRateDegs);

                float stickInputPitch = nomalise_stick(rx_data.us_values[CHANNEL_ELEVATOR]);
                float targetAnglePitch = -stickInputPitch * 35.0f; // -35° à +35°
                float targetRatePitch = 3.5f * (targetAnglePitch - attitude_pitch);
                targetRatePitch = clampf(targetRatePitch, -pidPitch.maxRateDegs, pidPitch.maxRateDegs);

                rx_data.us_values[CHANNEL_AILERON]  = map_to_pwm(compute_axis_pid(0, targetRateRoll, gyro_data.rot_x, gyro_data.rot_x_low, dt, master_gain, &pidRoll, 0));
                rx_data.us_values[CHANNEL_ELEVATOR] = map_to_pwm(compute_axis_pid(0, targetRatePitch, gyro_data.rot_y, gyro_data.rot_y_low, dt, master_gain, &pidPitch, 0));
                rx_data.us_values[CHANNEL_RUDDER]   = compute_axis(&pidYaw, rx_data.us_values[CHANNEL_RUDDER], gyro_data.rot_z, gyro_data.rot_z_low, master_gain, dt);
            }
            else if (g_flightmode == FLIGHTMODE_STAB)
            {
                rx_data.us_values[CHANNEL_AILERON]  = compute_axis(&pidRoll, rx_data.us_values[CHANNEL_AILERON], gyro_data.rot_x, gyro_data.rot_x_low, master_gain, dt);
                rx_data.us_values[CHANNEL_ELEVATOR] = compute_axis(&pidPitch, rx_data.us_values[CHANNEL_ELEVATOR], gyro_data.rot_y, gyro_data.rot_y_low, master_gain, dt);
                rx_data.us_values[CHANNEL_RUDDER]   = compute_axis(&pidYaw, rx_data.us_values[CHANNEL_RUDDER], gyro_data.rot_z, gyro_data.rot_z_low, master_gain, dt);
            }
        }
        else if (gyro_data.valid && !rx_data.valid)
        {
#ifdef LEVEL_MODE_MAHONY
            // 33us execution timex
            mahony_get_euler(&g_attitude);
#endif
            float attitude_pitch = g_attitude.pitchDeg + attitude_correction_rp[1];
            float attitude_roll = g_attitude.rollDeg + attitude_correction_rp[0];
            // Failsafe mode: no RX data, but gyro is valid. Try to keep plane flat and turning
            float targetAngleRoll = 20.f; // -45° à +45°
            float targetRateRoll = 4.0f * (targetAngleRoll - attitude_roll);
            if (pidPitch.invert) targetAngleRoll = -targetAngleRoll;
            targetRateRoll = clampf(targetRateRoll, -pidRoll.maxRateDegs, pidRoll.maxRateDegs);
            
            float targetAnglePitch = 10.f;
            float targetRatePitch = 3.5f * (targetAnglePitch - attitude_pitch);
            if (pidPitch.invert) targetRatePitch = -targetRatePitch;
            targetRatePitch = clampf(targetRatePitch, -pidPitch.maxRateDegs, pidPitch.maxRateDegs);

            rx_data.us_values[CHANNEL_AILERON]  = map_to_pwm(compute_axis_pid(0.f, targetRateRoll, gyro_data.rot_x, gyro_data.rot_x_low, dt, master_gain, &pidRoll, 0));
            rx_data.us_values[CHANNEL_ELEVATOR] = map_to_pwm(compute_axis_pid(0.f, targetRatePitch, gyro_data.rot_y, gyro_data.rot_y_low, dt, master_gain, &pidPitch, 0));
            rx_data.us_values[CHANNEL_THROTTLE] = 1000; // Motor off
            rx_data.us_values[CHANNEL_RUDDER]   = 1500; // Yaw neutral
            gyro_failsafe = true;
        }

        int64_t delta = esp_timer_get_time() - servo_timer;
        if (delta > 20000)
        {
            // 50 Hz refresh (20 ms)
            servo_timer = esp_timer_get_time();
            if (rx_data.valid || gyro_failsafe)
            {
                for (int i = 0; i < NUM_PWM_OUPUTS; i++)
                {
                    uint16_t us = rx_data.us_values[g_ouput_mapping[i]];

                    // Clamp to 1000 µs - 2000 µs
                    us = clampu(us, 1000, 2000);

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
            // printf("Servo values : %d %d %d %d %d %d\n", rx_data.us_values[0], rx_data.us_values[1], rx_data.us_values[2], rx_data.us_values[3], rx_data.us_values[4], rx_data.us_values[5]);
            printf("Gyro values : %f %f %f\n", gyro_data.rot_x, gyro_data.rot_y, gyro_data.rot_z);
            printf("Accel values : %f %f %f\n", gyro_data.ax, gyro_data.ay, gyro_data.az);
            printf("Attitude : roll %f pitch %f dt %f [%i %i %i]\n", g_attitude.rollDeg, g_attitude.pitchDeg, dt, g_invert_accel[0], g_invert_accel[1], g_invert_accel[2]);
            fflush(stdout);
        }
#endif
    }
}

void app_main(void)
{
    // Check EEPROM initialization
    vTaskDelay(pdMS_TO_TICKS(3000));

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        printf("Erasing NVS\n");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // GPIO pin 4 temp config, seems to avoid crashes
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // Fixe l'état haut par défaut
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // Désactive toute interruption si inutile
    };
    gpio_config(&io_conf);

    init_pid_factory();
    init_pwm_factory();

    load_pid_config();
    load_pwm_config();
    load_attitude_correction();

    start_webserver();

    gyro_queue = xQueueCreate(1, sizeof(gyro_data_t));

    xTaskCreate(actions_task, "action_task", 8192, NULL, 5, &actions_task_handle);
    xTaskCreate(crsf_rx_task, "crsf_rx", 2048, NULL, 15, &crsf_task_handle);
    xTaskCreate(servo_update_task, "servo_ctrl", 4096, NULL, 20, &servo_task_handle);
    xTaskCreate(gyro_control_task, "gyro", 4096, NULL, 21, &gyro_task_handle);

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

        if (gyro_task_handle != NULL)
        {
            UBaseType_t remaining_stack = uxTaskGetStackHighWaterMark(gyro_task_handle);

            printf("Remaining stack for Gyro Button Task: %u words\n", (unsigned int)remaining_stack);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif
}