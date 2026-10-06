#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <utils.h>
#include "pid.h"
#include "config.h"

#include "esp_attr.h"
typedef struct
{
    float q0, q1, q2, q3;   // Attitude quaternion components
    float ix, iy, iz;       // integral error accumulators
} MahonyFilter;

static MahonyFilter mahony = {.q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f, .ix = 0.0f, .iy = 0.0f, .iz = 0.0f};

/**
 * @brief Compute the PID correction for a given axis based on stick input, target rate, measured rate, and PID configuration.
 * @param stick_input     Direct order from the stick input, normalized to [-1.0, 1.0]
 * @param target_rate     Target rotation rate in deg/s
 * @param measured_rate   Measured rotation rate by the gyro in deg/s
 * @param measured_rate_low Measured rotation rate by the low-pass filtered gyro in deg/s
 * @param dt             Delta time between two loops in seconds
 * @param master_kp_gain GGlobal radio gain [0.0 to 1.0]
 * @param pid            Pointer to the PID structure for the axis
 */
IRAM_ATTR float compute_axis_pid(float stick_input, float target_rate, float measured_rate, float measured_rate_low, float dt, float master_kp_gain, float master_kd_gain, PID_Config_t *pid, char use_stick_factor)
{
    float i_term = 0.0f;

    // a. Angular rate error normalized to [-1.0, 1.0] range
    float error_nomalized = (target_rate - measured_rate) / pid->max_rate_degs;

    // b. Stick Derating : reduce correction when stick is near the end of travel to avoid overshoot
    float stick_factor = 1.0f;
    if (use_stick_factor)
    {
        stick_factor = 1.0f - fast_fabsf(stick_input);
        if (stick_factor < 0.0f)
            stick_factor = 0.0f;
    }
    // c. Proportionnal term (P)
    float p_term = pid->Kp * error_nomalized * master_kp_gain;

    if (pid->Ki > 0.0f)
    {
        // d. Integral term (I) with anti-windup and reset when stick is moved
        pid->integral_acc += error_nomalized * dt;
        pid->integral_acc = clampf(pid->integral_acc, -MAX_I_TERM, MAX_I_TERM);

        // I term cancellation when stick is moved significantly to avoid integral windup
        if (fast_fabsf(stick_input) > 0.05f)
        {
            pid->integral_acc = 0.0f;
        }
        i_term = pid->Ki * pid->integral_acc;
    }

    // e. Derivate Term (D)
    float d_term = 0.0f;
    if (dt > 0.0f)
    {
        float raw_derivative = -(measured_rate_low - pid->prev_measured_rate) / dt;
        float d_normalized = raw_derivative / pid->max_rate_degs;
        d_term = pid->Kd * d_normalized * master_kd_gain;
    }
    pid->prev_measured_rate = measured_rate_low;

    // f. Final gyro correction with master gain and stick factor
    float gyro_correction = (p_term + i_term + d_term) * stick_factor;

    if (pid->invert)
    {
        gyro_correction = -gyro_correction;
    }

    // Invert correction if needed
    float output = stick_input + gyro_correction;

    // h. Clamp output to [-1.0, 1.0] range
    clampf(output, -1.0f, 1.0f);

    return output;
}

#define ALPHA 0.98f // 98% Gyro, 2% Accel

/*
 * @brief Compute the attitude using a complementary filter combining gyro and accelerometer data.
 * @param attitude Pointer to the attitude structure to update.
 * @param ax, ay, az Accelerometer readings in g.
 * @param gyro_roll_deg_s, gyro_pitch_deg_s Gyro readings in deg/s.
 * @param dt Time step in seconds.
 */
IRAM_ATTR void compute_attitude(attitude_t *attitude, float ax, float ay, float az, float gyro_roll_deg_s, float gyro_pitch_deg_s, float dt)
{
    if (dt <= 0.00001f)
        return;

    // 1. Total accel norm squared (to check if it's within 1g ± 0.2g)
    float total_accel_norm = ax * ax + ay * ay + az * az;
    
    // 2. Gyro integration to estimate roll and pitch angles
    float gyro_roll  = attitude->roll_deg + gyro_roll_deg_s * dt;
    float gyro_pitch = attitude->pitch_deg + gyro_pitch_deg_s * dt;

    // 3. Check if accelerometer readings are within 1g ± 0.2g (0.8^2 = 0.64, 1.2^2 = 1.44)
    if (total_accel_norm >= 0.64f && total_accel_norm <= 1.44f) 
    {
        float accel_norm_YZ = fast_sqrtf(ay * ay + az * az);
        float accel_roll    = rad_to_deg(fast_atan2f(ay, az));
        float accel_pitch   = (accel_norm_YZ > 0.001f) ? rad_to_deg(fast_atan2f(-ax, accel_norm_YZ)) : attitude->pitch_deg;

        // Adaptative fusion of gyro and accelerometer data using complementary filter
        attitude->roll_deg  = ALPHA * gyro_roll + (1.0f - ALPHA) * accel_roll;
        attitude->pitch_deg = ALPHA * gyro_pitch + (1.0f - ALPHA) * accel_pitch;
    } 
    else 
    {
        // Outside of valid accel range, rely solely on gyro integration
        attitude->roll_deg  = gyro_roll;
        attitude->pitch_deg = gyro_pitch;
    }
}

/*
 * @brief  Compute the attitude using a Mahony filter with new sensor readings.
 * @param gx, gy, gz Gyro readings in rad/s.
 * @param ax, ay, az Accelerometer readings in g.
 * @param dt Time step in seconds.
 */
IRAM_ATTR void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float q0 = mahony.q0, q1 = mahony.q1, q2 = mahony.q2, q3 = mahony.q3;
    float norm;

    // Convert degrees to radians
    gx *=  DEG_TO_RAD;
    gy *=  DEG_TO_RAD;
    gz *=  DEG_TO_RAD;

    // Accel normalization
    norm = ax * ax + ay * ay + az * az;

    // Weighting the KP term to avoid instability (accelerometer confidence)
    const float deviation = fast_fabsf(norm - 1.0f);
    float weight = 1.0f - (deviation * 2.0f);
    if (weight < 0.0f) weight = 0.0f;
    const float kp_effective = MAHONY_KP * weight;

    if (norm > 0.0001f)
    {
        norm = fast_inv_sqrtf(norm);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Gravity estimation (Z axis)
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // Cross product of error (a x v)
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        // Integral correction (simple anti-windup)
        if (MAHONY_KI > 0.0f && deviation < 0.15f)
        {
            mahony.ix += ex * MAHONY_KI * dt;
            mahony.iy += ey * MAHONY_KI * dt;
            mahony.iz += ez * MAHONY_KI * dt;

            gx += mahony.ix;
            gy += mahony.iy;
            gz += mahony.iz;
        }
        else
        {
            mahony.ix = 0.0f;
            mahony.iy = 0.0f;
            mahony.iz = 0.0f;
        }

        // Proportionnal correction
        gx += kp_effective * ex;
        gy += kp_effective * ey;
        gz += kp_effective * ez;
    }

    // Quaternion integration (Euler formula)
    float ha = 0.5f * gx * dt;
    float hb = 0.5f * gy * dt;
    float hc = 0.5f * gz * dt;

    float qa = q0, qb = q1, qc = q2, qd = q3;
    
    q0 += (-qb * ha - qc * hb - qd * hc);
    q1 += ( qa * ha + qc * hc - qd * hb);
    q2 += ( qa * hb - qb * hc + qd * ha);
    q3 += ( qa * hc + qb * hb - qc * ha);

    // Quaternion normalization
    norm = fast_inv_sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    mahony.q0 = q0 * norm;
    mahony.q1 = q1 * norm;
    mahony.q2 = q2 * norm;
    mahony.q3 = q3 * norm;
}

// Euler angle extractor (-180° à +180° pour Roll, -90° à +90° pour Pitch)
IRAM_ATTR void mahony_get_euler(attitude_t *attidude)
{
    float q0 = mahony.q0, q1 = mahony.q1, q2 = mahony.q2, q3 = mahony.q3;

    // Roll : -180° à +180°
    attidude->roll_deg = rad_to_deg(fast_atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)));

    float sinp = 2.0f * (q0 * q2 - q1 * q3); 
    
    if (fabsf(sinp) >= 1.0f)
    {
        attidude->pitch_deg = copysignf(90.0f, sinp);
    }
    else
    {
        attidude->pitch_deg = rad_to_deg(fast_asinf(sinp));
    }
    // Yaw : -180° to +180° (can drift without magnetometer, OK in relative)
    // attidude->yawDeg = rad_to_deg(fast_atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)));
}

void init_attitude(attitude_t *attitude, float ax, float ay, float az)
{
    float accelNorm = fast_sqrtf(ay * ay + az * az);

    // Init with gravity vector
    attitude->roll_deg  = rad_to_deg(fast_atan2f(ay, az));
    attitude->pitch_deg = (accelNorm > 0.001f)
                         ? rad_to_deg(fast_atan2f(-ax, accelNorm))
                         : 0.0f;
    attitude->yaw_deg  = 0;
}