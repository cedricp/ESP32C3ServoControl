#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <utils.h>
#include "pid.h"

#include "esp_attr.h"

#define MAX_I_TERM       0.2f
#define MAX_SERVO_OUTPUT 1.0f

#define MAHONY_KP 0.5f 
#define MAHONY_KI 0.0f

float nomalise_stick(uint16_t pulse_us)
{
    return ((float)pulse_us - 1500.0f) / 500.0f;
}

/**
 * @brief Compute the PID correction for a given axis based on stick input, target rate, measured rate, and PID configuration.
 * @param stickInput   Direct order from the stick input, normalized to [-1.0, 1.0]
 * @param targetRate   Target rotation rate in deg/s
 * @param measuredRate Measured rotation rate by the gyro in deg/s
 * @param dt           Delta time between two loops in seconds
 * @param masterGain   GGlobal radio gain [0.0 to 1.0]
 * @param pid          Pointer to the PID structure for the axis
 */
IRAM_ATTR float compute_axis_pid(float stickInput, float targetRate, float measuredRate, float measuredRate_low, float dt, float masterGain, PID_Config_t* pid, char useStickFactor) {
    float iTerm = 0.0f;

    // a. Angular rate error normalized to [-1.0, 1.0] range
    float error_nomalized = (targetRate - measuredRate) / pid->maxRateDegs;

    // b. Stick Derating : reduce correction when stick is near the end of travel to avoid overshoot
    float stickFactor = 1.0f;
    if (useStickFactor){
        stickFactor = 1.0f - fast_fabsf(stickInput);
        if (stickFactor < 0.0f) stickFactor = 0.0f;
    }
    // c. Proportionnal term (P)
    float pTerm = pid->Kp * error_nomalized;

    if (pid->Ki > 0.0f) {
        // d. Integral term (I) with anti-windup and reset when stick is moved
        pid->integralAcc += error_nomalized * dt;
        pid->integralAcc = clampf(pid->integralAcc, -MAX_I_TERM, MAX_I_TERM);
        
        // I term cancellation when stick is moved significantly to avoid integral windup
        if (fast_fabsf(stickInput) > 0.05f) {
            pid->integralAcc = 0.0f;
        }
        iTerm = pid->Ki * pid->integralAcc;
    }

    // e. Derivate Term (D)
    float dTerm = 0.0f;
    if (dt > 0.0f) {
        float rawDerivative = -(measuredRate_low - pid->prevMeasuredRate) / dt;
        float d_normalized = rawDerivative / pid->maxRateDegs;
        dTerm = pid->Kd * d_normalized;
    }
    pid->prevMeasuredRate = measuredRate_low;

    // f. Final gyro correction with master gain and stick factor
    float gyroCorrection = (pTerm + iTerm + dTerm) * masterGain * stickFactor;

    // Invert correction if needed
    if (pid->invert) gyroCorrection = -gyroCorrection;
    float output = stickInput + gyroCorrection;

    // h. Clamp output to [-1.0, 1.0] range
    clampf(output, -MAX_SERVO_OUTPUT, MAX_SERVO_OUTPUT);

    return output;
}

/**
 * @brief Convert a PWM pulse width in microseconds to a target rotation rate in degrees per second, considering deadband and maximum rate.
 * 
 * @param pulse_us     PWM pulse width in microseconds (e.g., 1000 to 2000)
 * @param max_rate_dps Rotation rate limit in degrees per second (e.g., 250.0)
 * @param deadband_us  Half of the deadzone around the center (e.g., 12 us to ignore 1488..1512 us)
 * 
 * @return float Target rotation rate in deg/s
 */
IRAM_ATTR float mapStickToRate(uint16_t pulse_us, float max_rate_dps, uint16_t deadband_us) {
    int32_t offset = (int32_t)pulse_us - 1500;

    if (fast_fabsf(offset) <= deadband_us) {
        return 0.0f;
    }

    if (offset > 0) {
        offset -= deadband_us;
    } else {
        offset += deadband_us;
    }

    // 3. Normalization -1.0f et +1.0f
    float max_range = 500.0f - (float)deadband_us;
    float x = (float)offset / max_range;

    // Clamping to ensure x is within [-1.0, 1.0]
    if (x > 1.0f)  x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    // 5. Conversion in deg/s
    return x * max_rate_dps;
}

#define ALPHA  0.98f // 98% Gyro, 2% Accel

/*
 * @brief Compute the attitude using a complementary filter combining gyro and accelerometer data.
 * @param attitude Pointer to the attitude structure to update.
 * @param ax, ay, az Accelerometer readings in g.
 * @param gyroRollDegS, gyroPitchDegS Gyro readings in deg/s.
 * @param dt Time step in seconds.
 */
IRAM_ATTR void compute_attitude(attitude_t *attitude, float ax, float ay, float az, float gyroRollDegS, float gyroPitchDegS, float dt) {
    if (dt <= 0.00001f) return;
    
    float accelNorm = fast_sqrtf(ay * ay + az * az);
    float accelRoll  = fast_atan2f(ay, az) * RAD_TO_DEG;
    float accelPitch = (accelNorm > 0.001f) ? fast_atan2f(-ax, accelNorm) * RAD_TO_DEG : attitude->pitchDeg;

    attitude->rollDeg  = ALPHA * (attitude->rollDeg  + gyroRollDegS  * dt) + (1.0f - ALPHA) * accelRoll;
    attitude->pitchDeg = ALPHA * (attitude->pitchDeg + gyroPitchDegS * dt) + (1.0f - ALPHA) * accelPitch;
}

typedef struct {
    float q0, q1, q2, q3;
    float ix, iy, iz;    // integral error accumulators
} MahonyFilter;

static MahonyFilter mahony = { .q0 = 1.0f, .q1 = 0.0f, .q2 = 0.0f, .q3 = 0.0f, .ix = 0.0f, .iy = 0.0f, .iz = 0.0f };

/*
 * @brief  Compute the attitude using a Mahony filter with new sensor readings.
 * @param gx, gy, gz Gyro readings in rad/s.
 * @param ax, ay, az Accelerometer readings in g.
 * @param dt Time step in seconds.
 */
IRAM_ATTR void mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float q0 = mahony.q0, q1 = mahony.q1, q2 = mahony.q2, q3 = mahony.q3;
    float norm;

    // 1. Normalisation de l'accéléromètre
    norm = ax * ax + ay * ay + az * az;
    if (norm > 0.0001f) {
        norm = fast_inv_sqrtf(norm);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // 2. Estimation de la direction de la gravité à partir du quaternion actuel
        float vx = 2.0f * (q1 * q3 - q0 * q2);
        float vy = 2.0f * (q0 * q1 + q2 * q3);
        float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

        // 3. Calcul de l'erreur (produit vectoriel entre gravité mesurée et estimée)
        float ex = (ay * vz - az * vy);
        float ey = (az * vx - ax * vz);
        float ez = (ax * vy - ay * vx);

        // 4. Correction intégrale (optionnelle)
        if (MAHONY_KI > 0.0f) {
            mahony.ix += ex * MAHONY_KI * dt;
            mahony.iy += ey * MAHONY_KI * dt;
            mahony.iz += ez * MAHONY_KI * dt;
            gx += mahony.ix;
            gy += mahony.iy;
            gz += mahony.iz;
        }

        // 5. Application de la correction proportionnelle
        gx += MAHONY_KP * ex;
        gy += MAHONY_KP * ey;
        gz += MAHONY_KP * ez;
    }

    // 6. Intégration du quaternion par la méthode d'Euler (0.5 * q x w)
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 7. Re-normalisation impérative du quaternion
    norm = fast_inv_sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    mahony.q0 = q0 * norm;
    mahony.q1 = q1 * norm;
    mahony.q2 = q2 * norm;
    mahony.q3 = q3 * norm;
}

// Euler angle extractor (-180° à +180° pour Roll, -90° à +90° pour Pitch)
IRAM_ATTR void mahony_get_euler(attitude_t* attidude) {
    float q0 = mahony.q0, q1 = mahony.q1, q2 = mahony.q2, q3 = mahony.q3;

    // Roll : -180° à +180°
    attidude->rollDeg = fast_atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;

    // Pitch  : -90° à +90° with asin out of range protection
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f) {
        attidude->pitchDeg = copysignf(90.0f, sinp); // ±90° si au zénith/nadir
    } else {
        attidude->pitchDeg = fast_asinf(sinp) * RAD_TO_DEG;
    }

    // Yaw (cap) : -180° à +180° (dérive sans magnétomètre, mais utilisable en relatif)
    // *yaw = fast_atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
}