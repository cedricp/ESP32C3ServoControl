#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "types.h"
#include <utils.h>
#include "pid.h"

#include "esp_attr.h"

#define MAX_I_TERM       0.2f
#define MAX_SERVO_OUTPUT 1.0f

float nomaliseStick(uint16_t pulse_us)
{
    return (float)(pulse_us - 1500.0f) / 500.0f;
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
IRAM_ATTR float computeAxisPID(float stickInput, float targetRate, float measuredRate, float measuredRate_low, float dt, float masterGain, PID_Config_t* pid) {
    
    // a. Angular rate error normalized to [-1.0, 1.0] range
    float error_nomalized = (targetRate - measuredRate) / pid->maxRateDegs;

    // b. Stick Derating : reduce correction when stick is near the end of travel to avoid overshoot
    float stickFactor = 1.0f - fast_fabsf(stickInput);
    if (stickFactor < 0.0f) stickFactor = 0.0f;

    // c. Proportionnal term (P)
    float pTerm = pid->Kp * error_nomalized;

    // d. Integral term (I) with anti-windup and reset when stick is moved
    pid->integralAcc += error_nomalized * dt;
    if (pid->integralAcc > MAX_I_TERM)  pid->integralAcc = MAX_I_TERM;
    if (pid->integralAcc < -MAX_I_TERM) pid->integralAcc = -MAX_I_TERM;
    
    // I term cancellation when stick is moved significantly to avoid integral windup
    if (fast_fabsf(stickInput) > 0.05f) {
        pid->integralAcc = 0.0f;
    }
    float iTerm = pid->Ki * pid->integralAcc;

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
    if (output > MAX_SERVO_OUTPUT)  output = MAX_SERVO_OUTPUT;
    if (output < -MAX_SERVO_OUTPUT) output = -MAX_SERVO_OUTPUT;

    return output;
}

/**
 * @brief Convertit un signal de manche RC en vitesse de rotation cible (deg/s).
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

static float RAD_TO_DEG = 57.295779513f;
static float ALPHA = 0.98f; // 98% Gyro, 2% Accel

IRAM_ATTR void computeAttitude(attitude_t *attitude, float ax, float ay, float az, float gyroRollDegS, float gyroPitchDegS, float dt) {
    if (dt <= 0.00001f) return;
    
    float accelNorm = fast_sqrtf(ay * ay + az * az);
    float accelRoll  = fast_atan2(ay, az) * RAD_TO_DEG;
    float accelPitch = (accelNorm > 0.001f) ? fast_atan2(-ax, accelNorm) * RAD_TO_DEG : attitude->pitchDeg;;

    attitude->rollDeg  = ALPHA * (attitude->rollDeg  + gyroRollDegS  * dt) + (1.0f - ALPHA) * accelRoll;
    attitude->pitchDeg = ALPHA * (attitude->pitchDeg + gyroPitchDegS * dt) + (1.0f - ALPHA) * accelPitch;
}