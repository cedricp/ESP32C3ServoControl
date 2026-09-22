#pragma once

typedef struct {
    float Kp;           // Gain Proportionnel (réponse immédiate à l'erreur)
    float Ki;           // Gain Intégral (corrige les dérives lentes)
    float Kd;           // Gain Dérivé (amortit les oscillations)
    float maxRateDegs;  // Vitesse de rotation maximale (ex: 250.0 deg/s)
    int invert;        // Inversion du signal de rotation
    char checksum;
    float integralAcc;  // Accumulateur de l'erreur intégrale
    float prevMeasuredRate;
} PID_Config_t;

typedef struct  {
    float rollDeg; // Inclinaison en Roulis (-180° à +180°)
    float pitchDeg; // Inclinaison en Tangage (-90° à +90°)
    float yawDeg; // Orientation en Lacet (0° à 360°)
} attitude_t;

float compute_axis_pid(float stickInput, float targetRate, float measuredRate, float measuredRate_low, float dt, float master_kp_gain, float master_kd_gain, PID_Config_t *pid, char useStickFactor);
float mapStickToRate(uint16_t pulse_us, float max_rate_dps, uint16_t deadband_us);
void  compute_attitude(attitude_t *attitude, float ax, float ay, float az, float gyroRollDegS, float gyroPitchDegS, float dt);
void  mahony_get_euler(attitude_t* attidude);
void  mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void  init_attitude(attitude_t *attitude, float ax, float ay, float az);

inline float nomalise_stick(uint16_t pulse_us)
{
    return ((float)pulse_us - 1500.0f) / 500.0f;
}