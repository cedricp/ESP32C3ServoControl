#pragma once

typedef struct {
    float Kp;           // Gain Proportionnel (réponse immédiate à l'erreur)
    float Ki;           // Gain Intégral (corrige les dérives lentes)
    float Kd;           // Gain Dérivé (amortit les oscillations)
    float maxRateDegs;  // Vitesse de rotation maximale (ex: 250.0 deg/s)
    float integralAcc;  // Accumulateur de l'erreur intégrale
    float prevError;    // Erreur de la boucle précédente
} PID_Config_t;

float computeAxisPID(float stickInput, float targetRate, float measuredRate, float dt, float masterGain, float max_rate_degs, PID_Config_t* pid);
float mapStickToRate(uint16_t pulse_us, float max_rate_dps, uint16_t deadband_us);
float nomaliseStick(uint16_t pulse_us);