#include <stdint.h>
#include <math.h>
#include "pid.h"

#define MAX_I_TERM       0.2f
#define MAX_SERVO_OUTPUT 1.0f

float nomaliseStick(uint16_t pulse_us)
{
    return (float)(pulse_us - 1500.0f) / 500.0f;
}

/**
 * @brief Calcule la sortie servo pour un axe avec stabilisation.
 * @param stickInput   Ordre direct de la radio [-1.0 à 1.0]
 * @param targetRate   Vitesse de rotation cible (deg/s)
 * @param measuredRate Vitesse de rotation mesurée par le gyro (deg/s)
 * @param dt           Delta time entre deux boucles (secondes)
 * @param masterGain   Gain global radio [0.0 à 1.0]
 * @param pid          Pointeur vers la structure PID de l'axe
 */
float computeAxisPID(float stickInput, float targetRate, float measuredRate, float dt, float masterGain, float max_rate_degs, PID_Config_t* pid) {
    
    // a. Calcul de l'erreur de vitesse angulaire
    float error_nomalized = (targetRate - measuredRate) / pid->maxRateDegs;

    // b. Stick Derating : réduction de la correction si le pilote agit sur le manche
    float stickFactor = 1.0f - fabsf(stickInput);
    if (stickFactor < 0.0f) stickFactor = 0.0f;

    // c. Terme Proportionnel (P)
    float pTerm = pid->Kp * error_nomalized;

    // d. Terme Intégral (I) avec protection Anti-Windup
    pid->integralAcc += error_nomalized * dt;
    if (pid->integralAcc > MAX_I_TERM)  pid->integralAcc = MAX_I_TERM;
    if (pid->integralAcc < -MAX_I_TERM) pid->integralAcc = -MAX_I_TERM;
    
    // Annulation du terme I si le pilote effectue une manœuvre (évite le décalage d'attitude)
    if (fabsf(stickInput) > 0.05f) {
        pid->integralAcc = 0.0f;
    }
    float iTerm = pid->Ki * pid->integralAcc;

    // e. Terme Dérivé (D)
    float dTerm = 0.0f;
    if (dt > 0.0f) {
        dTerm = pid->Kd * (error_nomalized - pid->prevMeasuredRate) / dt;
    }
    pid->prevMeasuredRate = measuredRate / pid->maxRateDegs;

    // f. Calcul de la correction Gyro globale pondérée par Master Gain et Stick Derating
    float gyroCorrection = (pTerm + iTerm + dTerm) * masterGain * stickFactor;

    // g. Superposition de la commande directe pilote et de la correction

    if (pid->invert) gyroCorrection = -gyroCorrection;
    float output = stickInput + gyroCorrection;

    // h. Limitation du débattement servo final
    if (output > MAX_SERVO_OUTPUT)  output = MAX_SERVO_OUTPUT;
    if (output < -MAX_SERVO_OUTPUT) output = -MAX_SERVO_OUTPUT;

    return output;
}

/**
 * @brief Convertit un signal de manche RC en vitesse de rotation cible (deg/s).
 * 
 * @param pulse_us     Largeur d'impulsion du canal (ex: 1000 à 2000 us, neutre à 1500 us)
 * @param max_rate_dps Taux de rotation maximum souhaité en bout de manche (ex: 200.0 deg/s)
 * @param deadband_us  Demi-zone morte autour du neutre (ex: 12 us pour ignorer 1488..1512 us)
 * 
 * @return float Consigne de vitesse de rotation en deg/s
 */
float mapStickToRate(uint16_t pulse_us, float max_rate_dps, uint16_t deadband_us) {
    // 1. Calcul de l'écart par rapport au neutre (1500 µs)
    int32_t offset = (int32_t)pulse_us - 1500;

    // 2. Gestion de la zone morte (Deadband)
    if (abs(offset) <= deadband_us) {
        return 0.0f;
    }

    // Soustraction de la zone morte pour repartir de 0 dès la sortie du neutre (transition douce)
    if (offset > 0) {
        offset -= deadband_us;
    } else {
        offset += deadband_us;
    }

    // 3. Normalisation entre -1.0f et +1.0f
    float max_range = 500.0f - (float)deadband_us;
    float x = (float)offset / max_range;

    // Saturation de sécurité (au cas où la radio envoie 980 us ou 2020 us)
    if (x > 1.0f)  x = 1.0f;
    if (x < -1.0f) x = -1.0f;

    // 5. Conversion en deg/s
    return x * max_rate_dps;
}