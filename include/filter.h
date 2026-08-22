#pragma once

typedef struct  {
    float state;       // Mémoire de la dernière valeur filtrée
    float alpha;       // Coefficient de lissage précalculé
    float cutoffFreq;  // Fréquence de coupure en Hz (ex: 30 Hz à 90 Hz)
} FilterPT1;

void initPT1Filter(FilterPT1 *filter, float cutoffFreq, float dt) {
    filter->cutoffFreq = cutoffFreq;
    
    // Calcul de la constante de temps Tau (rc = 1 / (2 * pi * f_c))
    float rc = 1.0f / (2.0f * 3.14159265f * cutoffFreq);
    
    // Calcul du coefficient alpha
    filter->alpha = dt / (rc + dt);
    
    // Alternative simplifiée directe :
    // float omega = 2.0 * 3.14159265 * cutoffFreq;
    // filter.alpha = (omega * dt) / (1.0 + omega * dt);
}

// -----------------------------------------------------------------------------
// 3. FONCTION D'EXÉCUTION DU FILTRE (À chaque échantillon)
// -----------------------------------------------------------------------------
float applyPT1Filter(FilterPT1 *filter, float rawInput) {
    // Équation de mise à jour du filtre PT1 :
    // new_state = old_state + alpha * (raw_input - old_state)
    filter->state = filter->state + filter->alpha * (rawInput - filter->state);
    
    return filter->state;
}