#pragma once

typedef struct  {
    float state;       // Mémoire de la dernière valeur filtrée
    float alpha;       // Coefficient de lissage précalculé
    float cutoffFreq;  // Fréquence de coupure en Hz (ex: 30 Hz à 90 Hz)
} filter_pt1;

inline void init_pt1_filter(filter_pt1 *filter, float cutoffFreq, float dt) {
    filter->cutoffFreq = cutoffFreq;
    // Compute time constant Tau (rc = 1 / (2 * pi * f_c))
    float rc = 1.0f / (2.0f * M_PI * cutoffFreq);
    // Calculate alpha based on the time constant and the sampling interval
    filter->alpha = dt / (rc + dt);
}

inline float apply_pt1_filter(filter_pt1 *filter, float rawInput) {
    // new_state = old_state + alpha * (raw_input - old_state)
    filter->state = filter->state + filter->alpha * (rawInput - filter->state);
    
    return filter->state;
}

typedef struct LPF_U32 {
    uint8_t  shift_factor;
    uint32_t filtered_acc;
} lpf_u32_t;


inline void lpf_u32_init(lpf_u32_t* filter, uint32_t initial_val, uint8_t alpha) {
    filter->shift_factor = alpha;
    filter->filtered_acc = (uint32_t)initial_val << alpha;
}

inline void lpf_u32_update(lpf_u32_t* filter, uint32_t input) {
    filter->filtered_acc = filter->filtered_acc - (filter->filtered_acc >> filter->shift_factor) + input;
}

inline uint32_t lpf_u32_value(const lpf_u32_t* filter) {
    return (uint32_t)(filter->filtered_acc >> filter->shift_factor);
}