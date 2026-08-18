# pragma once

#include <stdint.h>

#define RAD_TO_DEG 57.295779513f

inline uint32_t __attribute__((always_inline)) us_to_ledc_duty(uint32_t us) {
    return (us * 16384) / 20000;
}

inline float __attribute__((always_inline)) fast_fabsf(float x) {
    union { float f; uint32_t i; } conv = {x};
    conv.i &= 0x7FFFFFFF; // Force le bit de signe à 0
    return conv.f;
}

inline float __attribute__((always_inline)) fast_atan2(float x, float y){
    if (x == 0.0f && y == 0.0f) return 0.0f;

    union { float f; uint32_t i; } u = {y};
    u.i &= 0x7FFFFFFF;
    float abs_y = u.f + 1e-10f;

    float r, angle;
    if (x >= 0.0f) {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.1963f * r * r * r - 0.9817f * r + 0.78539816f; // PI/4
    } else {
        r = (x + abs_y) / (abs_y - x);
        angle = 0.1963f * r * r * r - 0.9817f * r + 2.35619449f; // 3*PI/4
    }

    return (y < 0.0f) ? -angle : angle;
}

inline float __attribute__((always_inline)) fast_sqrtf(float x) {
    if (x <= 0.0f) return 0.0f;

    union {
        float f;
        int32_t i;
    } conv = {x};

    conv.i = 0x1fbd1df5 + (conv.i >> 1);
    conv.f = 0.5f * (conv.f + x / conv.f);

    return conv.f;
}

inline float __attribute__((always_inline)) inv_sqrtf(float x) {
    float halfx = 0.5f * x;
    union {
        float f;
        uint32_t i;
    } conv = {x};
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

inline float __attribute__((always_inline)) clampf(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

inline uint16_t __attribute__((always_inline)) map_to_pwm(float x) {
    return (uint16_t)((x+1) * 500.0f + 1500.0f);
}