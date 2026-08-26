#pragma once

#include <stdint.h>
#include <math.h>
#include "esp_err.h"

#define DEG_TO_RAD (M_PI / 180.0f)
#define RAD_TO_DEG (180.0f / M_PI)

#define NUM_PWM_OUPUTS 6
#define NUM_CRSF_CHANNELS 8

const char *reset_reason_to_str(uint8_t reason);
void check_i2c(int gpio_sda, int gpio_scl);

esp_err_t nvs_save_struct(const char *key, const void *data, size_t size);
esp_err_t nvs_load_struct(const char *key, void *data, size_t size);

typedef struct
{
    uint16_t us_values[NUM_CRSF_CHANNELS];
    char valid;
} servo_data_t;

inline float __attribute__((always_inline)) fast_fabsf(float x)
{
    union
    {
        float f;
        uint32_t i;
    } conv = {x};
    conv.i &= 0x7FFFFFFF; // Force le bit de signe à 0
    return conv.f;
}

inline float __attribute__((always_inline)) fast_atan2f(float y, float x)
{
    if (x == 0.0f && y == 0.0f)
        return 0.0f;

    union
    {
        float f;
        uint32_t i;
    } u = {y};
    u.i &= 0x7FFFFFFF;
    float abs_y = u.f + 1e-10f;

    float r, angle;
    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.1963f * r * r * r - 0.9817f * r + 0.78539816f; // PI/4
    }
    else
    {
        r = (x + abs_y) / (abs_y - x);
        angle = 0.1963f * r * r * r - 0.9817f * r + 2.35619449f; // 3*PI/4
    }

    return (y < 0.0f) ? -angle : angle;
}

inline float __attribute__((always_inline)) fast_sqrtf(float x)
{
    if (x <= 0.0f)
        return 0.0f;

    union
    {
        float f;
        int32_t i;
    } conv = {x};

    conv.i = 0x1fbd1df5 + (conv.i >> 1);
    conv.f = 0.5f * (conv.f + x / conv.f);

    return conv.f;
}

inline float __attribute__((always_inline)) fast_inv_sqrtf(float x)
{
    float halfx = 0.5f * x;
    union
    {
        float f;
        uint32_t i;
    } conv = {x};
    conv.i = 0x5f3759df - (conv.i >> 1);
    conv.f = conv.f * (1.5f - (halfx * conv.f * conv.f));
    return conv.f;
}

inline float __attribute__((always_inline)) fast_asinf(float x)
{
    float abs_x = fast_fabsf(x);

    if (abs_x >= 1.0f)
    {
        return (x > 0.0f) ? 1.57079632679f : -1.57079632679f;
    }

    float poly = -0.0187293f;
    poly = poly * abs_x + 0.0742610f;
    poly = poly * abs_x - 0.2121144f;
    poly = poly * abs_x + 1.5707288f;

    float res = 1.57079632679f - fast_sqrtf(1.0f - abs_x) * poly;

    return (x < 0.0f) ? -res : res;
}

inline float __attribute__((always_inline)) clampf(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

inline uint16_t __attribute__((always_inline)) clampu(uint16_t value, uint16_t min, uint16_t max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

inline uint16_t __attribute__((always_inline)) map_to_pwm(float x)
{
    return (uint16_t)((x + 1.0f) * 500.0f + 1000.0f);
}
