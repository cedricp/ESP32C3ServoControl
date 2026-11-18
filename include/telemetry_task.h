#pragma once

#include <stdint.h>

#define UBX_NAV_POSLLH_LEN 28
#define UBX_NAV_PVT_LEN    92

typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    uint8_t  valid;
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;
    int32_t  lat;
    int32_t  height;
    int32_t  hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t  velN, velE, velD;
    int32_t  gSpeed;
    int32_t  headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t  reserved1[6];   // 6 octets (M8: 6, +4 avec headVeh/magDec)
    int32_t  headVeh;        // seulement sur M8/M9 (octets 84-87)
    int16_t  magDec;
    uint16_t magAcc;
} ubx_nav_pvt_t;

typedef struct __attribute__((packed)) {
    uint32_t iTOW;   // ms
    int32_t  lon;    // 1e-7 deg
    int32_t  lat;    // 1e-7 deg
    int32_t  height; // mm, ellipsoïde
    int32_t  hMSL;   // mm
    uint32_t hAcc;   // mm
    uint32_t vAcc;   // mm
} ubx_nav_posllh_t;

typedef struct {
    uint8_t  temperature;       // °C
    uint32_t voltage_mv;        // cVolts
    uint32_t current_ma;        // cAmpères
    uint16_t mah;               // mAh
    uint32_t erpm;              // Tours/min électriques
} esc_telemetry_t;

void telemetry_task(void *pvParameters);
void gps_init();