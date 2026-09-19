#pragma once

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint32_t iTOW;         // GPS Time of Week (ms)
    uint16_t year;         // Year (UTC)
    uint8_t  month;        // Month (UTC)
    uint8_t  day;          // Day (UTC)
    uint8_t  hour;         // Hour (UTC)
    uint8_t  min;          // Minute (UTC)
    uint8_t  sec;          // Second (UTC)
    uint8_t  valid;        // Validity flags
    uint32_t tAcc;         // Time accuracy estimate (ns)
    int32_t  nano;         // Fraction of second (ns), range -1e9 to 1e9
    uint8_t  fixType;      // GNSS fix Type (0=No fix, 3=3D fix, etc.)
    uint8_t  flags;        // Navigation status flags
    uint8_t  flags2;       // Additional flags
    uint8_t  numSV;        // Number of satellites used in Nav Solution
    int32_t  lon;          // Longitude (Degrees * 10^-7)
    int32_t  lat;          // Latitude (Degrees * 10^-7)
    int32_t  height;       // Height above ellipsoid (mm)
    int32_t  hMSL;         // Height above mean sea level (mm)
    uint32_t hAcc;         // Horizontal accuracy estimate (mm)
    uint32_t vAcc;         // Vertical accuracy estimate (mm)
    int32_t  velN;         // NED north velocity (mm/s)
    int32_t  velE;         // NED east velocity (mm/s)
    int32_t  velD;         // NED down velocity (mm/s)
    int32_t  gSpeed;       // Ground Speed (2D) (mm/s)
    int32_t  heading;      // Heading of motion (Degrees * 10^-5)
    uint32_t sAcc;         // Speed accuracy estimate (mm/s)
    uint32_t headAcc;      // Heading accuracy estimate (Degrees * 10^-5)
    uint16_t pDOP;         // Position DOP (*0.01)
    uint8_t  reserved1[6]; // Reserved
    int32_t  headVeh;      // Heading of vehicle (Degrees * 10^-5)
    uint16_t gDOP;         // Geometric DOP (*0.01)
    uint8_t  reserved2[4]; // Reserved
} ubx_nav_pvt_t;

typedef struct {
    uint8_t  temperature;       // °C
    uint32_t voltage_mv;        // cVolts
    uint32_t current_ma;        // cAmpères
    uint16_t mah;               // mAh
    uint32_t erpm;              // Tours/min électriques
} esc_telemetry_t;

void telemetry_task(void *pvParameters);
void gps_init();