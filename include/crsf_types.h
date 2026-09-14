#pragma once

#include <stdint.h>

typedef enum
{
    CRSF_FRAMETYPE_GPS                  = 0x02,
    CRSF_FRAMETYPE_VARIO                = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR       = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE        = 0x09,
    CRSF_FRAMETYPE_HEARTBEAT            = 0x0B,  //no need to support? (rev07)
    CRSF_FRAMETYPE_CELLS_SENSOR         = 0x0e,
    CRSF_FRAMETYPE_VIDEO_TRANSMITTER    = 0x0F,  //no need to support? (rev07)
    CRSF_FRAMETYPE_LINK_STATISTICS      = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC          = 0x10,  //not in edgeTX
    CRSF_FRAMETYPE_RADIO_ID             = 0x3A,  //no need to support?
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED   = 0x16,
    CRSF_FRAMETYPE_LINK_RX_ID           = 0x1C,  //no need to support?
    CRSF_FRAMETYPE_LINK_TX_ID           = 0x1D,  //no need to support?
    CRSF_FRAMETYPE_ATTITUDE             = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE          = 0x21,  //no need to support?
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING          = 0x28,  //no "flight controller" needs to know about this
    CRSF_FRAMETYPE_DEVICE_INFO          = 0x29,  //no "flight controller" needs to know about this
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,  //no "flight controller" needs to know about this
    CRSF_FRAMETYPE_PARAMETER_READ       = 0x2C,  //no "flight controller" needs to know about this
    CRSF_FRAMETYPE_PARAMETER_WRITE      = 0x2D,  //no "flight controller" needs to know about this
    CRSF_FRAMETYPE_COMMAND              = 0x32,  //no "flight controller" needs to know about this
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ             = 0x78,  //not in edgeTX
    CRSF_FRAMETYPE_KISS_RESP            = 0x79,  //not in edgeTX
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ              = 0x7A,  //not in edgeTX
    CRSF_FRAMETYPE_MSP_RESP             = 0x7B,  //not in edgeTX
    CRSF_FRAMETYPE_MSP_WRITE            = 0x7C,  //not in edgeTX
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP       = 0x80,
} crsf_frame_type_e;

typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
} crsf_addr_e;

#define CRSF_GPS_PAYLOAD_SIZE sizeof(crsf_telemetry_gps_t)
#define CRSF_FRAME_SIZE (1 + 1 + 1 + CRSF_GPS_PAYLOAD_SIZE + 1)

// CRSF Telemetry packet header
typedef struct __attribute__((packed)) {
    uint8_t device_address; // Target device address
    uint8_t length;         // Length of type + payload + crc
    uint8_t frame_type;     // Type of telemetry frame
} crsf_header_t;

typedef struct __attribute__((packed)) {
    uint16_t voltage;     // Voltage (V * 10) -> e.g. 126 = 12.6V
    uint16_t current;     // Current (A * 10) -> e.g. 45  = 4.5A
    uint8_t capacity[3];    // Fuel/Capacity drawn in mAh (Stored as 3 bytes)
    uint8_t  remaining;   // Remaining battery capacity in % (0 to 100)
} crsf_telemetry_battery_t;

typedef struct __attribute__((packed)) {
    int32_t latitude;   // Latitude (Degrees * 10^-7) - Direct mapping from u-blox
    int32_t longitude;  // Longitude (Degrees * 10^-7) - Direct mapping from u-blox
    uint16_t ground_speed; // Ground speed (km/h * 10)
    uint16_t heading;   // GPS heading (Degrees * 100)
    uint16_t altitude;  // Altitude (meters + 1000m offset)
    uint8_t satellites; // Number of satellites
} crsf_telemetry_gps_t;