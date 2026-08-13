#include "utils.h"


uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}

uint16_t crsf_get_channel(int ch, const uint8_t *payload) {
    int bit_offset = ch * 11;
    int byte_index = bit_offset >> 3;        // bit_offset / 8
    int bit_shift  = bit_offset & 0x07;      // bit_offset % 8

    // Read 3 bytes to guarantee 11 bits are always available regardless of alignment
    uint32_t raw = (uint32_t)payload[byte_index]
                 | (uint32_t)payload[byte_index + 1] << 8
                 | (uint32_t)payload[byte_index + 2] << 16;

    return (raw >> bit_shift) & 0x07FF;
}

uint32_t us_to_ledc_duty(uint32_t us) {
    return (us * 16384) / 20000;
}