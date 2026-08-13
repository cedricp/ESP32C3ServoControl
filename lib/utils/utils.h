# pragma once

#include <stdint.h>

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);
uint16_t crsf_get_channel(int ch, const uint8_t *payload);
uint32_t us_to_ledc_duty(uint32_t us);