#ifndef __CRC_UTIL_H__
#define __CRC_UTIL_H__

#include <stdint.h>

uint16_t CRC16_Table(uint8_t *p, uint8_t counter);
uint8_t CRC8_Table(uint8_t *p, uint8_t counter);
uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint16_t len);
#endif // __CRC_UTIL_H__
