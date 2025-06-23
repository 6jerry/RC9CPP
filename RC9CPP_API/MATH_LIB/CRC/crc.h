#ifndef CRC_H
#define CRC_H



#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

#define crclen 256

class GENERIC_CRC8
{
private:
    uint8_t crc8tab[crclen];
    uint8_t crcpoly;

public:
    GENERIC_CRC8(uint8_t poly);
    uint8_t calc(const uint8_t data);
    uint8_t calc(const uint8_t *data, uint16_t len, uint8_t crc = 0);
};
#endif

#endif