#ifndef CRC_H
#define CRC_H

#include <stdint.h>

class Crc
{
public:
    static uint8_t crc8(const uint8_t* data, uint32_t len);
};

#endif
