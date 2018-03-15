#ifndef SPICONTROLLER_H
#define SPICONTROLLER_H

#include <stdint.h>

class SpiController
{
public:
    SpiController();
    void init(void);
    uint8_t readReg(uint8_t addr);
    void readReg(uint8_t addr, uint8_t* data, uint32_t len);
    void writeReg(uint8_t addr, uint8_t data);
    void writeReg(uint8_t addr, uint8_t* data, uint32_t len);
};

#endif
