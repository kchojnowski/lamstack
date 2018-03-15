#ifndef RADIOCONFIG_H
#define RADIOCONFIG_H

#include <stdint.h>
#include "Sx127x.h"

#define MAX_NODES 32

typedef struct {
    uint8_t nodeId;
    uint8_t address;
    uint8_t routingTable[MAX_NODES];
    uint32_t carrierFrequency;
    Sx127x_BW bandwidth;
    Sx127x_SF spreadingFactor;
    Sx127x_CR codingRate;
    uint8_t outputPower;
} RadioConfig;

#endif
