#ifndef RADIORXPACKET_H
#define RADIORXPACKET_H

#include <stdint.h>

#define RADIORXPACKET_FIELDS 9

typedef struct {
    uint8_t dst;
    uint8_t src;
    uint8_t srcrt;
    uint8_t type;
    uint8_t life;
    uint8_t id;
    uint8_t rssi;
    uint8_t snr;
    uint8_t len;
    uint8_t* data;
} RadioRxPacket;

#endif
