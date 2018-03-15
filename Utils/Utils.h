#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

class Utils
{
public:
    static uint8_t byte2Hex(uint8_t value);
    static uint8_t int2str(int value, char* str);
    static int str2int(char* str);
    static int len(const char* str);

};

#endif
