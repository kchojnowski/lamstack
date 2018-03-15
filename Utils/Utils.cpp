#include "Utils.h"

uint8_t Utils::byte2Hex(uint8_t value)
{
  if (value > 9)
    return value + 0x37;
  else
    return value + 0x30;
}

uint8_t Utils::int2str(int value, char* str)
{
    if(value == 0) {
        str[0] = '0';
        return 1;
    }

    uint8_t len = 0;
    bool negative = false;
    if(value < 0) {
        negative = true;
        value *= -1;
    }

    while(value > 0) {
        str[len++] = (value % 10) + 0x30;
        value /= 10;
    }

    if(negative) {
        str[len++] = '-';
    }

    for(int i = 0; i < len/2; i++) {
        char temp = str[i];
        str[i] = str[len - i - 1];
        str[len - i - 1] = temp;
    }

    return len;
}

int Utils::str2int(char* str)
{
    int len = 0;
    while(str[len++] != '\0');

    if(len < 2)
        return 0;

    int factor = 1;
    int value = 0;

    for(int i=len-2; i>=0; i--) {
        value += factor * (str[i] - 0x30);
        factor *= 10;
    }

    return value;
}

int Utils::len(const char* str)
{
    int len = 0;
    while(str[len++] != '\0');
    return len - 1;
}
