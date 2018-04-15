#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

class Logger
{
public:
    Logger();
    void init(void);
    void print(const char* log);
    void printWithDate(const char* log);
    void print(int val);
    void printWithNewline(const char* log);
    void printAsHex(uint8_t* log, uint16_t length);
    void printAsHex(uint8_t log);
};

#endif


