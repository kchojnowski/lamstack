#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>

#include "stm32l0xx_hal.h"

class Logger
{
public:
    Logger();
    void init(void);
    void print(const char* log);
    void print(int val);
    void printWithNewline(const char* log);
    void printAsHex(uint8_t* log, uint16_t length);
    void printAsHex(uint8_t log);
    void printTime(void);

private:
    UART_HandleTypeDef uartLogHandle;
};

#endif


