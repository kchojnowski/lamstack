#include <string.h>

#include "Utils.h"
#include "Logger.h"

Logger::Logger()
{
    uint8_t* uartHandlePtr = (uint8_t*)(&this->uartLogHandle);
    for(uint32_t i=0; i<sizeof(UART_HandleTypeDef); i++)
        uartHandlePtr[i] = 0;
}

void Logger::init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    this->uartLogHandle.Instance = USART2;
    this->uartLogHandle.Init.BaudRate = 115200;
    this->uartLogHandle.Init.WordLength = UART_WORDLENGTH_8B;
    this->uartLogHandle.Init.StopBits = UART_STOPBITS_1;
    this->uartLogHandle.Init.Parity = UART_PARITY_NONE;
    this->uartLogHandle.Init.Mode = UART_MODE_TX_RX;
    this->uartLogHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    this->uartLogHandle.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&this->uartLogHandle);
}

void Logger::print(const char* log)
{
    HAL_UART_Transmit(&this->uartLogHandle, (uint8_t*)log, strlen(log), 1000);
}

void Logger::printWithDate(const char* log)
{
    this->print(log);
}

void Logger::printWithNewline(const char* log)
{
    HAL_UART_Transmit(&this->uartLogHandle, (uint8_t*)log, strlen(log), 1000);
    HAL_UART_Transmit(&this->uartLogHandle, (uint8_t*)"\r\n", 2, 1000);
}

void Logger::print(int val)
{
    char digits[10];
    uint8_t len = Utils::int2str(val, digits);
    digits[len] = '\0';
    this->print(digits);
}

void Logger::printAsHex(uint8_t* log, uint16_t length)
{
    for(int i = 0; i < length; i++)
    {
        this->printAsHex(log[i]);
        this->print(" ");
    }
}

void Logger::printAsHex(uint8_t value)
{
    uint8_t hex[3];
    hex[0] = Utils::byte2Hex((value >> 4) & 0xF);
    hex[1] = Utils::byte2Hex(value & 0xF);
    hex[2] = '\0';
    this->print((const char*)hex);
}
