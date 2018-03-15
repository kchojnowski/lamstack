#include "Delay.h"
#include "SpiController.h"

SpiController::SpiController()
{
    uint8_t* spiHandlePtr = (uint8_t*)(&this->spiHandle);
    for(uint32_t i=0; i<sizeof(SPI_HandleTypeDef); i++)
        spiHandlePtr[i] = 0;
}

void SpiController::init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    GPIO_InitTypeDef gpioInit;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioInit.Alternate = GPIO_AF0_SPI1;
    gpioInit.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpioInit);

    gpioInit.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &gpioInit);


    gpioInit.Mode = GPIO_MODE_OUTPUT_PP;
    gpioInit.Pull = GPIO_PULLUP;
    gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
    gpioInit.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOC, &gpioInit);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);


    gpioInit.Pin = GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &gpioInit);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);


    this->spiHandle.Instance = SPI1;
    this->spiHandle.Init.Mode = SPI_MODE_MASTER;
    this->spiHandle.Init.Direction = SPI_DIRECTION_2LINES;
    this->spiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
    this->spiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
    this->spiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
    this->spiHandle.Init.NSS = SPI_NSS_SOFT;
    this->spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    this->spiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
    this->spiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
    this->spiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&this->spiHandle);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
    Delay::delay(10);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
    Delay::delay(10);

}

uint8_t SpiController::readReg(uint8_t addr)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[2];
    txBuf[0] = addr & 0x7F;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&this->spiHandle, txBuf, rxBuf, 2, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

    return rxBuf[1];
}

void SpiController::readReg(uint8_t addr, uint8_t* data, uint32_t len)
{
    uint8_t txBuf;
    txBuf = addr & 0x7F;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(&this->spiHandle, &txBuf, data, 1, 1000);
    txBuf = 0;
    for(uint32_t i = 0; i < len; i++)
        HAL_SPI_TransmitReceive(&this->spiHandle, &txBuf, &data[i], 1, 1000);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void SpiController::writeReg(uint8_t addr, uint8_t data)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[2];
    txBuf[0] = addr | 0x80;
    txBuf[1] = data;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&this->spiHandle, txBuf, rxBuf, 2, 1000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
}

void SpiController::writeReg(uint8_t addr, uint8_t* data, uint32_t len)
{
    addr |= 0x80;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    uint8_t rxBuf;
    HAL_SPI_TransmitReceive(&this->spiHandle, &addr, &rxBuf, 1, 1000);
    for(uint32_t i = 0; i < len; i++)
        HAL_SPI_TransmitReceive(&this->spiHandle, &data[i], &rxBuf, 1, 1000);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

}
