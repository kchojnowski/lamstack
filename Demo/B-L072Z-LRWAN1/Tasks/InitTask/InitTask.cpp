#include "InitTask.h"
#include "stm32l0xx_hal.h"

#include "Logger.h"

#include "RadioTxPacket.h"
#include "RadioManager.h"
#include "RadioTask.h"

TaskHandle_t InitTask::handle = 0;

static RadioTxPacket txpPacket;

static void txCallback(TxResult res, RadioTxPacket* txp)
{
    xQueueSend(RadioTask::txQueueHandle, (void*) &txpPacket, portMAX_DELAY);
}


void InitTask::task(void* args)
{
    uint8_t dataBuffer[1] = {PACKET_STAT_ACTIVE};


    txpPacket.dst = 0x01;
    txpPacket.src = 0;
    txpPacket.type = PACKET_TYPE_STAT | PACKET_FLAGS_ACK_REQ;
    txpPacket.life = PACKET_MAX_LIFE;
    txpPacket.id = 0;
    txpPacket.len = 1;
    txpPacket.data = dataBuffer;
    txpPacket.callback = txCallback;
    xQueueSend(RadioTask::txQueueHandle, (void*) &txpPacket, portMAX_DELAY);
    while(1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
