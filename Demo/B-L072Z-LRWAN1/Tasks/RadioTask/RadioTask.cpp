#include <stdlib.h>
#include "SpiController.h"
#include "RadioTask.h"

TaskHandle_t RadioTask::handle = 0;
QueueHandle_t RadioTask::txQueueHandle = 0;

RadioTask::RadioTask(SpiController& spi, RadioConfig& config, EventHandler& evtHandler, Logger& log) : rm(spi, evtHandler, log)
{
    this->rm.setConfig(config);
    this->rm.init();
}

void RadioTask::task(void* args)
{
    RadioConfig config;
    for(int i=0; i<MAX_NODES; i++)
        config.routingTable[i] = i;

    config.nodeId = 1;
    config.address = 2;
    config.carrierFrequency = 868000000;
    config.bandwidth = Sx127x_BW_125_KHZ;
    config.spreadingFactor = Sx127x_SF_128;
    config.codingRate = Sx127x_CR_4_5;
    config.outputPower = 10;

    Logger log;
    log.init();

    SpiController spi;
    EventHandler evtHandler;
    evtHandler.init();

    RadioTask r(spi, config, evtHandler, log);
    RadioTxPacket p;

    uint32_t maxQueueWaitMs = 250;
    TickType_t maxQueueWait = maxQueueWaitMs / portTICK_PERIOD_MS;
    TickType_t lastRadioUpdateTicks = xTaskGetTickCount();

    while(1)
    {
        uint32_t elapsedMs = (xTaskGetTickCount() - lastRadioUpdateTicks) * portTICK_PERIOD_MS;
        lastRadioUpdateTicks = xTaskGetTickCount();
        r.rm.process(elapsedMs);

        if (xQueueReceive(r.txQueueHandle, (void*)&p, maxQueueWait) == pdPASS) {
            p.src = config.address;
            r.rm.sendPacket(p);
        }
    }
}
