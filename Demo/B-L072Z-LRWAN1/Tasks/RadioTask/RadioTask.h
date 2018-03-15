#ifndef RADIOTASK_H
#define RADIOTASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Sx127x.h"
#include "SpiController.h"
#include "Logger.h"
#include "RadioManager.h"
#include "EventHandler.h"

class RadioTask
{
public:
    RadioTask(SpiController& spi, RadioConfig& config, EventHandler& evtHandler, Logger& log);
    static void task(void* args);
    static TaskHandle_t handle;
    static QueueHandle_t txQueueHandle;

private:
    RadioManager<SpiController, EventHandler, Logger> rm;
};

#endif
