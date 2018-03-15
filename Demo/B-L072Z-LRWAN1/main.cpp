#include "stm32l0xx_hal.h"
#include "system_clock.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "InitTask.h"
#include "RadioTask.h"

int main(void)
{
    SystemClock_Config();
    HAL_Init();

    RadioTask::txQueueHandle = xQueueCreate(10, sizeof(RadioTxPacket));

    xTaskCreate(InitTask::task, "initTask", 1024, NULL, 5, &(InitTask::handle));
    xTaskCreate(RadioTask::task, "radioTask", 1024, NULL, 1, &(RadioTask::handle));

    vTaskStartScheduler();
    while(1);
}
