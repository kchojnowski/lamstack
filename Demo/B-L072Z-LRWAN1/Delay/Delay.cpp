#include "FreeRTOS.h"
#include "task.h"
#include "Delay.h"

void Delay::delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

