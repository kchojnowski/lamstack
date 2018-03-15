#ifndef INITTASK_H
#define INITTASK_H

#include "FreeRTOS.h"
#include "task.h"

class InitTask
{
public:
    static void task(void* args);
    static TaskHandle_t handle;
};

#endif
