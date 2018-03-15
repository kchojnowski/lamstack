AM_CCASFLAGS = -mcpu=cortex-m0plus \
               -mthumb \
               -mfloat-abi=soft

AM_CFLAGS = -O0 \
            -g -g3 \
            -mcpu=cortex-m0plus \
            -mthumb \
            -mfloat-abi=soft \
            -DSTM32L072xx \
            -fdata-sections \
            -ffunction-sections \
            -Wl,--gc-sections \
            -I$(top_builddir)/CMSIS/Include \
            -I$(top_builddir)/CMSIS/Device/ST/STM32L0xx/Include \
            -I$(top_builddir)/STM32L0xx_HAL_Driver/Inc \
            -I$(top_builddir)/../../FreeRTOS/Source/include \
            -I$(top_builddir)/../../FreeRTOS/Source/CMSIS_RTOS \
            -I$(top_builddir)/../../FreeRTOS/Source/portable/GCC/ARM_CM0 \
            -I$(top_builddir)/Tasks/InitTask \
            -I$(top_builddir)/Tasks/RadioTask \
            -I$(top_builddir)/../../Drivers/Sx127x \
            -I$(top_builddir)/SpiController \
            -I$(top_builddir)/../../RadioManager \
            -I$(top_builddir)/EventHandler \
            -I$(top_builddir)/Logger \
            -I$(top_builddir)/Delay \
            -I$(top_builddir)/../../Crc \
            -I$(top_builddir)/../../Utils

AM_CXXFLAGS = ${AM_CFLAGS} \
              -fno-exceptions \
              -fno-rtti \
              -std=c++11

noinst_PROGRAMS = lamstack_demo

lamstack_demo_SOURCES = startup_stm32l072xx.s \
                        system_stm32l0xx.c \
                        stm32l0xx_it.c \
                        main.cpp \
                        system_clock.c \
                        STM32L0xx_HAL_Driver/Src/stm32l0xx_hal.c \
                        STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_cortex.c \
                        STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc.c \
                        STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_rcc_ex.c \
                        STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_gpio.c \
                        STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_spi.c \
                        STM32L0xx_HAL_Driver/Src/stm32l0xx_hal_uart.c \
                        ../../FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
                        ../../FreeRTOS/Source/tasks.c \
                        ../../FreeRTOS/Source/list.c \
                        ../../FreeRTOS/Source/timers.c \
                        ../../FreeRTOS/Source/queue.c \
                        ../../FreeRTOS/Source/portable/GCC/ARM_CM0/port.c \
                        ../../FreeRTOS/Source/portable/MemMang/heap_1.c \
                        SpiController/SpiController.cpp \
                        EventHandler/EventHandler.cpp \
                        Delay/Delay.cpp \
                        Logger/Logger.cpp \
                        ../../Utils/Utils.cpp \
                        ../../Crc/Crc.cpp \
                        Tasks/InitTask/InitTask.cpp \
                        Tasks/RadioTask/RadioTask.cpp


lamstack_demo_LDFLAGS = -mcpu=cortex-m0plus \
                        -mthumb \
                        -mfloat-abi=soft \
                        -T$(top_builddir)/stm32_flash.ld

all:
	arm-none-eabi-objcopy -O binary lamstack_demo lamstack_demo.bin
	arm-none-eabi-size lamstack_demo

flash: all
	openocd -f interface/stlink-v2-1.cfg -f target/stm32l0.cfg -c "program lamstack_demo verify reset exit"
