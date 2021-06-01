
HAL = Drivers/STM32F4xx_HAL_Driver/
BSP = Drivers/BSP/STM32F4xx-Nucleo/
CMSIS_DEVICE = Drivers/CMSIS/Device/ST/STM32F4xx/
CMSIS_CORE = Drivers/CMSIS/Core/

INCLUDE = -IInc/
INCLUDE += -I$(HAL)Inc/ 
INCLUDE += -I$(BSP)
INCLUDE += -I$(CMSIS_DEVICE)Include/ 
INCLUDE += -I$(CMSIS_CORE)Include/ 

SOURCE = Src/*
SOURCE += SW4STM32/startup_stm32f446xx.s
SOURCE += $(HAL)Src/*.c 
SOURCE += $(BSP)*.c
av:
	arm-none-eabi-gcc -g --specs=nosys.specs -march=armv7+fp -mfloat-abi=hard -mtune=cortex-m4 -TSW4STM32/STM32446E_Nucleo/STM32F446RETx_FLASH.ld $(SOURCE) $(INCLUDE) -o av
clean:
	rm -f av
	rm -f *.out