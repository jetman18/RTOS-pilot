################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/flight/attitude_ctrl.c \
../Core/flight/estimate.c \
../Core/flight/mavlink_handler.c \
../Core/flight/navigation.c 

OBJS += \
./Core/flight/attitude_ctrl.o \
./Core/flight/estimate.o \
./Core/flight/mavlink_handler.o \
./Core/flight/navigation.o 

C_DEPS += \
./Core/flight/attitude_ctrl.d \
./Core/flight/estimate.d \
./Core/flight/mavlink_handler.d \
./Core/flight/navigation.d 


# Each subdirectory must supply rules for building sources it contributes
Core/flight/%.o: ../Core/flight/%.c Core/flight/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/tuan/Documents/stm/RTOSpilot/Core/mavlink" -I"C:/Users/tuan/Documents/stm/RTOSpilot/Core/Driver" -I"C:/Users/tuan/Documents/stm/RTOSpilot/Core/epprom" -I"C:/Users/tuan/Documents/stm/RTOSpilot/Core/flight" -I"C:/Users/tuan/Documents/stm/RTOSpilot/Core/Lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-flight

clean-Core-2f-flight:
	-$(RM) ./Core/flight/attitude_ctrl.d ./Core/flight/attitude_ctrl.o ./Core/flight/estimate.d ./Core/flight/estimate.o ./Core/flight/mavlink_handler.d ./Core/flight/mavlink_handler.o ./Core/flight/navigation.d ./Core/flight/navigation.o

.PHONY: clean-Core-2f-flight

