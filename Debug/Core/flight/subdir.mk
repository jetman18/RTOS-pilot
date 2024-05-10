################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/flight/attitude.c \
../Core/flight/estimate.c \
../Core/flight/mavlink_handler.c \
../Core/flight/navigation.c \
../Core/flight/plane.c 

OBJS += \
./Core/flight/attitude.o \
./Core/flight/estimate.o \
./Core/flight/mavlink_handler.o \
./Core/flight/navigation.o \
./Core/flight/plane.o 

C_DEPS += \
./Core/flight/attitude.d \
./Core/flight/estimate.d \
./Core/flight/mavlink_handler.d \
./Core/flight/navigation.d \
./Core/flight/plane.d 


# Each subdirectory must supply rules for building sources it contributes
Core/flight/%.o Core/flight/%.su Core/flight/%.cyclo: ../Core/flight/%.c Core/flight/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/flight" -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/mavlink" -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/simulation" -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/epprom" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FatFs/src/drivers -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-flight

clean-Core-2f-flight:
	-$(RM) ./Core/flight/attitude.cyclo ./Core/flight/attitude.d ./Core/flight/attitude.o ./Core/flight/attitude.su ./Core/flight/estimate.cyclo ./Core/flight/estimate.d ./Core/flight/estimate.o ./Core/flight/estimate.su ./Core/flight/mavlink_handler.cyclo ./Core/flight/mavlink_handler.d ./Core/flight/mavlink_handler.o ./Core/flight/mavlink_handler.su ./Core/flight/navigation.cyclo ./Core/flight/navigation.d ./Core/flight/navigation.o ./Core/flight/navigation.su ./Core/flight/plane.cyclo ./Core/flight/plane.d ./Core/flight/plane.o ./Core/flight/plane.su

.PHONY: clean-Core-2f-flight

