################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/epprom/AT24Cxx_stm32_hal.c 

OBJS += \
./Core/epprom/AT24Cxx_stm32_hal.o 

C_DEPS += \
./Core/epprom/AT24Cxx_stm32_hal.d 


# Each subdirectory must supply rules for building sources it contributes
Core/epprom/%.o Core/epprom/%.su Core/epprom/%.cyclo: ../Core/epprom/%.c Core/epprom/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/flight" -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/mavlink" -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/simulation" -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/epprom" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FatFs/src/drivers -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-epprom

clean-Core-2f-epprom:
	-$(RM) ./Core/epprom/AT24Cxx_stm32_hal.cyclo ./Core/epprom/AT24Cxx_stm32_hal.d ./Core/epprom/AT24Cxx_stm32_hal.o ./Core/epprom/AT24Cxx_stm32_hal.su

.PHONY: clean-Core-2f-epprom

