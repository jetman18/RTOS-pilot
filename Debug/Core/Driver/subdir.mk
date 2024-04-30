################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Driver/bmp280.c \
../Core/Driver/hmc5883.c \
../Core/Driver/ibus.c \
../Core/Driver/mpu6050.c \
../Core/Driver/ms5611.c \
../Core/Driver/qmc5883.c 

OBJS += \
./Core/Driver/bmp280.o \
./Core/Driver/hmc5883.o \
./Core/Driver/ibus.o \
./Core/Driver/mpu6050.o \
./Core/Driver/ms5611.o \
./Core/Driver/qmc5883.o 

C_DEPS += \
./Core/Driver/bmp280.d \
./Core/Driver/hmc5883.d \
./Core/Driver/ibus.d \
./Core/Driver/mpu6050.d \
./Core/Driver/ms5611.d \
./Core/Driver/qmc5883.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Driver/%.o Core/Driver/%.su Core/Driver/%.cyclo: ../Core/Driver/%.c Core/Driver/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../Middlewares/Third_Party/FatFs/src -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/flight" -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/mavlink" -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/simulation" -I../Middlewares/Third_Party/FatFs/src/drivers -I"C:/Users/tuan/Documents/Stmproject/RTOSpilot/Core/epprom" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Driver

clean-Core-2f-Driver:
	-$(RM) ./Core/Driver/bmp280.cyclo ./Core/Driver/bmp280.d ./Core/Driver/bmp280.o ./Core/Driver/bmp280.su ./Core/Driver/hmc5883.cyclo ./Core/Driver/hmc5883.d ./Core/Driver/hmc5883.o ./Core/Driver/hmc5883.su ./Core/Driver/ibus.cyclo ./Core/Driver/ibus.d ./Core/Driver/ibus.o ./Core/Driver/ibus.su ./Core/Driver/mpu6050.cyclo ./Core/Driver/mpu6050.d ./Core/Driver/mpu6050.o ./Core/Driver/mpu6050.su ./Core/Driver/ms5611.cyclo ./Core/Driver/ms5611.d ./Core/Driver/ms5611.o ./Core/Driver/ms5611.su ./Core/Driver/qmc5883.cyclo ./Core/Driver/qmc5883.d ./Core/Driver/qmc5883.o ./Core/Driver/qmc5883.su

.PHONY: clean-Core-2f-Driver

