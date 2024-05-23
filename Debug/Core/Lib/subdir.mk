################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/baro.c \
../Core/Lib/blackbox.c \
../Core/Lib/compass.c \
../Core/Lib/gps.c \
../Core/Lib/imu.c \
../Core/Lib/maths.c \
../Core/Lib/pid.c \
../Core/Lib/ppmreceiver.c \
../Core/Lib/pwm.c \
../Core/Lib/sensordetect.c \
../Core/Lib/timer.c 

OBJS += \
./Core/Lib/baro.o \
./Core/Lib/blackbox.o \
./Core/Lib/compass.o \
./Core/Lib/gps.o \
./Core/Lib/imu.o \
./Core/Lib/maths.o \
./Core/Lib/pid.o \
./Core/Lib/ppmreceiver.o \
./Core/Lib/pwm.o \
./Core/Lib/sensordetect.o \
./Core/Lib/timer.o 

C_DEPS += \
./Core/Lib/baro.d \
./Core/Lib/blackbox.d \
./Core/Lib/compass.d \
./Core/Lib/gps.d \
./Core/Lib/imu.d \
./Core/Lib/maths.d \
./Core/Lib/pid.d \
./Core/Lib/ppmreceiver.d \
./Core/Lib/pwm.d \
./Core/Lib/sensordetect.d \
./Core/Lib/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/%.o Core/Lib/%.su Core/Lib/%.cyclo: ../Core/Lib/%.c Core/Lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/tuan/Documents/Stmproject/RTOS-pilot/Core/Driver" -I"C:/Users/tuan/Documents/Stmproject/RTOS-pilot/Core/epprom" -I"C:/Users/tuan/Documents/Stmproject/RTOS-pilot/Core/flight" -I"C:/Users/tuan/Documents/Stmproject/RTOS-pilot/Core/Lib" -I"C:/Users/tuan/Documents/Stmproject/RTOS-pilot/Core/mavlink" -I"C:/Users/tuan/Documents/Stmproject/RTOS-pilot/Core/simulation" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib

clean-Core-2f-Lib:
	-$(RM) ./Core/Lib/baro.cyclo ./Core/Lib/baro.d ./Core/Lib/baro.o ./Core/Lib/baro.su ./Core/Lib/blackbox.cyclo ./Core/Lib/blackbox.d ./Core/Lib/blackbox.o ./Core/Lib/blackbox.su ./Core/Lib/compass.cyclo ./Core/Lib/compass.d ./Core/Lib/compass.o ./Core/Lib/compass.su ./Core/Lib/gps.cyclo ./Core/Lib/gps.d ./Core/Lib/gps.o ./Core/Lib/gps.su ./Core/Lib/imu.cyclo ./Core/Lib/imu.d ./Core/Lib/imu.o ./Core/Lib/imu.su ./Core/Lib/maths.cyclo ./Core/Lib/maths.d ./Core/Lib/maths.o ./Core/Lib/maths.su ./Core/Lib/pid.cyclo ./Core/Lib/pid.d ./Core/Lib/pid.o ./Core/Lib/pid.su ./Core/Lib/ppmreceiver.cyclo ./Core/Lib/ppmreceiver.d ./Core/Lib/ppmreceiver.o ./Core/Lib/ppmreceiver.su ./Core/Lib/pwm.cyclo ./Core/Lib/pwm.d ./Core/Lib/pwm.o ./Core/Lib/pwm.su ./Core/Lib/sensordetect.cyclo ./Core/Lib/sensordetect.d ./Core/Lib/sensordetect.o ./Core/Lib/sensordetect.su ./Core/Lib/timer.cyclo ./Core/Lib/timer.d ./Core/Lib/timer.o ./Core/Lib/timer.su

.PHONY: clean-Core-2f-Lib

