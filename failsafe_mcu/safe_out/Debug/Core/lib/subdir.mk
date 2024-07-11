################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lib/ibus.c \
../Core/lib/mavlink_handler.c \
../Core/lib/pwm.c \
../Core/lib/timer.c 

OBJS += \
./Core/lib/ibus.o \
./Core/lib/mavlink_handler.o \
./Core/lib/pwm.o \
./Core/lib/timer.o 

C_DEPS += \
./Core/lib/ibus.d \
./Core/lib/mavlink_handler.d \
./Core/lib/pwm.d \
./Core/lib/timer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lib/%.o: ../Core/lib/%.c Core/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tuan/Documents/stm/safe_out/Core/mavlink" -I"C:/Users/tuan/Documents/stm/safe_out/Core/lib" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-lib

clean-Core-2f-lib:
	-$(RM) ./Core/lib/ibus.d ./Core/lib/ibus.o ./Core/lib/mavlink_handler.d ./Core/lib/mavlink_handler.o ./Core/lib/pwm.d ./Core/lib/pwm.o ./Core/lib/timer.d ./Core/lib/timer.o

.PHONY: clean-Core-2f-lib

