################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../can/lib/can1.c \
../can/lib/can2.c \
../can/lib/flash.c 

OBJS += \
./can/lib/can1.o \
./can/lib/can2.o \
./can/lib/flash.o 

C_DEPS += \
./can/lib/can1.d \
./can/lib/can2.d \
./can/lib/flash.d 


# Each subdirectory must supply rules for building sources it contributes
can/lib/%.o can/lib/%.su can/lib/%.cyclo: ../can/lib/%.c can/lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"C:/Users/Jordi/STM32CubeIDE/workspace_1.19.0/AMS-SW/can/lib" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-can-2f-lib

clean-can-2f-lib:
	-$(RM) ./can/lib/can1.cyclo ./can/lib/can1.d ./can/lib/can1.o ./can/lib/can1.su ./can/lib/can2.cyclo ./can/lib/can2.d ./can/lib/can2.o ./can/lib/can2.su ./can/lib/flash.cyclo ./can/lib/flash.d ./can/lib/flash.o ./can/lib/flash.su

.PHONY: clean-can-2f-lib

