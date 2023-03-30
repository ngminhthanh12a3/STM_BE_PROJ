################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CRC16.c \
../Core/Src/FRAME_PARSE.c \
../Core/Src/dev_act_hdl.c \
../Core/Src/main.c \
../Core/Src/max30102.c \
../Core/Src/spo2_algorithm.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f1xx.c \
../Core/Src/uart_imp.c \
../Core/Src/uart_print.c 

OBJS += \
./Core/Src/CRC16.o \
./Core/Src/FRAME_PARSE.o \
./Core/Src/dev_act_hdl.o \
./Core/Src/main.o \
./Core/Src/max30102.o \
./Core/Src/spo2_algorithm.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f1xx.o \
./Core/Src/uart_imp.o \
./Core/Src/uart_print.o 

C_DEPS += \
./Core/Src/CRC16.d \
./Core/Src/FRAME_PARSE.d \
./Core/Src/dev_act_hdl.d \
./Core/Src/main.d \
./Core/Src/max30102.d \
./Core/Src/spo2_algorithm.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f1xx.d \
./Core/Src/uart_imp.d \
./Core/Src/uart_print.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xE -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CRC16.d ./Core/Src/CRC16.o ./Core/Src/CRC16.su ./Core/Src/FRAME_PARSE.d ./Core/Src/FRAME_PARSE.o ./Core/Src/FRAME_PARSE.su ./Core/Src/dev_act_hdl.d ./Core/Src/dev_act_hdl.o ./Core/Src/dev_act_hdl.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/max30102.d ./Core/Src/max30102.o ./Core/Src/max30102.su ./Core/Src/spo2_algorithm.d ./Core/Src/spo2_algorithm.o ./Core/Src/spo2_algorithm.su ./Core/Src/stm32f1xx_hal_msp.d ./Core/Src/stm32f1xx_hal_msp.o ./Core/Src/stm32f1xx_hal_msp.su ./Core/Src/stm32f1xx_it.d ./Core/Src/stm32f1xx_it.o ./Core/Src/stm32f1xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f1xx.d ./Core/Src/system_stm32f1xx.o ./Core/Src/system_stm32f1xx.su ./Core/Src/uart_imp.d ./Core/Src/uart_imp.o ./Core/Src/uart_imp.su ./Core/Src/uart_print.d ./Core/Src/uart_print.o ./Core/Src/uart_print.su

.PHONY: clean-Core-2f-Src

