################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Encoder.c \
../Core/Src/Kinematics_Mecanum.c \
../Core/Src/MotorControl.c \
../Core/Src/NRF24L01.c \
../Core/Src/error.c \
../Core/Src/fonts.c \
../Core/Src/loop.c \
../Core/Src/main.c \
../Core/Src/mpu.c \
../Core/Src/pid.c \
../Core/Src/servo.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/Encoder.o \
./Core/Src/Kinematics_Mecanum.o \
./Core/Src/MotorControl.o \
./Core/Src/NRF24L01.o \
./Core/Src/error.o \
./Core/Src/fonts.o \
./Core/Src/loop.o \
./Core/Src/main.o \
./Core/Src/mpu.o \
./Core/Src/pid.o \
./Core/Src/servo.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/Encoder.d \
./Core/Src/Kinematics_Mecanum.d \
./Core/Src/MotorControl.d \
./Core/Src/NRF24L01.d \
./Core/Src/error.d \
./Core/Src/fonts.d \
./Core/Src/loop.d \
./Core/Src/main.d \
./Core/Src/mpu.d \
./Core/Src/pid.d \
./Core/Src/servo.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Encoder.d ./Core/Src/Encoder.o ./Core/Src/Kinematics_Mecanum.d ./Core/Src/Kinematics_Mecanum.o ./Core/Src/MotorControl.d ./Core/Src/MotorControl.o ./Core/Src/NRF24L01.d ./Core/Src/NRF24L01.o ./Core/Src/error.d ./Core/Src/error.o ./Core/Src/fonts.d ./Core/Src/fonts.o ./Core/Src/loop.d ./Core/Src/loop.o ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/mpu.d ./Core/Src/mpu.o ./Core/Src/pid.d ./Core/Src/pid.o ./Core/Src/servo.d ./Core/Src/servo.o ./Core/Src/ssd1306.d ./Core/Src/ssd1306.o ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o

.PHONY: clean-Core-2f-Src

