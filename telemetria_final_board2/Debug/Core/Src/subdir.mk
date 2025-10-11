################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GPS.c \
../Core/Src/IMU.c \
../Core/Src/PWM_sensors.c \
../Core/Src/eMMC.c \
../Core/Src/main.c \
../Core/Src/rfm95.c \
../Core/Src/sensor.c \
../Core/Src/sensors.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c 

OBJS += \
./Core/Src/GPS.o \
./Core/Src/IMU.o \
./Core/Src/PWM_sensors.o \
./Core/Src/eMMC.o \
./Core/Src/main.o \
./Core/Src/rfm95.o \
./Core/Src/sensor.o \
./Core/Src/sensors.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Core/Src/GPS.d \
./Core/Src/IMU.d \
./Core/Src/PWM_sensors.d \
./Core/Src/eMMC.d \
./Core/Src/main.d \
./Core/Src/rfm95.d \
./Core/Src/sensor.d \
./Core/Src/sensors.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I./CANopenNode -I./CANopenNode_STM32 -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../CANopenNode -I../CANopenNode_STM32 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/GPS.cyclo ./Core/Src/GPS.d ./Core/Src/GPS.o ./Core/Src/GPS.su ./Core/Src/IMU.cyclo ./Core/Src/IMU.d ./Core/Src/IMU.o ./Core/Src/IMU.su ./Core/Src/PWM_sensors.cyclo ./Core/Src/PWM_sensors.d ./Core/Src/PWM_sensors.o ./Core/Src/PWM_sensors.su ./Core/Src/eMMC.cyclo ./Core/Src/eMMC.d ./Core/Src/eMMC.o ./Core/Src/eMMC.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/rfm95.cyclo ./Core/Src/rfm95.d ./Core/Src/rfm95.o ./Core/Src/rfm95.su ./Core/Src/sensor.cyclo ./Core/Src/sensor.d ./Core/Src/sensor.o ./Core/Src/sensor.su ./Core/Src/sensors.cyclo ./Core/Src/sensors.d ./Core/Src/sensors.o ./Core/Src/sensors.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su

.PHONY: clean-Core-2f-Src

