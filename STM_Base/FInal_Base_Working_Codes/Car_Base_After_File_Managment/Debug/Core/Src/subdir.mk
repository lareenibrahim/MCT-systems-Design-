################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Base_Manual.c \
../Core/Src/Base_Position.c \
../Core/Src/Hiwonder_Motor.c \
../Core/Src/bmp180.c \
../Core/Src/chassis_control.c \
../Core/Src/comm_parser.c \
../Core/Src/imu_fusion.c \
../Core/Src/mag_sensor.c \
../Core/Src/main.c \
../Core/Src/mpu6050.c \
../Core/Src/odometry.c \
../Core/Src/sensor_sys.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/telemetry.c 

OBJS += \
./Core/Src/Base_Manual.o \
./Core/Src/Base_Position.o \
./Core/Src/Hiwonder_Motor.o \
./Core/Src/bmp180.o \
./Core/Src/chassis_control.o \
./Core/Src/comm_parser.o \
./Core/Src/imu_fusion.o \
./Core/Src/mag_sensor.o \
./Core/Src/main.o \
./Core/Src/mpu6050.o \
./Core/Src/odometry.o \
./Core/Src/sensor_sys.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/telemetry.o 

C_DEPS += \
./Core/Src/Base_Manual.d \
./Core/Src/Base_Position.d \
./Core/Src/Hiwonder_Motor.d \
./Core/Src/bmp180.d \
./Core/Src/chassis_control.d \
./Core/Src/comm_parser.d \
./Core/Src/imu_fusion.d \
./Core/Src/mag_sensor.d \
./Core/Src/main.d \
./Core/Src/mpu6050.d \
./Core/Src/odometry.d \
./Core/Src/sensor_sys.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/telemetry.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xC -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Base_Manual.cyclo ./Core/Src/Base_Manual.d ./Core/Src/Base_Manual.o ./Core/Src/Base_Manual.su ./Core/Src/Base_Position.cyclo ./Core/Src/Base_Position.d ./Core/Src/Base_Position.o ./Core/Src/Base_Position.su ./Core/Src/Hiwonder_Motor.cyclo ./Core/Src/Hiwonder_Motor.d ./Core/Src/Hiwonder_Motor.o ./Core/Src/Hiwonder_Motor.su ./Core/Src/bmp180.cyclo ./Core/Src/bmp180.d ./Core/Src/bmp180.o ./Core/Src/bmp180.su ./Core/Src/chassis_control.cyclo ./Core/Src/chassis_control.d ./Core/Src/chassis_control.o ./Core/Src/chassis_control.su ./Core/Src/comm_parser.cyclo ./Core/Src/comm_parser.d ./Core/Src/comm_parser.o ./Core/Src/comm_parser.su ./Core/Src/imu_fusion.cyclo ./Core/Src/imu_fusion.d ./Core/Src/imu_fusion.o ./Core/Src/imu_fusion.su ./Core/Src/mag_sensor.cyclo ./Core/Src/mag_sensor.d ./Core/Src/mag_sensor.o ./Core/Src/mag_sensor.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mpu6050.cyclo ./Core/Src/mpu6050.d ./Core/Src/mpu6050.o ./Core/Src/mpu6050.su ./Core/Src/odometry.cyclo ./Core/Src/odometry.d ./Core/Src/odometry.o ./Core/Src/odometry.su ./Core/Src/sensor_sys.cyclo ./Core/Src/sensor_sys.d ./Core/Src/sensor_sys.o ./Core/Src/sensor_sys.su ./Core/Src/stm32f4xx_hal_msp.cyclo ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_it.cyclo ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.cyclo ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/telemetry.cyclo ./Core/Src/telemetry.d ./Core/Src/telemetry.o ./Core/Src/telemetry.su

.PHONY: clean-Core-2f-Src

