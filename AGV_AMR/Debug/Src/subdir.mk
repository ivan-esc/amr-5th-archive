################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/amr_mode.c \
../Src/bluetooth.c \
../Src/encoders.c \
../Src/esp32_com.c \
../Src/i2c_color_imu.c \
../Src/line_follow.c \
../Src/main.c \
../Src/motor_pwm.c \
../Src/odometry.c \
../Src/pid_diff_drive.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/ultrasonic.c 

OBJS += \
./Src/amr_mode.o \
./Src/bluetooth.o \
./Src/encoders.o \
./Src/esp32_com.o \
./Src/i2c_color_imu.o \
./Src/line_follow.o \
./Src/main.o \
./Src/motor_pwm.o \
./Src/odometry.o \
./Src/pid_diff_drive.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/ultrasonic.o 

C_DEPS += \
./Src/amr_mode.d \
./Src/bluetooth.d \
./Src/encoders.d \
./Src/esp32_com.d \
./Src/i2c_color_imu.d \
./Src/line_follow.d \
./Src/main.d \
./Src/motor_pwm.d \
./Src/odometry.d \
./Src/pid_diff_drive.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/ultrasonic.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F0 -DSTM32F051R8Tx -c -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/amr_mode.cyclo ./Src/amr_mode.d ./Src/amr_mode.o ./Src/amr_mode.su ./Src/bluetooth.cyclo ./Src/bluetooth.d ./Src/bluetooth.o ./Src/bluetooth.su ./Src/encoders.cyclo ./Src/encoders.d ./Src/encoders.o ./Src/encoders.su ./Src/esp32_com.cyclo ./Src/esp32_com.d ./Src/esp32_com.o ./Src/esp32_com.su ./Src/i2c_color_imu.cyclo ./Src/i2c_color_imu.d ./Src/i2c_color_imu.o ./Src/i2c_color_imu.su ./Src/line_follow.cyclo ./Src/line_follow.d ./Src/line_follow.o ./Src/line_follow.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/motor_pwm.cyclo ./Src/motor_pwm.d ./Src/motor_pwm.o ./Src/motor_pwm.su ./Src/odometry.cyclo ./Src/odometry.d ./Src/odometry.o ./Src/odometry.su ./Src/pid_diff_drive.cyclo ./Src/pid_diff_drive.d ./Src/pid_diff_drive.o ./Src/pid_diff_drive.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/ultrasonic.cyclo ./Src/ultrasonic.d ./Src/ultrasonic.o ./Src/ultrasonic.su

.PHONY: clean-Src

