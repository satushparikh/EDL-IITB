################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/DC_MOTOR.c \
../Core/Inc/DC_MOTOR_cfg.c \
../Core/Inc/motor_encoders.c 

OBJS += \
./Core/Inc/DC_MOTOR.o \
./Core/Inc/DC_MOTOR_cfg.o \
./Core/Inc/motor_encoders.o 

C_DEPS += \
./Core/Inc/DC_MOTOR.d \
./Core/Inc/DC_MOTOR_cfg.d \
./Core/Inc/motor_encoders.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/DC_MOTOR.cyclo ./Core/Inc/DC_MOTOR.d ./Core/Inc/DC_MOTOR.o ./Core/Inc/DC_MOTOR.su ./Core/Inc/DC_MOTOR_cfg.cyclo ./Core/Inc/DC_MOTOR_cfg.d ./Core/Inc/DC_MOTOR_cfg.o ./Core/Inc/DC_MOTOR_cfg.su ./Core/Inc/motor_encoders.cyclo ./Core/Inc/motor_encoders.d ./Core/Inc/motor_encoders.o ./Core/Inc/motor_encoders.su

.PHONY: clean-Core-2f-Inc

