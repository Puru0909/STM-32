################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LIBRARY/libmotor.cpp \
../LIBRARY/main.cpp 

OBJS += \
./LIBRARY/libmotor.o \
./LIBRARY/main.o 

CPP_DEPS += \
./LIBRARY/libmotor.d \
./LIBRARY/main.d 


# Each subdirectory must supply rules for building sources it contributes
LIBRARY/%.o LIBRARY/%.su LIBRARY/%.cyclo: ../LIBRARY/%.cpp LIBRARY/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-LIBRARY

clean-LIBRARY:
	-$(RM) ./LIBRARY/libmotor.cyclo ./LIBRARY/libmotor.d ./LIBRARY/libmotor.o ./LIBRARY/libmotor.su ./LIBRARY/main.cyclo ./LIBRARY/main.d ./LIBRARY/main.o ./LIBRARY/main.su

.PHONY: clean-LIBRARY

