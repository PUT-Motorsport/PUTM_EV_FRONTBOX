################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Src/interfaces/Sc.cpp \
../Core/Src/interfaces/Tensometers.cpp \
../Core/Src/interfaces/accelerometer.cpp \
../Core/Src/interfaces/analog.cpp \
../Core/Src/interfaces/apps.cpp \
../Core/Src/interfaces/brakes.cpp 

OBJS += \
./Core/Src/interfaces/Sc.o \
./Core/Src/interfaces/Tensometers.o \
./Core/Src/interfaces/accelerometer.o \
./Core/Src/interfaces/analog.o \
./Core/Src/interfaces/apps.o \
./Core/Src/interfaces/brakes.o 

CPP_DEPS += \
./Core/Src/interfaces/Sc.d \
./Core/Src/interfaces/Tensometers.d \
./Core/Src/interfaces/accelerometer.d \
./Core/Src/interfaces/analog.d \
./Core/Src/interfaces/apps.d \
./Core/Src/interfaces/brakes.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/interfaces/%.o Core/Src/interfaces/%.su Core/Src/interfaces/%.cyclo: ../Core/Src/interfaces/%.cpp Core/Src/interfaces/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=c++20 -DUSE_HAL_DRIVER -DSTM32G484xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-interfaces

clean-Core-2f-Src-2f-interfaces:
	-$(RM) ./Core/Src/interfaces/Sc.cyclo ./Core/Src/interfaces/Sc.d ./Core/Src/interfaces/Sc.o ./Core/Src/interfaces/Sc.su ./Core/Src/interfaces/Tensometers.cyclo ./Core/Src/interfaces/Tensometers.d ./Core/Src/interfaces/Tensometers.o ./Core/Src/interfaces/Tensometers.su ./Core/Src/interfaces/accelerometer.cyclo ./Core/Src/interfaces/accelerometer.d ./Core/Src/interfaces/accelerometer.o ./Core/Src/interfaces/accelerometer.su ./Core/Src/interfaces/analog.cyclo ./Core/Src/interfaces/analog.d ./Core/Src/interfaces/analog.o ./Core/Src/interfaces/analog.su ./Core/Src/interfaces/apps.cyclo ./Core/Src/interfaces/apps.d ./Core/Src/interfaces/apps.o ./Core/Src/interfaces/apps.su ./Core/Src/interfaces/brakes.cyclo ./Core/Src/interfaces/brakes.d ./Core/Src/interfaces/brakes.o ./Core/Src/interfaces/brakes.su

.PHONY: clean-Core-2f-Src-2f-interfaces

