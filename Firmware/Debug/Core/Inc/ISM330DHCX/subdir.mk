################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/ISM330DHCX/ism330dhcx_reg.c 

C_DEPS += \
./Core/Inc/ISM330DHCX/ism330dhcx_reg.d 

OBJS += \
./Core/Inc/ISM330DHCX/ism330dhcx_reg.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/ISM330DHCX/%.o Core/Inc/ISM330DHCX/%.su Core/Inc/ISM330DHCX/%.cyclo: ../Core/Inc/ISM330DHCX/%.c Core/Inc/ISM330DHCX/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G473xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc-2f-ISM330DHCX

clean-Core-2f-Inc-2f-ISM330DHCX:
	-$(RM) ./Core/Inc/ISM330DHCX/ism330dhcx_reg.cyclo ./Core/Inc/ISM330DHCX/ism330dhcx_reg.d ./Core/Inc/ISM330DHCX/ism330dhcx_reg.o ./Core/Inc/ISM330DHCX/ism330dhcx_reg.su

.PHONY: clean-Core-2f-Inc-2f-ISM330DHCX

