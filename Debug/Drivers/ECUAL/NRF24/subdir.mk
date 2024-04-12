################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ECUAL/NRF24/NRF.c 

OBJS += \
./Drivers/ECUAL/NRF24/NRF.o 

C_DEPS += \
./Drivers/ECUAL/NRF24/NRF.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ECUAL/NRF24/%.o Drivers/ECUAL/NRF24/%.su Drivers/ECUAL/NRF24/%.cyclo: ../Drivers/ECUAL/NRF24/%.c Drivers/ECUAL/NRF24/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"F:/College/lvl 400 second/Graduation Project/NRF/NRF_Module/Drivers/ECUAL/NRF24" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-ECUAL-2f-NRF24

clean-Drivers-2f-ECUAL-2f-NRF24:
	-$(RM) ./Drivers/ECUAL/NRF24/NRF.cyclo ./Drivers/ECUAL/NRF24/NRF.d ./Drivers/ECUAL/NRF24/NRF.o ./Drivers/ECUAL/NRF24/NRF.su

.PHONY: clean-Drivers-2f-ECUAL-2f-NRF24

