################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/hx8347g/hx8347g.c 

OBJS += \
./Drivers/Components/hx8347g/hx8347g.o 

C_DEPS += \
./Drivers/Components/hx8347g/hx8347g.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/hx8347g/%.o Drivers/Components/hx8347g/%.su Drivers/Components/hx8347g/%.cyclo: ../Drivers/Components/hx8347g/%.c Drivers/Components/hx8347g/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/theog/Documents/CodingProjects/ECSE444_Labs/Lab 4/Drivers/Components" -I"C:/Users/theog/Documents/CodingProjects/ECSE444_Labs/Lab 4/Drivers/Components/hts221" -I"C:/Users/theog/Documents/CodingProjects/ECSE444_Labs/Lab 4/Drivers/Components/lis3mdl" -I"C:/Users/theog/Documents/CodingProjects/ECSE444_Labs/Lab 4/Drivers/Components/lsm6dsl" -I"C:/Users/theog/Documents/CodingProjects/ECSE444_Labs/Lab 4/Drivers/Components/lps22hb" -I"C:/Users/theog/Documents/CodingProjects/ECSE444_Labs/Lab 4/Drivers/Components/Common" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-hx8347g

clean-Drivers-2f-Components-2f-hx8347g:
	-$(RM) ./Drivers/Components/hx8347g/hx8347g.cyclo ./Drivers/Components/hx8347g/hx8347g.d ./Drivers/Components/hx8347g/hx8347g.o ./Drivers/Components/hx8347g/hx8347g.su

.PHONY: clean-Drivers-2f-Components-2f-hx8347g

