################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/ft6x06/ft6x06.c 

OBJS += \
./Drivers/Components/ft6x06/ft6x06.o 

C_DEPS += \
./Drivers/Components/ft6x06/ft6x06.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/ft6x06/%.o Drivers/Components/ft6x06/%.su Drivers/Components/ft6x06/%.cyclo: ../Drivers/Components/ft6x06/%.c Drivers/Components/ft6x06/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/hts221" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lis3mdl" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lsm6dsl" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lps22hb" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/Common" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-ft6x06

clean-Drivers-2f-Components-2f-ft6x06:
	-$(RM) ./Drivers/Components/ft6x06/ft6x06.cyclo ./Drivers/Components/ft6x06/ft6x06.d ./Drivers/Components/ft6x06/ft6x06.o ./Drivers/Components/ft6x06/ft6x06.su

.PHONY: clean-Drivers-2f-Components-2f-ft6x06

