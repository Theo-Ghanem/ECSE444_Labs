################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/st25dv/st25dv.c \
../Drivers/Components/st25dv/st25dv_reg.c 

OBJS += \
./Drivers/Components/st25dv/st25dv.o \
./Drivers/Components/st25dv/st25dv_reg.o 

C_DEPS += \
./Drivers/Components/st25dv/st25dv.d \
./Drivers/Components/st25dv/st25dv_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/st25dv/%.o Drivers/Components/st25dv/%.su Drivers/Components/st25dv/%.cyclo: ../Drivers/Components/st25dv/%.c Drivers/Components/st25dv/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/hts221" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lis3mdl" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lsm6dsl" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lps22hb" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/Common" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-st25dv

clean-Drivers-2f-Components-2f-st25dv:
	-$(RM) ./Drivers/Components/st25dv/st25dv.cyclo ./Drivers/Components/st25dv/st25dv.d ./Drivers/Components/st25dv/st25dv.o ./Drivers/Components/st25dv/st25dv.su ./Drivers/Components/st25dv/st25dv_reg.cyclo ./Drivers/Components/st25dv/st25dv_reg.d ./Drivers/Components/st25dv/st25dv_reg.o ./Drivers/Components/st25dv/st25dv_reg.su

.PHONY: clean-Drivers-2f-Components-2f-st25dv
