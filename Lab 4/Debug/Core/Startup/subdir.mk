################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32l4s5vitx.s 

OBJS += \
./Core/Startup/startup_stm32l4s5vitx.o 

S_DEPS += \
./Core/Startup/startup_stm32l4s5vitx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/hts221" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lis3mdl" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lsm6dsl" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/lps22hb" -I"C:/Users/Philippe/STM32CubeIDE/workspace_1.13.1/ECSE444_Labs/Lab 4/Drivers/Components/Common" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32l4s5vitx.d ./Core/Startup/startup_stm32l4s5vitx.o

.PHONY: clean-Core-2f-Startup

