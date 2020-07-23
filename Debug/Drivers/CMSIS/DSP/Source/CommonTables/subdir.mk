################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c \
../Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.c 

OBJS += \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.o \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.o 

C_DEPS += \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.d \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.o: ../Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.o: ../Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

