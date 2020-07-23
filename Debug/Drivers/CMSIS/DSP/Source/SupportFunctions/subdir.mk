################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.c \
../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.c 

OBJS += \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.o \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.o 

C_DEPS += \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.d \
./Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_copy_q7.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_f32.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_fill_q7.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_float_to_q7.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_float.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q15_to_q7.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_float.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q31_to_q7.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_float.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q15.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.o: ../Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DARM_MATH_CM4 -DDEBUG '-D__FPU_PRESENT=1' -DSTM32F429xx -c -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/STM32F429I-Discovery" -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/CMSIS/DSP/Include" -I../Drivers/CMSIS/Include -I"C:/Users/User/STM32CubeIDE/Other/prac2_test/Drivers/BSP/Components/ili9341" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/CMSIS/DSP/Source/SupportFunctions/arm_q7_to_q31.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

