################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.c \
../Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/system_ARMCM7.c 

OBJS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.o \
./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/system_ARMCM7.o 

C_DEPS += \
./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.d \
./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/system_ARMCM7.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/%.o Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/%.su Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/%.cyclo: ../Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/%.c Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Include -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/modules -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/porting -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx -I/Users/omuryildirim/Downloads/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_class_marks_example-2f-RTE-2f-Device-2f-ARMCM7_SP

clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_class_marks_example-2f-RTE-2f-Device-2f-ARMCM7_SP:
	-$(RM) ./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.cyclo ./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/startup_ARMCM7.su ./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/system_ARMCM7.cyclo ./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/system_ARMCM7.d ./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/system_ARMCM7.o ./Drivers/CMSIS/DSP/Examples/ARM/arm_class_marks_example/RTE/Device/ARMCM7_SP/system_ARMCM7.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Examples-2f-ARM-2f-arm_class_marks_example-2f-RTE-2f-Device-2f-ARMCM7_SP

