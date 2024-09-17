################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/vl53l4cd/vl53l4cd.c 

OBJS += \
./Drivers/BSP/Components/vl53l4cd/vl53l4cd.o 

C_DEPS += \
./Drivers/BSP/Components/vl53l4cd/vl53l4cd.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/vl53l4cd/%.o Drivers/BSP/Components/vl53l4cd/%.su Drivers/BSP/Components/vl53l4cd/%.cyclo: ../Drivers/BSP/Components/vl53l4cd/%.c Drivers/BSP/Components/vl53l4cd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Include -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/modules -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/porting -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx -I/Users/omuryildirim/Downloads/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-vl53l4cd

clean-Drivers-2f-BSP-2f-Components-2f-vl53l4cd:
	-$(RM) ./Drivers/BSP/Components/vl53l4cd/vl53l4cd.cyclo ./Drivers/BSP/Components/vl53l4cd/vl53l4cd.d ./Drivers/BSP/Components/vl53l4cd/vl53l4cd.o ./Drivers/BSP/Components/vl53l4cd/vl53l4cd.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-vl53l4cd

