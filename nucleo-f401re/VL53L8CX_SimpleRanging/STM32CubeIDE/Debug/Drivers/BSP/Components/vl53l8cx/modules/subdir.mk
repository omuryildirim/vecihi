################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.c \
../Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.c \
../Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.c \
../Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.c 

OBJS += \
./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.o \
./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.o \
./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.o \
./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.o 

C_DEPS += \
./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.d \
./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.d \
./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.d \
./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/vl53l8cx/modules/%.o Drivers/BSP/Components/vl53l8cx/modules/%.su Drivers/BSP/Components/vl53l8cx/modules/%.cyclo: ../Drivers/BSP/Components/vl53l8cx/modules/%.c Drivers/BSP/Components/vl53l8cx/modules/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Include -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/modules -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/porting -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx -I/Users/omuryildirim/Downloads/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-vl53l8cx-2f-modules

clean-Drivers-2f-BSP-2f-Components-2f-vl53l8cx-2f-modules:
	-$(RM) ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.cyclo ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.d ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.o ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_api.su ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.cyclo ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.d ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.o ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_detection_thresholds.su ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.cyclo ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.d ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.o ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_motion_indicator.su ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.cyclo ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.d ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.o ./Drivers/BSP/Components/vl53l8cx/modules/vl53l8cx_plugin_xtalk.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-vl53l8cx-2f-modules

