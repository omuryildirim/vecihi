################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32412G-Discovery/stm32412g_discovery.c \
../Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_audio.c \
../Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_eeprom.c \
../Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_lcd.c \
../Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_qspi.c \
../Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_sd.c \
../Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_ts.c 

OBJS += \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery.o \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_audio.o \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_eeprom.o \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_lcd.o \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_qspi.o \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_sd.o \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_ts.o 

C_DEPS += \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery.d \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_audio.d \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_eeprom.d \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_lcd.d \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_qspi.d \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_sd.d \
./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_ts.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32412G-Discovery/%.o Drivers/BSP/STM32412G-Discovery/%.su Drivers/BSP/STM32412G-Discovery/%.cyclo: ../Drivers/BSP/STM32412G-Discovery/%.c Drivers/BSP/STM32412G-Discovery/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Include -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/modules -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/porting -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx -I/Users/omuryildirim/Downloads/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32412G-2d-Discovery

clean-Drivers-2f-BSP-2f-STM32412G-2d-Discovery:
	-$(RM) ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery.cyclo ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery.d ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery.o ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery.su ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_audio.cyclo ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_audio.d ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_audio.o ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_audio.su ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_eeprom.cyclo ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_eeprom.d ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_eeprom.o ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_eeprom.su ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_lcd.cyclo ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_lcd.d ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_lcd.o ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_lcd.su ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_qspi.cyclo ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_qspi.d ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_qspi.o ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_qspi.su ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_sd.cyclo ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_sd.d ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_sd.o ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_sd.su ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_ts.cyclo ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_ts.d ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_ts.o ./Drivers/BSP/STM32412G-Discovery/stm32412g_discovery_ts.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32412G-2d-Discovery

