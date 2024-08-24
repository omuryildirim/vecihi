################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/app_tof.c \
/Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/app_tof_pin_conf.c \
/Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/custom_ranging_sensor.c \
/Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/main.c \
/Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/stm32f4xx_hal_msp.c \
/Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/stm32f4xx_it.c \
/Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/stm32f4xx_nucleo.c \
/Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/stm32f4xx_nucleo_bus.c \
../Application/User/syscalls.c \
../Application/User/sysmem.c 

OBJS += \
./Application/User/app_tof.o \
./Application/User/app_tof_pin_conf.o \
./Application/User/custom_ranging_sensor.o \
./Application/User/main.o \
./Application/User/stm32f4xx_hal_msp.o \
./Application/User/stm32f4xx_it.o \
./Application/User/stm32f4xx_nucleo.o \
./Application/User/stm32f4xx_nucleo_bus.o \
./Application/User/syscalls.o \
./Application/User/sysmem.o 

C_DEPS += \
./Application/User/app_tof.d \
./Application/User/app_tof_pin_conf.d \
./Application/User/custom_ranging_sensor.d \
./Application/User/main.d \
./Application/User/stm32f4xx_hal_msp.d \
./Application/User/stm32f4xx_it.d \
./Application/User/stm32f4xx_nucleo.d \
./Application/User/stm32f4xx_nucleo_bus.d \
./Application/User/syscalls.d \
./Application/User/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/app_tof.o: /Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/app_tof.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/app_tof_pin_conf.o: /Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/app_tof_pin_conf.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/custom_ranging_sensor.o: /Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/custom_ranging_sensor.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/main.o: /Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/main.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f4xx_hal_msp.o: /Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/stm32f4xx_hal_msp.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f4xx_it.o: /Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/stm32f4xx_it.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f4xx_nucleo.o: /Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/stm32f4xx_nucleo.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/stm32f4xx_nucleo_bus.o: /Users/omuryildirim/dev/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Src/stm32f4xx_nucleo_bus.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Application/User/%.o Application/User/%.su Application/User/%.cyclo: ../Application/User/%.c Application/User/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc -I../../../../../../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../../../../../../../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../../../../../../../Drivers/CMSIS/Include -I../../../../../../../Drivers/BSP/Components/vl53l8cx/modules -I../../../../../../../Drivers/BSP/Components/vl53l8cx/porting -I../../../../../../../Drivers/BSP/Components/vl53l8cx -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Application-2f-User

clean-Application-2f-User:
	-$(RM) ./Application/User/app_tof.cyclo ./Application/User/app_tof.d ./Application/User/app_tof.o ./Application/User/app_tof.su ./Application/User/app_tof_pin_conf.cyclo ./Application/User/app_tof_pin_conf.d ./Application/User/app_tof_pin_conf.o ./Application/User/app_tof_pin_conf.su ./Application/User/custom_ranging_sensor.cyclo ./Application/User/custom_ranging_sensor.d ./Application/User/custom_ranging_sensor.o ./Application/User/custom_ranging_sensor.su ./Application/User/main.cyclo ./Application/User/main.d ./Application/User/main.o ./Application/User/main.su ./Application/User/stm32f4xx_hal_msp.cyclo ./Application/User/stm32f4xx_hal_msp.d ./Application/User/stm32f4xx_hal_msp.o ./Application/User/stm32f4xx_hal_msp.su ./Application/User/stm32f4xx_it.cyclo ./Application/User/stm32f4xx_it.d ./Application/User/stm32f4xx_it.o ./Application/User/stm32f4xx_it.su ./Application/User/stm32f4xx_nucleo.cyclo ./Application/User/stm32f4xx_nucleo.d ./Application/User/stm32f4xx_nucleo.o ./Application/User/stm32f4xx_nucleo.su ./Application/User/stm32f4xx_nucleo_bus.cyclo ./Application/User/stm32f4xx_nucleo_bus.d ./Application/User/stm32f4xx_nucleo_bus.o ./Application/User/stm32f4xx_nucleo_bus.su ./Application/User/syscalls.cyclo ./Application/User/syscalls.d ./Application/User/syscalls.o ./Application/User/syscalls.su ./Application/User/sysmem.cyclo ./Application/User/sysmem.d ./Application/User/sysmem.o ./Application/User/sysmem.su

.PHONY: clean-Application-2f-User

