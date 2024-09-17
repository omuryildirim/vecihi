################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.c \
../Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.c \
../Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c \
../Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables_f16.c \
../Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.c \
../Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs_f16.c \
../Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables.c \
../Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables_f16.c 

OBJS += \
./Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.o \
./Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.o \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.o \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables_f16.o \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.o \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs_f16.o \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables.o \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables_f16.o 

C_DEPS += \
./Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.d \
./Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.d \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.d \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables_f16.d \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.d \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs_f16.d \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables.d \
./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables_f16.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/DSP/Source/CommonTables/%.o Drivers/CMSIS/DSP/Source/CommonTables/%.su Drivers/CMSIS/DSP/Source/CommonTables/%.cyclo: ../Drivers/CMSIS/DSP/Source/CommonTables/%.c Drivers/CMSIS/DSP/Source/CommonTables/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../../Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/Users/omuryildirim/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.0/Drivers/CMSIS/Include -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/modules -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx/porting -I/Users/omuryildirim/STM32Cube/Repository//Packs/STMicroelectronics/X-CUBE-TOF1/3.4.2/Drivers/BSP/Components/vl53l8cx -I/Users/omuryildirim/Downloads/en.X-CUBE-TOF1/Projects/NUCLEO-F401RE/Examples/CUSTOM/VL53L8CX_SimpleRanging/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-CommonTables

clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-CommonTables:
	-$(RM) ./Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.cyclo ./Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.d ./Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.o ./Drivers/CMSIS/DSP/Source/CommonTables/CommonTables.su ./Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.cyclo ./Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.d ./Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.o ./Drivers/CMSIS/DSP/Source/CommonTables/CommonTablesF16.su ./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.cyclo ./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.d ./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.o ./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.su ./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables_f16.cyclo ./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables_f16.d ./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables_f16.o ./Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables_f16.su ./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.cyclo ./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.d ./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.o ./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs.su ./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs_f16.cyclo ./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs_f16.d ./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs_f16.o ./Drivers/CMSIS/DSP/Source/CommonTables/arm_const_structs_f16.su ./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables.cyclo ./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables.d ./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables.o ./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables.su ./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables_f16.cyclo ./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables_f16.d ./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables_f16.o ./Drivers/CMSIS/DSP/Source/CommonTables/arm_mve_tables_f16.su

.PHONY: clean-Drivers-2f-CMSIS-2f-DSP-2f-Source-2f-CommonTables

