################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Lib/Src/delay.c \
../Core/Lib/Src/fonts.c \
../Core/Lib/Src/oled.c \
../Core/Lib/Src/spi.c \
../Core/Lib/Src/test.c 

OBJS += \
./Core/Lib/Src/delay.o \
./Core/Lib/Src/fonts.o \
./Core/Lib/Src/oled.o \
./Core/Lib/Src/spi.o \
./Core/Lib/Src/test.o 

C_DEPS += \
./Core/Lib/Src/delay.d \
./Core/Lib/Src/fonts.d \
./Core/Lib/Src/oled.d \
./Core/Lib/Src/spi.d \
./Core/Lib/Src/test.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Lib/Src/%.o Core/Lib/Src/%.su Core/Lib/Src/%.cyclo: ../Core/Lib/Src/%.c Core/Lib/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/SAMSUNG/STM32CubeIDE/SSD1306OLED_SPI/Inc" -I"C:/Users/SAMSUNG/STM32CubeIDE/SSD1306OLED_SPI/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Lib-2f-Src

clean-Core-2f-Lib-2f-Src:
	-$(RM) ./Core/Lib/Src/delay.cyclo ./Core/Lib/Src/delay.d ./Core/Lib/Src/delay.o ./Core/Lib/Src/delay.su ./Core/Lib/Src/fonts.cyclo ./Core/Lib/Src/fonts.d ./Core/Lib/Src/fonts.o ./Core/Lib/Src/fonts.su ./Core/Lib/Src/oled.cyclo ./Core/Lib/Src/oled.d ./Core/Lib/Src/oled.o ./Core/Lib/Src/oled.su ./Core/Lib/Src/spi.cyclo ./Core/Lib/Src/spi.d ./Core/Lib/Src/spi.o ./Core/Lib/Src/spi.su ./Core/Lib/Src/test.cyclo ./Core/Lib/Src/test.d ./Core/Lib/Src/test.o ./Core/Lib/Src/test.su

.PHONY: clean-Core-2f-Lib-2f-Src

