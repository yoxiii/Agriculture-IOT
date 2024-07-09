################################################################################
# MRS Version: 1.9.1
# 自动生成的文件。不要编辑！
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/AP3216C.c \
../User/IIC.c \
../User/MyI2C.c \
../User/ch32v30x_it.c \
../User/lcd.c \
../User/main.c \
../User/output.c \
../User/serial.c \
../User/sht31_soft.c \
../User/system_ch32v30x.c 

OBJS += \
./User/AP3216C.o \
./User/IIC.o \
./User/MyI2C.o \
./User/ch32v30x_it.o \
./User/lcd.o \
./User/main.o \
./User/output.o \
./User/serial.o \
./User/sht31_soft.o \
./User/system_ch32v30x.o 

C_DEPS += \
./User/AP3216C.d \
./User/IIC.d \
./User/MyI2C.d \
./User/ch32v30x_it.d \
./User/lcd.d \
./User/main.d \
./User/output.d \
./User/serial.d \
./User/sht31_soft.d \
./User/system_ch32v30x.d 


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"C:\Users\YOXI\Desktop\MCU\ch32\ch32v307_bsp_lib\EXAM\SRC\Debug" -I"C:\Users\YOXI\Desktop\MCU\ch32\ch32v307_bsp_lib\EXAM\SRC\Core" -I"C:\Users\YOXI\Desktop\MCU\ch32\ch32v307_bsp_lib\EXAM\USART\merge-loop\User" -I"C:\Users\YOXI\Desktop\MCU\ch32\ch32v307_bsp_lib\EXAM\SRC\Peripheral\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

