# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=usbtest

##########
# DEFINES
##########
#DEFINES = -DJSMN_PARENT_LINKS -DUSE_USB_FS
######## - DEFINES

##########
# STLINK
##########
STLINK=/home/elkhadiy/stlink
######## - STLINK

#############
# cross tools
#############
CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld
OBJCOPY=arm-none-eabi-objcopy
######## - cross tools

################
# COMMON CFLAGS
################

CFLAGS  = -g -Wall -std=gnu99
CFLAGS += -T$(LDSCRIPT)
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.
CFLAGS += -I/usr/arm-none-eabi/lib

CFLAGS += -lrdimon #--specs=rdimon.specs 
CFLAGS += $(DEFINES)

######## - COMMON CFLAGS

#########
# ARM DEF
#########
CFLAGS += -IDrivers/CMSIS/Include/
######## - ARM DEF

##################
# STM32F4xx Drivers
##################

CFLAGS += -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
SRCS += Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c

CFLAGS += -IDrivers/STM32F4xx_HAL_Driver/Inc
SRCS += $(wildcard Drivers/STM32F4xx_HAL_Driver/Src/*.c)

######## - STM32F4xx Drivers


########################
# specific to STM32F429
########################

DEFS = -DSTM32F429xx

SRCS += Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f429xx.s

LDSCRIPT = SW4STM32/usbtest\ Configuration/STM32F429ZITx_FLASH.ld

######## - specific to STM32F429

CFLAGS += -IInc
SRCS += $(wildcard Src/*.c)

##########
# Rules
##########
OBJS = $(SRCS:.c=.o)

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	@echo "Compiling project....."
	@$(CC) $(CFLAGS) $(DEFS) $(LDFLAGS) $^ -o $@
	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	@echo "DONE"

clean:
	@echo "Cleaning object files and binaries....."
	@rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin
	@echo "DONE"

# Flash the STM32F4
burn: proj
	$(STLINK)/st-flash write $(PROJ_NAME).bin 0x8000000
