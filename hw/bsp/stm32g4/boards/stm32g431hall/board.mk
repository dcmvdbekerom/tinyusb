# STM32G431K6U6 32 pins 32kB ROM UFQFPN  32kB RAM 
# Build with:
# make BOARD=stm32g431hall flash-openocd OPENOCD_OPTION="-f interface/stlink.cfg -f target/stm32g4x.cfg"

MCU_VARIANT = stm32g431xx

CFLAGS += \
	-DSTM32G431xx
# Linker
LD_FILE = $(BOARD_PATH)/STM32G431K6Ux_FLASH.ld

# For flash-jlink target
JLINK_DEVICE = stm32g431k6
