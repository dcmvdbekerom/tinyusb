# Toolchain
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
CFLAGS = -mcpu=cortex-m4 -mthumb -O2 -ffunction-sections -fdata-sections -Wall -std=gnu11 \
         -DSTM32F446xx \
         -Iinclude -Iinc -Itinyusb -Itinyusb/class/hid -Itinyusb/src/common \
         -ICMSIS/Device/ST/STM32F4xx/Include -ICMSIS/Core/Include
LDFLAGS = -TSTM32F446RET6.ld -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/output.map

BUILD_DIR := build

SRCS = src/main.c \
       src/usb_descriptors.c \
       tinyusb/tusb.c \
       tinyusb/class/hid/hid_device.c \
       tinyusb/device/usbd.c \
       tinyusb/device/usbd_control.c \
       tinyusb/common/tusb_fifo.c \
       tinyusb/portable/synopsys/dwc2/dcd_dwc2.c \
       tinyusb/portable/synopsys/dwc2/dwc2_common.c \
       CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c \
       CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f446xx.s

# Replace .c and .s extensions with .o inside build dir
OBJECTS = $(SRCS:%.c=$(BUILD_DIR)/%.o)
OBJECTS := $(OBJECTS:%.s=$(BUILD_DIR)/%.o)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@


all: $(BUILD_DIR)/output.elf $(BUILD_DIR)/output.bin

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Helper to create directory for object file
define make_obj_dir
	mkdir -p $(dir $1)
endef

$(BUILD_DIR)/output.elf: $(BUILD_DIR) $(OBJECTS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJECTS)

$(BUILD_DIR)/output.bin: $(BUILD_DIR)/output.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(call make_obj_dir,$@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s | $(BUILD_DIR)
	$(call make_obj_dir,$@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)/*

.PHONY: all clean