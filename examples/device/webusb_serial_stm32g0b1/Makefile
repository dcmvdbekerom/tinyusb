include ../../../hw/bsp/family_support.mk

INC += \
  src \
  ../../../hw/mcu/st/cmsis_core/Include \
  $(TOP)/hw \

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)

SRC_C += $(addprefix $(EXAMPLE_PATH)/, $(EXAMPLE_SOURCE))
CFLAGS += -mcpu=cortex-m0plus -mthumb -DSTM32G0B1xx -D__CORTEX_SC=0U -Wno-cast-align

include ../../../hw/bsp/family_rules.mk
