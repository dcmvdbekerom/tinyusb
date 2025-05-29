BOARD = stm32g0b1aliex


include ../build_system/make.mk

INC += \
  src \
  ../hw/mcu/st/cmsis_core/Include \
  $(TOP)/hw \


# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)

SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))
   

CFLAGS += -mcpu=cortex-m0plus -mthumb -DSTM32G0B1xx -DSTM32G0 -D__CORTEX_SC=0U -Wno-cast-align

include ../build_system/rules.mk
