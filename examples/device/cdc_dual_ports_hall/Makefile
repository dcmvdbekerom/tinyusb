include ../../../hw/bsp/family_support.mk

INC += \
  src \


# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(EXAMPLE_PATH)/, $(EXAMPLE_SOURCE))
CFLAGS +=  -Wno-cast-align

include ../../../hw/bsp/family_rules.mk
