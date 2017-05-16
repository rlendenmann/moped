# prefered version
CC_VERSION=6.2.1
# ARMv7, little endian
cflags-y += -O2  -mcpu=cortex-a7 -mfpu=neon
cflags-y += -ggdb

cflags-y += -ffunction-sections

lib-y   += -lgcc -lc

ASFLAGS += -mcpu=cortex-a7 -mfpu=neon

ifeq ($(ARCH_MCU),bcm2835)
cflags-$(ARCH_MCU) += -DBCM2835=1
ASFLAGS += -DBCM2835=1
else
ifeq ($(ARCH_MCU),bcm2836)
cflags-$(ARCH_MCU) += -DBCM2836=1 -DMULTI_CPU=1
ASFLAGS += -DBCM2836=1 -DMULTI_CPU=1
else
ifeq ($(ARCH_MCU),bcm2837)
cflags-$(ARCH_MCU) += -DBCM2837=1 -DMULTI_CPU=1
ASFLAGS += -DBCM2837=1 -DMULTI_CPU=1
endif
endif
endif
