##
## This file is part of the libopenstm32 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

# FAMILY=f1 is the STM32F107 build and the default. `make f4` builds the
# STM32F407 image into stm32_vcu_f407, with its own objects and linker script.
# VCU_HSE_HZ is only used by the F4 clock. 8000000 matches the F107
# assumption. Use 25000000 only when that crystal has been measured.
# Make will not rebuild objects when only this value changes on the
# command line, so run `make clean` before switching it.
FAMILY ?= f1
VCU_HSE_HZ ?= 8000000

ifeq ($(FAMILY),f4)
CPUFLAGS        = -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
ARCHDEF         = -DSTM32F4 -DVCU_HSE_HZ=$(VCU_HSE_HZ)
BINARY          = stm32_vcu_f407
OUT_DIR         = obj-f4
LDSCRIPT        = stm32_vcu_f407.ld
OPENCM3_LIB     = opencm3_stm32f4
OPENCM3_TARGETS = stm32/f4
MAPFILE         = linker-f4.map
LDFLAGS_ARCH    = $(CPUFLAGS)
else ifeq ($(FAMILY),f1)
CPUFLAGS        = -mcpu=cortex-m3 -mthumb
ARCHDEF         = -DSTM32F1
BINARY          = stm32_vcu
OUT_DIR         = obj
LDSCRIPT        = stm32_vcu.ld
OPENCM3_LIB     = opencm3_stm32f1
OPENCM3_TARGETS = stm32/f1
MAPFILE         = linker.map
LDFLAGS_ARCH    = -march=armv7
else
$(error FAMILY must be f1 or f4)
endif

PREFIX	  ?= arm-none-eabi
SIZE  = $(PREFIX)-size
CC		= $(PREFIX)-gcc
CPP	= $(PREFIX)-g++
LD		= $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
MKDIR_P     = mkdir -p
TERMINAL_DEBUG ?= 0
CFLAGS		= -Os -Wall -Wextra -Ilibopeninv/include -Iinclude/ -Ilibopencm3/include \
              -fno-common -fno-builtin -pedantic $(ARCHDEF) -DMAX_USER_MESSAGES=30 \
				  $(CPUFLAGS) -std=gnu99 -ffunction-sections -fdata-sections -ggdb3
CPPFLAGS    = -Os -Wall -Wextra -Ilibopeninv/include -Iinclude/ -Ilibopencm3/include \
              -fno-common -std=c++17 -pedantic $(ARCHDEF) -DMAX_USER_MESSAGES=30  \
				  -ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions \
              -fno-unwind-tables $(CPUFLAGS) -ggdb3
LDFLAGS  = -Llibopencm3/lib -T$(LDSCRIPT) $(LDFLAGS_ARCH) -nostartfiles -Wl,--gc-sections,-Map,$(MAPFILE)
OBJSL		= stm32_vcu.o hwinit.o stm32scheduler.o params.o terminal.o terminal_prj.o \
           my_string.o digio.o my_fp.o printf.o anain.o throttle.o isa_shunt.o BMW_E65.o GS450H.o temp_meas.o \
           BMW_E39.o BMW_F11.o Can_VAG.o Can_OI.o MCP2515.o CANSPI.o outlanderinverter.o canhardware.o canmap.o \
           param_save.o errormessage.o stm32_can.o leafinv.o utils.o terminalcommands.o i3LIM.o \
           chademo.o amperaheater.o amperacharger.o subaruvehicle.o iomatrix.o bmw_sbox.o NissanPDM.o teslaCharger.o extCharger.o vag_sbox.o \
           daisychainbms.o simpbms.o outlanderCharger.o Can_OBD2.o cansdo.o sdocommands.o TeslaDCDC.o BMW_E31.o F30_Lever.o \
           CPC.o ElconCharger.o RearOutlanderinverter.o linbus.o VWCoolantHeater.o JLR_G1.o JLR_G2.o Foccci.o digipot.o\
		   OutlanderHeartBeat.o E65_Lever.o leafbms.o V_Classic.o kangoobms.o OutlanderCanHeater.o VWAirHeater.o\
		   MGCoolantHeater.o NissLeafMng.o preheater.o MGgen2V2Lcharger.o OutlanderCompressor.o ElconDCDC.o PWMHeater.o \
		   stwmbms.o oibms.o bmw_sp51.o WebastoHVH.o InverterACP.o Maintainer12V.o PKP2300_Lever.o C5Charger.o C5DCDC.o



OBJS     = $(patsubst %.o,$(OUT_DIR)/%.o, $(OBJSL))
vpath %.c src/ src/HVAC/ src/bms/ src/chargers/ src/dcdc/ src/inverters/ src/shifters/ src/vehicles/ libopeninv/src/
vpath %.cpp src/ src/HVAC/ src/bms/ src/chargers/ src/dcdc/ src/inverters/ src/shifters/ src/vehicles/ libopeninv/src/

OPENOCD_BASE	= /usr
OPENOCD		= $(OPENOCD_BASE)/bin/openocd
OPENOCD_SCRIPTS	= $(OPENOCD_BASE)/share/openocd/scripts
OPENOCD_FLASHER	= $(OPENOCD_SCRIPTS)/interface/parport.cfg
OPENOCD_BOARD	= $(OPENOCD_SCRIPTS)/board/olimex_stm32_h103.cfg

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
endif

# try-run
# Usage: option = $(call try-run, command,option-ok,otherwise)
# Exit code chooses option.
try-run = $(shell set -e;		\
	if ($(1)) >/dev/null 2>&1;	\
	then echo "$(2)";		\
	else echo "$(3)";		\
	fi)

# Test a linker (ld) option and return the gcc link command equivalent
comma := ,
link_command := -Wl$(comma)
ld-option = $(call try-run, $(PREFIX)-ld $(1) -v,$(link_command)$(1))

# Test whether we can suppress a safe warning about rwx segments
# only supported on binutils 2.39 or later
LDFLAGS	+= $(call ld-option,--no-warn-rwx-segments)

all: directories images
Debug:images
Release: images
cleanDebug:clean
images: get-deps $(BINARY)
	@printf "  OBJCOPY $(BINARY).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(BINARY) $(BINARY).bin
	@printf "  OBJCOPY $(BINARY).hex\n"
	$(Q)$(OBJCOPY) -Oihex $(BINARY) $(BINARY).hex
	$(Q)$(SIZE) $(BINARY)

directories: ${OUT_DIR}

${OUT_DIR}:
	$(Q)${MKDIR_P} ${OUT_DIR}

$(BINARY): $(OBJS) $(LDSCRIPT)
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS) -l$(OPENCM3_LIB) -lm

$(OUT_DIR)/%.o: %.c Makefile
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(OUT_DIR)/%.o: %.cpp Makefile
	@printf "  CPP     $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CPP) $(CPPFLAGS) -MMD -MP -o $@ -c $<

DEP = $(OBJS:%.o=%.d)
-include $(DEP)

clean:
	@printf "  CLEAN   obj obj-f4\n"
	$(Q)rm -rf obj obj-f4
	@printf "  CLEAN   firmware images\n"
	$(Q)rm -f stm32_vcu stm32_vcu.bin stm32_vcu.hex stm32_vcu.srec stm32_vcu.list linker.map
	$(Q)rm -f stm32_vcu_f407 stm32_vcu_f407.bin stm32_vcu_f407.hex stm32_vcu_f407.srec stm32_vcu_f407.list linker-f4.map

flash: images
	@printf "  FLASH   $(BINARY).bin\n"
	@# IMPORTANT: Don't use "resume", only "reset" will work correctly!
	$(Q)$(OPENOCD) -s $(OPENOCD_SCRIPTS) \
		       -f $(OPENOCD_FLASHER) \
		       -f $(OPENOCD_BOARD) \
		       -c "init" -c "reset halt" \
		       -c "flash write_image erase $(BINARY).hex" \
		       -c "reset" \
		       -c "shutdown" $(NULL)

.PHONY: directories get-deps images clean f4

# Build the F407 image without touching the F107 objects.
ifneq ($(FAMILY),f4)
f4:
	$(MAKE) FAMILY=f4 all
endif

get-deps:
ifneq ($(shell test -s libopencm3/lib/lib$(OPENCM3_LIB).a && echo -n yes),yes)
	@printf "  GIT SUBMODULE\n"
	$(Q)git submodule update --init
	@printf "  MAKE libopencm3 $(OPENCM3_TARGETS)\n"
	$(Q)${MAKE} -C libopencm3 TARGETS=$(OPENCM3_TARGETS)
endif

Test:
	cd test && $(MAKE)
cleanTest:
	cd test && $(MAKE) clean
