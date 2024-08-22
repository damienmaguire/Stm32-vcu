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

BINARY		= stm32_vcu
OUT_DIR     = obj
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
              -fno-common -fno-builtin -pedantic -DSTM32F1 -DMAX_USER_MESSAGES=30 \
				  -mcpu=cortex-m3 -mthumb -std=gnu99 -ffunction-sections -fdata-sections -ggdb3
CPPFLAGS    = -Os -Wall -Wextra -Ilibopeninv/include -Iinclude/ -Ilibopencm3/include \
              -fno-common -std=c++17 -pedantic -DSTM32F1 -DMAX_USER_MESSAGES=30  \
				  -ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions \
              -fno-unwind-tables -mcpu=cortex-m3 -mthumb -ggdb3
LDSCRIPT	= $(BINARY).ld
LDFLAGS  = -Llibopencm3/lib -T$(LDSCRIPT) -march=armv7 -nostartfiles -Wl,--gc-sections,-Map,linker.map
OBJSL		= $(BINARY).o hwinit.o stm32scheduler.o params.o terminal.o terminal_prj.o \
           my_string.o digio.o my_fp.o printf.o anain.o throttle.o isa_shunt.o BMW_E65.o GS450H.o temp_meas.o \
           BMW_E39.o Can_VAG.o Can_OI.o MCP2515.o CANSPI.o outlanderinverter.o canhardware.o canmap.o \
           param_save.o errormessage.o stm32_can.o leafinv.o utils.o terminalcommands.o i3LIM.o \
           chademo.o amperaheater.o amperacharger.o subaruvehicle.o iomatrix.o bmw_sbox.o NissanPDM.o teslaCharger.o extCharger.o vag_sbox.o \
           daisychainbms.o simpbms.o outlanderCharger.o Can_OBD2.o cansdo.o TeslaDCDC.o BMW_E31.o F30_Lever.o \
           CPC.o ElconCharger.o RearOutlanderinverter.o linbus.o VWheater.o JLR_G1.o JLR_G2.o Focci.o \
		   E65_Lever.o
           
OBJS     = $(patsubst %.o,$(OUT_DIR)/%.o, $(OBJSL))
vpath %.c src/ libopeninv/src/
vpath %.cpp src/ libopeninv/src/

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
	$(Q)$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS) -lopencm3_stm32f1 -lm

$(OUT_DIR)/%.o: %.c Makefile
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(OUT_DIR)/%.o: %.cpp Makefile
	@printf "  CPP     $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CPP) $(CPPFLAGS) -MMD -MP -o $@ -c $<

DEP = $(OBJS:%.o=%.d)
-include $(DEP)

clean:
	@printf "  CLEAN   ${OUT_DIR}\n"
	$(Q)rm -rf ${OUT_DIR}
	@printf "  CLEAN   $(BINARY)\n"
	$(Q)rm -f $(BINARY)
	@printf "  CLEAN   $(BINARY).bin\n"
	$(Q)rm -f $(BINARY).bin
	@printf "  CLEAN   $(BINARY).hex\n"
	$(Q)rm -f $(BINARY).hex
	@printf "  CLEAN   $(BINARY).srec\n"
	$(Q)rm -f $(BINARY).srec
	@printf "  CLEAN   $(BINARY).list\n"
	$(Q)rm -f $(BINARY).list

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

.PHONY: directories get-deps images clean

get-deps:
ifneq ($(shell test -s libopencm3/lib/libopencm3_stm32f1.a && echo -n yes),yes)
	@printf "  GIT SUBMODULE\n"
	$(Q)git submodule update --init
	@printf "  MAKE libopencm3\n"
	$(Q)${MAKE} -C libopencm3
endif

Test:
	cd test && $(MAKE)
cleanTest:
	cd test && $(MAKE) clean
