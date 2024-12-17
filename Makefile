################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# Copyright (2024), Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


################################################################################
# Basic Configuration
################################################################################

# Type of ModusToolbox Makefile Options include:
#
# COMBINED    -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT     -- Project Makefile under Application
#
MTB_TYPE=COMBINED

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make library-manager' from command line), which will also update Eclipse IDE launch
# configurations.
TARGET=KIT_FX2G3_104LGA

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=mtb-example-fx2g3-uvc-uac

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox software
# ARM     -- ARM Clang Compiler (must be installed separately)
#
# To use the ARM toolchain, ensure the CY_COMPILER_ARM_DIR environment variable is set to the compiler's directory (absolute path).
# For example, the default path for ARMCLANG from a Keil installation is typically C:/Keil_v5/ARM/ARMCLANG.
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Generate Bootloader enabled hex. Options include:
#
# yes -- Hex with bootloader region
# no  -- Hex without bootloader region
BLENABLE ?= yes

# Generate hex compatible with REV2 Options include:
#
# yes -- Hex compatible for REV02 kit
# no  -- Hex compatible for REV01 kit
REV02 ?= yes

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
#
# If CONFIG is manually edited, ensure to update or regenerate launch configurations
# for your IDE.
CONFIG=Release

# If set to "true" or "1", display full command-lines when building.
VERBOSE=

# Name of CORE to use. Options include:
#
# CM4  -- Cortex M4
CORE=CM4

################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
COMPONENTS=FREERTOS RTOS_AWARE FX2G3_$(CORE)

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=

# Add additional defines to the build process (without a leading -D).
DEFINES= \
        FX2G3_EN=1 \
        CYUSB4014_BZXI=1 \
        BCLK__BUS_CLK__HZ=75000000 \
        DEBUG_INFRA_EN=1 \
        FREERTOS_ENABLE=1 \
        USBFS_LOGS_ENABLE=1 \
        BUS_WIDTH_16=1 \
        INTERLEAVE_EN=0 \
        DEVICE1_EN=0 \
        PRE_ADDED_HEADER=0 \
        MIPI_SOURCE_ENABLE=1 \
        AUDIO_IF_EN=1 \
        STEREO_ENABLE=0

# Conditionally append BLOAD_ENABLE=1 if BLENABLE is set to yes
ifneq ($(BLENABLE),)
ifneq ($(BLENABLE), no)
    DEFINES += BLOAD_ENABLE=1
endif
endif

ifeq ($(REV02), yes)
    DEFINES += FLASH_AT45D=0
else
    DEFINES += FLASH_AT45D=1
endif

# Select softfp or hardfp floating point. Default is softfp.
ifeq ($(TOOLCHAIN), GCC_ARM)
    VFP_SELECT= softfp
else ifeq ($(TOOLCHAIN), ARM)
    VFP_SELECT= soft
endif

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ifeq ($(TOOLCHAIN), GCC_ARM)
    CFLAGS= -Os -Og -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mthumb -ffunction-sections -fdata-sections -g
else ifeq ($(TOOLCHAIN), ARM)
    CFLAGS= -Os -Og -mcpu=Cortex-M4 -mfpu=fpv4-sp-d16 -mthumb -ffunction-sections -fdata-sections -g \
            --target=arm-arm-none-eabi -fno-rtti -fno-exceptions -Wno-error -fshort-wchar -fshort-enums
endif

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
CXXFLAGS=

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
ifeq ($(TOOLCHAIN), ARM)
    LDFLAGS=--cpu=Cortex-M4 --entry=Reset_Handler --diag_suppress=L6329W,L6314W
else ifeq ($(TOOLCHAIN), GCC_ARM)
    LDFLAGS=-Wl,--start-group -mcpu=cortex-m4 -mthumb --entry=Reset_Handler -Wl,--gc-sections -g -ffunction-sections -finline-functions -Os -Wl,--end-group
endif

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
ifeq ($(CORE),CM4)
    ifeq ($(BLENABLE), yes)
        # Use loadable linker script for CM4 core
        LINKER_SCRIPT = $(if $(filter GCC_ARM,$(TOOLCHAIN)),fx_cm4_loadable.ld,fx_cm4_loadable.sct)
    else
        # Use dual linker script for CM4 core
        LINKER_SCRIPT = $(if $(filter GCC_ARM,$(TOOLCHAIN)),fx_cm4.ld,fx_cm4_dual.sct)
    endif
endif


# Custom pre-build commands to run.
PREBUILD=

# Custom post-build commands to run.
#Post build to merge bootloader and application
ifeq ($(BLENABLE), yes)
    POSTBUILD=\
        $(CY_MCUELFTOOL) --sign build/$(TARGET)/$(CONFIG)/$(APPNAME).elf SHA256 --output build/$(TARGET)/$(CONFIG)/$(APPNAME).sha.elf && \
        $(OBJCOPY) -O ihex --gap-fill 0 build/$(TARGET)/$(CONFIG)/$(APPNAME).sha.elf build/$(TARGET)/$(CONFIG)/$(APPNAME).hex
else
ifeq ($(TOOLCHAIN), ARM)
    POSTBUILD=\
        $(FROMELF) --i32combined --base=0x10000000 -o build/$(TARGET)/$(CONFIG)/$(APPNAME).hex build/$(TARGET)/$(CONFIG)/$(APPNAME).elf
else
    POSTBUILD=\
        $(OBJCOPY) -O ihex build/$(TARGET)/$(CONFIG)/$(APPNAME).elf build/$(TARGET)/$(CONFIG)/$(APPNAME).hex
endif

endif

################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory. The variable name depends on the
# toolchain used for the build. Refer to the ModusToolbox user guide to get the correct
# variable name for the toolchain used in your build.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# software provided compiler by default).
CY_COMPILER_GCC_ARM_DIR=

# Locate ModusToolbox helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_* \
    $(HOME)/ModusToolbox/tools_* \
    /Applications/ModusToolbox/tools_*)

# If you install ModusToolbox software in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder). Make sure you use forward slashes.
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

# Path to Elf tool directory. 
CY_MCUELFTOOL_DIR=$(wildcard $(CY_TOOLS_DIR)/cymcuelftool-*)

# CY MCU ELF tool executable path.
ifeq ($(OS),Windows_NT)
    CY_MCUELFTOOL=$(CY_MCUELFTOOL_DIR)/bin/cymcuelftool.exe
else
    CY_MCUELFTOOL=$(CY_MCUELFTOOL_DIR)/bin/cymcuelftool
endif

# Path to OBJCOPY tool directory.
ifeq ($(TOOLCHAIN), ARM)
FROMELF_DIR= $(wildcard $(CY_COMPILER_ARM_DIR)/bin)
endif

OBJCOPY_DIR=$(wildcard $(CY_TOOLS_DIR)/gcc/arm-none-eabi/bin)


# Path to FROMELF and OBJCOPY tool directory.
ifeq ($(OS),Windows_NT)
    ifeq ($(TOOLCHAIN), ARM)
        FROMELF=$(FROMELF_DIR)/fromelf.exe
    endif
    OBJCOPY=$(OBJCOPY_DIR)/objcopy.exe
else
    ifeq ($(TOOLCHAIN), ARM)
        FROMELF=$(FROMELF_DIR)/fromelf
    endif
    OBJCOPY=$(OBJCOPY_DIR)/objcopy
endif

include $(CY_TOOLS_DIR)/make/start.mk
