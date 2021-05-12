# James Graham 2021
# Automatic AVR Makefile


# Configuration #

# The name of the executable (.hex)
TARGET_EXEC := robot
# Where the object files go.
BUILD_DIR := ./build
# Source code location
SRC_DIRS := ./src
# avr-g++ specific flags
CPPFLAGS := -std=c++17 -DUSB_PID=null -DUSB_VID=null -D__PROG_TYPES_COMPAT__
# General Flags
CXXFLAGS += -Os -Wall -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=105
# Library Flags
LNKFLAGS = rcs
# Linker Flags
LDFLAGS = -mmcu=atmega328p -Wl,--gc-sections -Os -flto -fuse-linker-plugin -lc -lm

# AVR Toolchain Locations
AVR_DIR = /usr/share/arduino/hardware/tools/avr/bin
CC = $(AVR_DIR)/avr-gcc
CXX = $(AVR_DIR)/avr-g++
LNK = $(AVR_DIR)/avr-gcc-ar
OBJCP = $(AVR_DIR)/avr-objcopy
AVRSZ = $(AVR_DIR)/avr-size

# ROSLIB Library Location
ROSLIB_DIR = /home/eced3901/sketchbook/libraries/ros_lib/

# Arduino Library Information
ARDUINO_DIR = /usr/share/arduino/




# Targets #


# Include Arduino Source Files
ARDLIB_DIR := $(ARDUINO_DIR)hardware/arduino/cores/arduino
ARD_SRCS := $(shell find $(ARDLIB_DIR) -name *.cpp -or -name *.c)
# Add ROSLIB Sources
ARD_SRCS += $(shell find $(ROSLIB_DIR) -name *.cpp -or -name *.c)

# Find all the C and C++ files we want to compile
SRCS += $(shell find $(SRC_DIRS) -name *.cpp -or -name *.c)


# String substitution for every C/C++ file.
# As an example, hello.cpp turns into ./build/hello.cpp.o
ARD_OBJS := $(ARD_SRCS:%=$(BUILD_DIR)/%.o)
OBJS := $(SRCS:%=$(BUILD_DIR)/%.o)

# String substitution (suffix version without %).
# As an example, ./build/hello.cpp.o turns into ./build/hello.cpp.d
DEPS := $(OBJS:.o=.d)

# Every folder in ./src will need to be passed to GCC so that it can find header files
INC_DIRS := $(shell find $(SRC_DIRS) -type d)
# Add a prefix to INC_DIRS. So moduleA would become -ImoduleA. GCC understands this -I flag
INC_FLAGS := $(addprefix -I,$(INC_DIRS))
# Include Arduino Flags
INC_FLAGS += -include Arduino.h
INC_FLAGS += -I$(ARDUINO_DIR)hardware/arduino/cores/arduino
INC_FLAGS += -I$(ARDUINO_DIR)hardware/arduino/variants/eightanaloginputs
# Add ROSLIB_DIR to INC_FLAGS
INC_FLAGS += -I$(ROSLIB_DIR)

# Object Copy
EXEC_DIR = $(BUILD_DIR)/$(TARGET_EXEC)
OBJCPFLAGS += -j .eeprom --set-section-flags=.eeprom='alloc,load'
OBJCPFLAGS += --no-change-warnings --change-section-lma .eeprom=0 -O ihex
OBJCPFLAGS += $(EXEC_DIR).elf $(EXEC_DIR).eep

OBJCP2FLAGS += -O ihex -R .eeprom $(EXEC_DIR).elf $(EXEC_DIR).hex
AVRSZFLAGS = --mcu=atmega328p -C --format=avr $(EXEC_DIR).elf

ARD_LIB = $(BUILD_DIR)/libcore.a

# The -MMD and -MP flags together generate Makefiles for us!
# These files will have .d instead of .o as the output.
CXXFLAGS += $(INC_FLAGS) -MMD -MP

# The final build step.
$(EXEC_DIR): $(OBJS) $(ARD_LIB)
	$(CC) $(LDFLAGS) $(OBJS) $(ARD_LIB) -o $@.elf
	# Object Copy to .hex
	$(OBJCP) $(OBJCPFLAGS)
	$(OBJCP) $(OBJCP2FLAGS)
	$(AVRSZ) $(AVRSZFLAGS)

$(ARD_LIB): $(ARD_OBJS)
	$(LNK) $(LNKFLAGS) $@ $^

# Build step for C source
$(BUILD_DIR)/%.c.o: %.c
	mkdir -p $(dir $@)
	$(CC) $(CXXFLAGS) $(CFLAGS) -c $< -o $@

# Build step for C++ source
$(BUILD_DIR)/%.cpp.o: %.cpp
	mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@


.PHONY: clean
clean:
	rm -r $(BUILD_DIR)

# Include the .d makefiles. The - at the front suppresses the errors of missing
# Makefiles. Initially, all the .d files will be missing, and we don't want those
# errors to show up.
-include $(DEPS)
