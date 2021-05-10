#
# Configuration

# Type of Arduino Board
BOARD_TAG = nano
# Type of chipset
BOARD_SUB = atmega328
# USB Port of Arduino
MONITOR_PORT = /dev/ttyUSB0
# Location of Arduino IDE
ARDUINO_DIR = /usr/share/arduino
# Location of avrdude
AVRDUDE = /usr/bin/avrdude
# Location of avrdude configuration
AVRDUDE_CONF = /etc/avrdude
# Include the ros library
ARDUINO_LIBS = ros_lib
# Location of Makefile library root directory
ARDMK_DIR = /usr/share/arduino
# CPU Speed
F_CPU = 16000000U

# Object Directory Name
OBJDIR = ../../build/obj

# Include the Arduino Makefile library
include $(ARDMK_DIR)/Arduino.mk

#
# Targets

#LOCAL_CPP_SRCS ?= $(shell find . -type f -name '*.cpp' | tr '\n' ' ')

# Doesn't appear to work.
# CXXFLAGS_STD = -std=c++11
CXXFLAGS := -Wall -std=c++17
#CXXFLAGS := -Werror
