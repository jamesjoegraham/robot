# James Graham 2021

# The root project directory. All others are relative.
PROJ_DIR=/home/eced3901/develop/robot/
# Source code location
SRC_DIR=src/
# Object file directory
OBJ_DIR=build
# The name of the .hex file
EXECUTABLE=robot

# Location of installed library ros_lib
ROSLIB_DIR=/home/ubuntu/sketchbook/libraries/ros_lib

# AVR Toolchain Location
AVR_DIR=/usr/share/arduino/hardware/tools/avr/bin/
# Compiler flags
CFLAGS := -Os -Wall -std=c++17 -DUSB-PID=null -DUSB_VID=null -D__PROG_TYPES_COMPAT__
# AVR-specific compiler flags
CFLAGS := -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=105

#########################################################################
# Targets

# Add ROSLIB
CFLAGS := -I$(ROSLIB_DIR)

FILES 	= Main.cpp
SOURCES = $(patsubst %,$(SRC_DIR)/%,$(FILES))
OBJECTS = $(patsubst %.c,$(OBJ_DIR)/%.o,$(FILES))

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): ($OBJECTS)
	$(AVR_DIR)/avr-ar rcs $(OBJECTS) $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	

robot:
	$(AVR_DIR)/avr-g++ -c -g $(CFLAGS)
	


clean:
	# Just delete the whole directory.
	rm -rf $PROJ_DIR/$OBJ_DIR
