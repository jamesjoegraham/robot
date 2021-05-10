# Team 8 Robot Arduino Code

##Installation Procedure

You should already have avr-gcc and avrdude as they were preinstalled on the VM.

 - Update APT package list:

	sudo apt-get update

 - Install pySerial:

	sudo apt-get install python3-serial

 - Install arduino-mk:
	
	sudo apt-get install arduino-mk

 - Install ros library:
	
	Download from PI
	mkdir ~/sketchbook/libraries
	scp -r ubuntu@10.0.39.39:~/sketchbook/libraries/ros_lib ~/sketchbook/libraries/
	Open Arduino IDE->Sketch->Import Library->Select ~/sketchbook/libraries/ros_lib

 - Setup SSH Keys

	
##Build Procedure

 - Navigate to the build directory

	cd build

 - Run make to build:

	make

 - Run make upload to upload to Raspberry PI -> Arduino

	make upload


