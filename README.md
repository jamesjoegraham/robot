# Team 8 Robot Arduino Code

This is Team 8's arduino code, featuring a custom avr-gcc Makefile and external uploading functionalities.

## Installation Procedure

Note: You should already have the avr-gcc toolchain as it were preinstalled on the VM. Avrdude is also already on the Pi.

 - Download ROS C++ library from Pi:
	```
	mkdir ~/sketchbook/libraries
	scp -r ubuntu@10.0.39.39:~/sketchbook/libraries/ros_lib ~/sketchbook/libraries/
	```
	
 - Setup SSH Keys (optional)
	
	SSH Keys are optional but they will prevent you from needing to enter the Pi's password a lot.
	
## Build Procedure

 - Run make to build:
	```
	make
	```
 - Clean up object/executable files (don't do this before upload):
	```
	make clean
	```
## Upload Procedure

 - Run the upload script:
	```
	tools/upload.sh
	```
