# Team 8 Robot Arduino Code

This is Team 8's Arduino code, featuring a custom avr-gcc Makefile and external uploading functionalities.

## Installation Procedure

Note: You should already have the avr-gcc toolchain as it is preinstalled on the VM. Avrdude is also already on the Pi.

 - Download C++ libraries from Pi:
	```
	mkdir -p ~/sketchbook/
	scp -r ubuntu@10.0.39.39:~/sketchbook/libraries/ ~/sketchbook/
	```
	
 - Setup SSH Keys (optional)
	```
	ssh-keygen
	ssh-copy-id ubuntu@10.0.39.39
	```

	SSH Keys are not required but they will prevent you from needing to enter the Pi's password a lot.
	
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
