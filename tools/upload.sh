#!/bin/bash

# This script uploads the .hex file to the Pi and puts it on the Arduino

# Configuration
username=ubuntu			# Username of the Pi
remote_ip=10.0.39.39		# IP Address of the Pi
remote_path=robot.hex		# Where to put the .hex file on the Pi
hex_path=build/robot.hex	# Where the .hex file is relative to execution location

# Upload the file to the Pi
scp $hex_path $username@$remote_ip:$remote_path

# Get path to USB port
usb_port=$(echo "ls /dev/ | grep ttyUSB" | ssh -T $username@$remote_ip | tail -1)
echo "Found USB Port $usb_port"

# Tell Pi's avrdude to upload. Use the first USB port found. We're assuming the Pi only has one USB connection.
echo "avrdude -c arduino -p m328p -b 57600 -P /dev/$usb_port -D -Uflash:w:$remote_path:i" | ssh -T $username@$remote_ip
echo "*** UPLOAD COMPLETE ***"
