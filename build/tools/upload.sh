#!/bin/bash

# This script uploads the .hex file to the Pi and puts it on the Arduino

username="ubuntu"
remote_ip="10.0.39.39"
remote_path="~/"
hex_path="obj/robot.hex"
port=/dev/ttyUSB0

# Upload the file

scp $hex_path $username@$remote_ip:$remote_path

# Tell Pi's avrdude to upload.
echo "avrdude -c avrisp -p m328p -b 57600 -P $port -D -Uflash:w:robot.hex" | ssh -T $username@$remote_ip
