#!/usr/bin/env bash

stty -F /dev/ttyACM0 speed 115200

echo "" > /dev/ttyACM0
echo "" > /dev/ttyACM0
echo "" > /dev/ttyACM0
echo "usbsd" > /dev/ttyACM0
echo "usbsd" > /dev/ttyACM0
echo "usbsd" > /dev/ttyACM0
echo "usbsd" > /dev/ttyACM0
sleep 4
udisksctl mount -b /dev/sdb
cp *.py /run/media/$USER/JEVOIS/modules/Team4069/TapeDetector/
udisksctl unmount -b /dev/sdb
sleep 1
echo "restart" > /dev/ttyACM0
echo "restart" > /dev/ttyACM0
echo "restart" > /dev/ttyACM0
echo "restart" > /dev/ttyACM0
echo "restart" > /dev/ttyACM0
echo "Files Deployed"
