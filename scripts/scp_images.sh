#!/bin/sh
echo "Beginning file transfer from pi zero"
scp pi@raspberrypi.local:/home/pi/images/* /home/ubuntu/images
echo "File transfer completed. Exiting sript"
