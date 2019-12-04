#!/bin/sh

# ========================
# RUN THIS INSIDE A SCREEN
# ========================

./maze-1fg.py -g /dev/ttyACM0 2>&1 | tee -a maze.log

# Use for RFID-reader
#	-r /dev/ttyUSB0 

echo 
echo PRESS ANY KEY TO EXIT

read TMP
