#!/bin/bash
# cockpit startup script
# opening multiple terminals without the hassle

# to make this script executable locate it on the terminal and run "chmod +x cockpit.sh"
# so it will execute when you call it with "sudo ./cockpit.sh"

# you can always add more commands to execute if you need more components to run
# example : 'xterm -hold -e "<docker command>" &'

#if you dont have xterm on your linux distro, run "sudo apt install xterm"

xterm -hold -e "xhost + && docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsimcamera --freq=10 --cid=111" &
