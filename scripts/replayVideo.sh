#!/bin/bash

# opening multiple terminals without the hassle

# to make this script executable locate it on the terminal and run "chmod +x cameraProxy.sh"
# so it will execute when you call it with "sudo ./cameraProxy.sh"

# you can always add more commands to execute if you need more components to run
# example : 'xterm -hold -e "<docker command>" &'

#if you dont have xterm on your linux distro, run "sudo apt install xterm"

xterm -hold -e "docker run -ti --rm --net=host -v /opt/od/bin:/opt/opendlv.core.configuration seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsupercomponent --cid=111 --verbose=1 --configuration=/opt/opendlv.core.configuration/configuration" &
sleep 3
xterm -hold -e "xhost + && docker run -ti --rm --net=host --ipc=host --user=odv -e	DISPLAY=$DISPLAY -v	/tmp/.X11-unix:/tmp/.X11-unix -v ~/recordings:/opt/recordings seresearch/opendlv-core-on-opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odcockpit --cid=111" &




