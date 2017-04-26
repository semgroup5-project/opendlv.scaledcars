#!/bin/bash
# cockpit startup script
# opening multiple terminals without the hassle

# to make this script executable locate it on the terminal and run "chmod +x cockpit.sh"
# so it will execute when you call it with "sudo ./cockpit.sh"

# you can always add more commands to execute if you need more components to run
# example : 'xterm -hold -e "<docker command>" &'

#if you dont have xterm on your linux distro, run "sudo apt install xterm"

xterm -hold -e "docker run -ti --rm --net=host -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/od4/bin/odsupercomponent --cid=111 --verbose=1" &
sleep 3
xterm -hold -e "docker run -ti --rm --net=host seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/od4/bin/odsimvehicle --cid=111 --freq=10" &
sleep 3
xterm -hold -e "xhost + && docker run -ti --rm --net=host -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest  /opt/od4/bin/odsimirus --cid=111 --freq=10" &
sleep 3
xterm -hold -e "docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/od4/bin/odcockpit --cid=111" &
