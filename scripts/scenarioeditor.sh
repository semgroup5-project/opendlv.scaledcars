#!/bin/bash
# cockpit startup script
# opening multiple terminals without the hassle

# to make this script executable locate it on the terminal and run "chmod +x cockpit.sh"
# so it will execute when you call it with "sudo ./cockpit.sh"

# you can always add more commands to execute if you need more components to run
# example : 'xterm -hold -e "<docker command>" &'

#if you dont have xterm on your linux distro, run "sudo apt install xterm"

xterm -hold -e "xhost + && docker run -t -i --rm -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/$USER/scenarios:/opt/data -w /opt/ScenarioEditor seresearch/scenario_editor:v2 ./ScUI" &



