#!/bin/bash
#if you dont have xterm on your linux distro, run "sudo apt install xterm"

xterm -hold -e "cd $HOME/CLionProjects/DIT_168/opendlv.scaledcars/docker && make buildIncremental && make createDockerImage && make removeNoneImagesFromDocker" &

