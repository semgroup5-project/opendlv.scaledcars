#!/bin/bash
#if you dont have xterm on your linux distro, run "sudo apt install xterm"

xterm -hold -e "cd $HOME/CLionProjects/DIT_168/opendlv.scaledcars/docker && make IMAGE=opendlv-on-opendlv-core-on-opendavinci-on-base-armhf updateDockerBaseImage && make IMAGE=opendlv-on-opendlv-core-on-opendavinci-on-base-armhf buildComplete && make IMAGE=opendlv-on-opendlv-core-on-opendavinci-on-base-armhf createDockerImage && make IMAGE=opendlv-on-opendlv-core-on-opendavinci-on-base-armhf removeNoneImagesFromDocker" &

