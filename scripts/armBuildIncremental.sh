#!/bin/bash
#if you dont have xterm on your linux distro, run "sudo apt install xterm"

xterm -hold -e "cd $HOME/CLionProjects/DIT_168/opendlv.scaledcars/docker && make BASE_IMAGE=seresearch/opendlv-on-opendlv-core-on-opendavinci-ubuntu-16.04-armhf-complete updateDockerBaseImage && make BASE_IMAGE=seresearch/opendlv-on-opendlv-core-on-opendavinci-ubuntu-16.04-armhf-complete buildIncremental && make BASE_IMAGE=seresearch/opendlv-on-opendlv-core-on-opendavinci-ubuntu-16.04-armhf-complete createDockerImage && make BASE_IMAGE=seresearch/opendlv-on-opendlv-core-on-opendavinci-ubuntu-16.04-armhf-complete removeNoneImagesFromDocker" &
