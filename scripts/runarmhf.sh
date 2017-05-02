#!/bin/bash

echo "Removing file /dev/video0..." >&2
rm -f /dev/video0
ln -f /dev/video8 /dev/video0
echo "Dev symlink created! Please set camId to 0" >&2
cd $HOME/opendlv.scaledcars/usecases/armhf/DIT-168 && ./setupODVconfig.sh && cd $HOME/opendlv.scaledcars/usecases/armhf && xhost + && docker-compose down && docker-compose up

