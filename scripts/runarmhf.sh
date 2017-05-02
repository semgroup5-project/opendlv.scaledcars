#!/bin/bash

if [ -f /dev/video8];
then
    echo "Removing file /dev/video0..." >&2
    rm -f /dev/video0
    if [ -f /dev/video0];
    then
        echo "Could not remove file /dev/video0..." >&2
    else
        ln -f /dev/video8 /dev/video0
        echo "Dev symlink created! Please set camId to 0" >&2
        cd $HOME/opendlv.scaledcars/usecases/armhf/DIT-168 && ./setupODVconfig.sh && cd $HOME/opendlv.scaledcars/usecases/armhf && xhost + && docker-compose down && docker-compose up
    fi
else
    echo "File /dev/video0 not found pleae plugg your webCam and try again..." >&2
fi
