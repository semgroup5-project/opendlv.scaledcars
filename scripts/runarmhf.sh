#!/bin/bash

#define parameters which are passed in.
#change config
ANS=$1
#change pid
ANS2=$2
IP=$3
P=$4
I=$5
D=$6
FUNCTION=$7
FUNCTION2=$8

sudo v4l2-ctl -d /dev/video8 -c focus_auto=0
echo "Removing file /dev/video0..." >&2
sudo rm -f /dev/video0
sudo ln -f /dev/video8 /dev/video0
echo "Dev symlink created! Please set camId to 0" >&2
cd $HOME/opendlv.scaledcars/usecases/armhf/DIT-168 && ./setupODVconfig.sh $ANS $ANS2 $IP $P $I $D $FUNCTION $FUNCTION2 && cd $HOME/opendlv.scaledcars/usecases/armhf && docker-compose down && docker-compose up --build

#$ANS $ANS2 $IP $P $I $D $FUNCTION $FUNCTION2

