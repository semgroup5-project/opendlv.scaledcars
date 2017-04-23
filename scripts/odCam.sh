xterm -hold -e "xhost + && docker run -ti --rm --net=host --ipc=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $HOME/DIT-168:/opt/configuration -w /opt/configuration seresearch/opendavinci-ubuntu-16.04-complete:latest /opt/od4/bin/odsimcamera --freq=10 --cid=111" &
sleep 3
