xterm -hold -e "docker run -ti --rm --net=host --ipc=host  --group-add video --device=/dev/video0:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix seresearch/scaledcars-on-opendlv-on-opendlv-core-on-opendavinci-on-base-dev:latest /opt/opendlv.scaledcars/bin/scaledcars-control-proxy --cid=111 --freq=20 --priveleged" &


#--user=odv
