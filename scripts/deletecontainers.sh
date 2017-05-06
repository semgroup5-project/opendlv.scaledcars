#!/bin/bash
#if you dont have xterm on your linux distro, run "sudo apt install xterm"

docker rm -f $(docker ps -a -q)
