#!/bin/bash 
echo "start wait for $1 seconds"
sleep $1
echo "end wait for $1 seconds"

shift # The sleep time is droped
    echo "now running 'roslaunch $@'"
roslaunch "$@"