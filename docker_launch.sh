#!/bin/bash
sudo xhost +local:docker
docker run -it -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro --mount type=volume,src=utn_robotics_volume,dst=/home/utn_robotica/ros_ws --user root --name ROS_UTN robotica_utn:latest
