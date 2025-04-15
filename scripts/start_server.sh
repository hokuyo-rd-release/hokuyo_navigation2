#!/bin/bash

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash
source ~/.bashrc
roscd expo_wizurg 

gnome-terminal --tab -- bash -c "python3 /home/takahashi/github/expo_software/takahashi/expo_gui/server.py ; bash"
gnome-terminal --tab -- bash -c "roslaunch vizanti server.launch ; bash"
sleep 2
gnome-terminal --tab -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch ; bash"