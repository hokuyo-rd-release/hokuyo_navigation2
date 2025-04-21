#!/bin/bash

source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash
source ~/.bashrc
roscd expo_wizurg

gnome-terminal --tab -- bash -c "python3 /home/hokuyo/github/expo_software/takahashi/expo_gui/server.py ; bash" &
sleep 1
gnome-terminal --tab -- bash -c "roslaunch vizanti server.launch ; bash" &
sleep 3
gnome-terminal --tab -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch ; bash" &
sleep 1

sleep 2

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done