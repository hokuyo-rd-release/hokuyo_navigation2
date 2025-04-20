#!/bin/bash

pkill -f vizanti
echo "Closing websocket!"
pkill -f websocket
sleep 0.5s
echo "Closing flask!"
pkill -f server.py
sleep 0.5s
echo "Closing roscore terminal!"
pkill -f roscore
sleep 0.5s

echo "rosnode kill all!"
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh

sleep 0.5s
echo "server restart!"


source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash
source ~/.bashrc
roscd expo_wizurg

gnome-terminal --tab -- bash -c "python3 /home/takahashi/github/expo_software/takahashi/expo_gui/server.py ; bash" &
sleep 0.5
gnome-terminal --tab -- bash -c "roslaunch vizanti server.launch ; bash" &
sleep 0.5
gnome-terminal --tab -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch ; bash" &
sleep 0.5

sleep 0.5

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done
