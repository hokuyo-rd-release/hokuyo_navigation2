#!/bin/bash

# ROS 2 環境設定
source "$(dirname "$0")/setup_ros_env.sh"

# export ROS_MASTER_URI=http://`hostname -I | cut -d' ' -f1`:11311
# export ROS_IP=`hostname -I | cut -d' ' -f1`

gnome-terminal --tab -- bash -c "python3 ${ROS2_WS}/src/hokuyo_navigation2/hokuyo_navigation2_gui/server.py ; bash" &
sleep 1
gnome-terminal --tab -- bash -c "ros2 launch vizanti_server vizanti_server.launch.py ; bash" &
sleep 3

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done
