#!/bin/bash
# Docker環境かどうかを判定する
# コンテナの起動時に -e DOCKER_ENV=1 を指定することで、Docker環境とみなすことができます。
if [ -n "$DOCKER_ENV" ]; then
    source /opt/ros/humble/setup.bash
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${HOME}/colcon_ws/src/hokuyo_navigation2"
else
    source /opt/ros/humble/setup.bash
    # ワークスペースのパスもホストOSのものに合わせる
    # 実際のホストOSのワークスペースパスに置き換えてください
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${HOME}/colcon_ws/src/hokuyo_navigation2"
fi

# export ROS_MASTER_URI=http://`hostname -I | cut -d' ' -f1`:11311
# export ROS_IP=`hostname -I | cut -d' ' -f1`
cd ${ROS2_WS}

gnome-terminal --tab -- bash -c "source install/setup.bash && source ~/.bashrc && python3 ${ROS2_WS}/../github/hokuyo_navigation2_gui/server.py ; bash" &
sleep 1
gnome-terminal --tab -- bash -c "ros2 launch vizanti_server vizanti_server.launch.py ; bash" &
sleep 3

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done
