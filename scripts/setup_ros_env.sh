#!/bin/bash
# --------------------------------------------------------------------------
# setup_ros_env.sh: ROS 2 環境設定を共通化するスクリプト
#
# このスクリプトは、呼び出し元のシェルのカレントディレクトリを変更せず、
# 必要な環境変数を設定します。
# --------------------------------------------------------------------------

# Docker環境かどうかを判定する
# コンテナの起動時に -e DOCKER_ENV=1 を指定することで、Docker環境とみなすことができます。
if [ -n "$DOCKER_ENV" ]; then
    # Docker環境
    ROS2_WS="/home/colcon_ws"
    source /opt/ros/humble/setup.bash
    source "${ROS2_WS}/install/setup.bash"
    source ~/.bashrc
    HOKUYO_NAV2_PKG_PATH="${ROS2_WS}/src/hokuyo_navigation2"
else
    # ホストOS環境
    ROS2_WS="${HOME}/colcon_ws"
    source /opt/ros/humble/setup.bash
    source "${ROS2_WS}/install/setup.bash"
    source ~/.bashrc
    HOKUYO_NAV2_PKG_PATH="${ROS2_WS}/src/hokuyo_navigation2"
fi

# 変数をエクスポートしてサブシェルでも利用可能にする
export ROS2_WS HOKUYO_NAV2_PKG_PATH