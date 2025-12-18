#!/bin/bash

# ROS 2 環境設定
SCRIPT_DIR=$(cd $(dirname $0); pwd)
source "${SCRIPT_DIR}/setup_ros_env.sh"

SELECTED_OPTION=$(zenity --list --title="Hokuyo Navigation2 Coordinator" --text="実行したい機能を選択してください" \
    --width=800 --height=400 \
    --print-column=1 --separator= \
    --column="番号" --column="機能" --column="説明" \
    1 "start_getting_rosbag" "センサーデータをROS Bagファイルとして記録します。" \
    2 "start_mapping" "ROS Bagから3D/2Dマップを作成します。" \
    3 "start_navigation" "作成したマップとウェイポイントを使用して自律走行します。" \
    4 "manual_control" "手動操縦用のノードを起動します。" \
    5 "rosbag_filter_gui" "ROS Bagフィルタリング用のGUIを起動します。" 2>/dev/null)
 
if [ -z "$SELECTED_OPTION" ]; then
  echo "終了します。>"
  exit 1
fi