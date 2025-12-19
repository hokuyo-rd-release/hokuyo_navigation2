#!/bin/bash

# スクリプトをより安全に実行するための設定
set -euo pipefail

# ROS 2 環境設定
set +u # AMENT_TRACE_SETUP_FILES 未定義エラーを回避
source "$(dirname "$0")/setup_ros_env.sh"
set -u

# 共通関数を読み込む
source "$(dirname "$0")/navigation/nav_common.sh"

# --- 固定値 ---
use_gnss_switch="false"
cd ${HOKUYO_NAV2_PKG_PATH}

# --- メイン処理 ---

# rosbag取得用の設定を読み込み
load_rosbag_options

echo "--- 実行パラメータ ---"
echo "use_motor_driver: ${use_motor_driver}"
echo "use_navigation: ${use_navigation}"
echo "use_sensor: ${use_sensor}"
echo "use_lio: ${use_lio}"
echo "use_gnss_switch: ${use_gnss_switch}"
echo "use_localization: ${use_localization}"
echo "----------------------"

# 既存のROSノードをクリーンアップ
echo "既存のROSノードを終了します..."
gnome-terminal -- bash -c "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/kill_all_rosnode.sh"

# モータドライバを起動
launch_motor_driver

# センサーとrosbag取得のためのシステムを起動
# launch_navigation_system "" "${use_gnss_switch}"
gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml use_navigation:=${use_navigation} use_sensor:=${use_sensor} use_lio:=${use_lio} use_localization:=${use_localization} use_gnss_switch:=${use_gnss_switch};bash"
sleep 1

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done
