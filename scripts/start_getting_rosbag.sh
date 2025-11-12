#!/bin/bash
# ROS 2 環境設定
source "$(dirname "$0")/setup_ros_env.sh"

# =====================（岡本→高橋）ここを引数にして使ってください==========================
use_gnss_switch="false" #-- コア技術でナビゲーションする場合は "true" にする. マップ作成・ウェイポイント作成などの場合は"false" --
stop_uam_manage="true" # ===========================================
cd ${HOKUYO_NAV2_PKG_PATH}

#-------kill_all_rosnode起動--------------
gnome-terminal -- bash -c "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/kill_all_rosnode.sh"

#-------ypspur-coordinator起動------------
echo "ypspur-coordinatorを起動します..."
gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 icart_mini_drive_launch.xml; bash"

echo "ypspur関連ノードの起動を待っています..."
local timeout=15
local start_time=$(date +%s)
local ypspur_node_found=false

while [ $(($(date +%s) - start_time)) -lt ${timeout} ]; do
    if ros2 node list | grep -q -e 'icart_mini' -e 'ypspur'; then
        ypspur_node_found=true
        break
    fi
    sleep 1
done

if [ "${ypspur_node_found}" = "false" ]; then
    echo "エラー: ypspur関連ノードの起動に失敗しました。(${timeout}秒タイムアウト)" >&2
    exit 1
fi
echo "ypspur関連ノードの起動を確認しました。"
sleep 1
#============= wizurg_satrt.launch起動 =============
#gnome-terminal -- bash -c "roscore" # 25/1/16 岡本追記
#sleep 1
#-------------------------------------
gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml use_localization:="false" use_joy:=use_mapping:="false" use_navigation:="false" use_loader:="false" use_editor:="false" use_sensor:="true" use_icart:="true"  use_lio:="true" use_gnss_switch:=${use_gnss_switch} stop_uam_manage:=${stop_uam_manage} ;bash"
sleep 1

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done