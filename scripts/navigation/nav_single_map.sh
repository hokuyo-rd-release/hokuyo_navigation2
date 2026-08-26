#!/bin/bash

# スクリプトをより安全に実行するための設定
set -euo pipefail

# ROS 2 環境設定
set +u
source "$(dirname "$0")/../setup_ros_env.sh"
set -u

# 共通関数を読み込む
# (Zenoh ルーター管理・ライフサイクル待機の各関数はここで定義される)
source "$(dirname "$0")/nav_common.sh"

# --- 引数からナビゲーション設定を取得 ---
mapfile_arg="${1:-}" 
wayfile_arg="${2:-}"
nav_type_arg="${3:-}"

use_gnss_switch="false"
use_lio="true"

if [ "${nav_type_arg}" = "gnss" ]; then
    echo "ナビゲーションタイプ: GNSS"
    use_gnss_switch="true"
elif [ "${nav_type_arg}" = "loc" ]; then
    echo "ナビゲーションタイプ: LIO (Localization)"
    use_gnss_switch="false"
else
    echo "警告: 不明なナビゲーションタイプです: '${nav_type_arg}'。デフォルト設定を使用します。"
fi

cd ${HOKUYO_NAV2_PKG_PATH}

load_options "nav_opt_lio"

mapfile=${mapfile_arg:-${default_mapfile}}
wayfile=${wayfile_arg:-${default_wayfile}}

echo "--- 実行パラメータ ---"
echo "mapfile: ${mapfile}"
echo "wayfile: ${wayfile}"
echo "navigation: ${navigation}"
echo "use_gnss_switch: ${use_gnss_switch}"
echo "----------------------"

echo "既存のROSノードを終了します..."
gnome-terminal -- bash -c "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/kill_all_rosnode.sh"

while true; do
    echo "----------------------------------------------------"
    echo "マップの処理を開始します: ${mapfile}"
    echo "----------------------------------------------------"

    load_initial_poses "${mapfile}"
    launch_motor_driver
    launch_navigation_system "${mapfile}" "${use_gnss_switch}"

    # map_server の状態を確実に監視
    if ! wait_for_map_server; then
        echo "エラー: map_server の起動に失敗しました。再試行します..."
        "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/multi_map_kill.sh"
        sleep 5
        continue
    fi

    # Nav2全体の準備を監視
    if ! wait_for_nav2_ready; then
        echo "エラー: Nav2の起動に失敗しました。再試行します..."
        "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/multi_map_kill.sh"
        sleep 5
        continue
    fi

    echo "ウェイポイント追従を開始します: ${wayfile}.json"
    cd "${HOKUYO_NAV2_PKG_PATH}/waypoints"
    
    if ros2 run waypoint_manager waypoint_manager "${wayfile}.json" --ros-args -p use_gnss_switch:="${use_gnss_switch}" -p cmd_vel_topic:=wizurg/cmd_vel; then
        echo "waypoint_managerが正常に完了しました。"
        cd -
        break 
    else
        echo "エラー: waypoint_managerが異常終了しました。15秒後に再試行します..."
        cd -
        "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/multi_map_kill.sh"
        echo "ノードの終了を待っています..."
        for ((j=15; j>0; j--)); do
            echo -ne "再起動待機中: ${j} 秒...  \r"
            sleep 1
        done
        echo ""
    fi
done

echo "ターミナルウィンドウを最小化します..."
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done

echo "スクリプトの実行が完了しました。"