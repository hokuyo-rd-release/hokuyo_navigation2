#!/bin/bash

# スクリプトをより安全に実行するための設定
# -e: コマンドがエラーになったら即座に終了
# -u: 未定義の変数を使おうとしたらエラーにする
# -o pipefail: パイプラインの途中でコマンドが失敗した場合もエラーにする
set -euo pipefail

# ROS 2 環境設定
set +u # AMENT_TRACE_SETUP_FILES 未定義エラーを回避
source "$(dirname "$0")/../setup_ros_env.sh"
set -u

# 共通関数を読み込む
source "$(dirname "$0")/nav_common.sh"

# --- 引数から設定を取得 ---
csv_file_arg="${1:-}"

cd ${HOKUYO_NAV2_PKG_PATH}

# --- メイン処理 ---

load_options "plural_opt_lio"

# --- CSVファイルのパスを決定 ---
if [ -n "$csv_file_arg" ]; then
    csv_file_path="$csv_file_arg"
else
    # デフォルトのCSVファイルパス
    csv_file_path="${HOKUYO_NAV2_PKG_PATH}/config/maps_and_waypoints.csv"
fi

if [ ! -f "${csv_file_path}" ]; then
    echo "エラー: マップリストファイルが見つかりません: ${csv_file_path}" >&2
    exit 1
fi

# 既存のROSノードをクリーンアップ
echo "既存のROSノードを終了します..."
gnome-terminal -- bash -c "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/kill_all_rosnode.sh"

start_ypspur_if_needed

# CSVファイルを1行ずつ読み込んでループ
tail -n +2 "${csv_file_path}" | while IFS=',' read -r map_name waypoint_name nav_type || [ -n "$map_name" ]; do
    echo "----------------------------------------------------"
    echo "次のマップの処理を開始します: ${map_name}"

    # ナビゲーションタイプに応じて use_gnss_switch を設定
    current_use_gnss_switch="false"
    if [ "${nav_type}" = "gnss" ]; then
        echo "ナビゲーションタイプ: GNSS"
        current_use_gnss_switch="true"
    elif [ "${nav_type}" = "loc" ]; then
        echo "ナビゲーションタイプ: LIO (Localization)"
    else
        echo "警告: 不明なナビゲーションタイプです: '${nav_type}'。デフォルト(loc)を使用します。"
    fi

    load_initial_poses "${map_name}"

    launch_navigation_system "${map_name}" "${current_use_gnss_switch}"

    echo "ウェイポイント追従を開始します: ${waypoint_name}.json"
    sleep 7.0s

    cd "${HOKUYO_NAV2_PKG_PATH}/waypoints"
    ros2 run waypoint_manager waypoint_manager "${waypoint_name}.json" --once --ros-args -p use_gnss_switch:="${current_use_gnss_switch}" -p cmd_vel_topic:=wizurg/cmd_vel
    cd -

    echo "マップ ${map_name} の処理が完了しました。"
    echo "次のマップの準備のため、ROSノードを終了します..."
    ros2 node list | grep -v 'gnome-terminal' | xargs ros2 node kill
    sleep 5 # ノードが完全に終了するのを待つ
done

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
echo "全てのナビゲーションが完了しました。ターミナルを最小化します。"
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done