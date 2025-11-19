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
# --- 引数からナビゲーション設定を取得 ---
mapfile_arg="${1:-}" # 引数がなくてもエラーにならないようにデフォルト値を設定
wayfile_arg="${2:-}"
nav_type_arg="${3:-}"

# --- 固定値 ---
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

# --- メイン処理 ---

load_options "nav_opt_lio"

# 引数で渡されたマップ/ウェイポイントファイルがなければCSVのデフォルト値を使用
mapfile=${mapfile_arg:-${default_mapfile}}
wayfile=${wayfile_arg:-${default_wayfile}}

echo "--- 実行パラメータ ---"
echo "mapfile: ${mapfile}"
echo "wayfile: ${wayfile}"
echo "navigation: ${navigation}"
echo "use_gnss_switch: ${use_gnss_switch}"
echo "----------------------"

# ROSノードをクリーンアップ
echo "既存のROSノードを終了します..."
gnome-terminal -- bash -c "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/kill_all_rosnode.sh"


# 初期位置情報を読み込み
load_initial_poses "${mapfile}"

# 必要に応じて YP-Spur ノードを起動
start_ypspur_if_needed

# SPELシステムを起動
start_spel_system
sleep 5

# ナビゲーションシステムを起動
launch_navigation_system "${mapfile}" "${use_gnss_switch}"

echo "ウェイポイント追従を開始します: ${wayfile}.json"
gnome-terminal -- bash -c "ros2 run waypoint_manager waypoint_manager ${HOKUYO_NAV2_PKG_PATH}/waypoints/${wayfile}.json --ros-args -p use_gnss_switch:=${use_gnss_switch} -p cmd_vel_topic:=wizurg/cmd_vel"

# 起動したターミナルを最小化
echo "ターミナルウィンドウを最小化します..."
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done

echo "スクリプトの実行が完了しました。"