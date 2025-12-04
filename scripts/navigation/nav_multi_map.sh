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

# CSVファイルの処理を無限に繰り返す
while true; do
    echo "=== CSVファイルの先頭からナビゲーションを開始します ==="
    # CSVファイルを1行ずつ読み込んでループ
    tail -n +2 "${csv_file_path}" | while IFS=',' read -r map_name waypoint_name nav_type interval || [ -n "$map_name" ]; do
        # Windowsの改行コード(CRLF)に対応するため、行末の\rを削除
        interval=$(echo "${interval}" | tr -d '\r')

        echo "----------------------------------------------------"
        echo "次のマップの処理を開始します: ${map_name}"
        echo "----------------------------------------------------"

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

        echo "--- 実行パラメータ ---"
        echo "mapfile: ${map_name}"
        echo "wayfile: ${waypoint_name}"
        echo "navigation: ${nav_type}"
        echo "interval: ${interval:-1} sec" # intervalが空ならデフォルト1秒
        echo "----------------------"

        # 初期位置情報を読み込む
        load_initial_poses "${map_name}"

        # 必要に応じて YP-Spur を起動
        start_ypspur_if_needed

        # ナビゲーションシステムを起動
        launch_navigation_system "${map_name}" "${current_use_gnss_switch}"

        echo "ウェイポイント追従を開始します: ${waypoint_name}.json"
        cd "${HOKUYO_NAV2_PKG_PATH}/waypoints"
        ros2 run waypoint_manager waypoint_manager "${waypoint_name}.json" --once --ros-args -p use_gnss_switch:="${current_use_gnss_switch}" -p cmd_vel_topic:=wizurg/cmd_vel
        cd -

        # 指定された秒数だけ待機
        wait_time=${interval:-1} # intervalが空または未設定の場合はデフォルト1秒
        echo "${wait_time}秒間待機します..."
        sleep "${wait_time}"

        echo "マップ ${map_name} の処理が完了しました。"
        echo "次のマップの準備のため、ROSノードを終了します..."
        # gnome-terminalを使わずに直接実行し、終了を待つ
        "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/multi_map_kill.sh"
        echo "ノードの終了を待っています..."
        sleep 15 # ノードが完全に終了するのを待つ
    done
    echo "=== CSVファイルの最後まで処理しました。ループを再開します。 ==="
    # sleep 3 # 次のループを開始する前に少し待機
done

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
echo "全てのナビゲーションが完了しました。ターミナルを最小化します。"
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done