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

# Unity では前回の走行の LIO 座標が残っていることがあるので、起動前にリセットする
reset_unity_sim

# 前回の走行が異常終了して残った共有メモリを掃除する
clean_stale_shm

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


# 特定のマップの処理を成功するまで繰り返すループ
while true; do
    echo "----------------------------------------------------"
    echo "マップの処理を開始します: ${mapfile}"
    echo "----------------------------------------------------"

    # 初期位置情報を読み込み
    load_initial_poses "${mapfile}"

    # モータドライバを起動
    launch_motor_driver

    # ナビゲーションシステムを起動
    launch_navigation_system "${mapfile}" "${use_gnss_switch}"

    echo "ウェイポイント追従を開始します: ${wayfile}.json"
    cd "${HOKUYO_NAV2_PKG_PATH}/waypoints"
    # waypoint_managerの実行とエラーハンドリング
    if ros2 run waypoint_manager waypoint_manager "${wayfile}.json" --ros-args -p use_gnss_switch:="${use_gnss_switch}" -p cmd_vel_topic:=wizurg/cmd_vel; then
        echo "waypoint_managerが正常に完了しました。"
        cd -
        break # 成功したのでリトライループを抜ける
    else
        echo "エラー: waypoint_managerが異常終了しました。ノードの終了後に再試行します..."
        cd -
        # 関連ノードを全て終了し、終了を確認する
        shutdown_nodes_for_next_map
    fi
done

# 起動したターミナルを最小化
echo "ターミナルウィンドウを最小化します..."
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done

echo "スクリプトの実行が完了しました。"