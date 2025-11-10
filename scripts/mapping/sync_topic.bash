#!/bin/bash

# $1: 入力rosbagディレクトリ
# $2: 出力rosbag名
# $3: configファイルパス (任意)

# --- プロセス終了時の後処理 ---
# スクリプト終了時にバックグラウンドプロセスをすべて終了させる
pids=()
cleanup() {
    echo "Cleaning up background processes..."
    for pid in "${pids[@]}"; do
        # プロセスが存在するか確認してからkill
        if kill -0 "$pid" 2>/dev/null; then
            echo "Killing process $pid"
            kill "$pid"
        fi
    done
    # Zenityが起動している場合も終了させる
    pkill zenity
}
trap cleanup EXIT

#------- ROS 2 環境設定 -------
source "$(dirname "$0")/../setup_ros_env.sh"
echo "ROS2_WS: $ROS2_WS"
echo "HOKUYO_NAV2_PKG_PATH: $HOKUYO_NAV2_PKG_PATH"

#------- 引数の確認 -------
if [ -z "$1" ]; then
  echo "Error: 引数が不足しています <入力rosbagディレクトリ>"
  exit 1
fi
if [ -z "$2" ]; then
  echo "Error: 引数が不足しています <出力rosbag名>"
  exit 1
fi

# ファイルが存在するかチェック
INPUT_BAG_DIR="$1"
if [ ! -d "$INPUT_BAG_DIR" ]; then
  echo "Error: directory: $INPUT_BAG_DIR does not exist."
  exit 1
else
  echo "Input rosbag directory: $INPUT_BAG_DIR exists."
fi

#------- config.csv 読み込み -------
CONFIG_FILE="$3"
if [ -z "$CONFIG_FILE" ]; then
  CONFIG_FILE="${HOKUYO_NAV2_PKG_PATH}/config/config.csv"
  echo "Using default config file: $CONFIG_FILE"
else
  echo "Using specified config file: $CONFIG_FILE"
fi

if [ ! -f "$CONFIG_FILE" ]; then
    echo "Error: Config file not found at $CONFIG_FILE"
    exit 1
fi

# ヘッダー行をスキップするために tail -n +2 を使用
options=(`cat "$CONFIG_FILE" | tail -n +2`)
echo "options (from config): ${options[@]}"

for i in ${!options[@]}; do
  option_arr[$i]=`echo ${options[$i]} | cut -d ',' -f 2`
done

gnss_topic="${option_arr[0]}"
pointcloud_topic="${option_arr[1]}"
lio_topic="${option_arr[2]}"
run_lio="${option_arr[3]}"

echo "gnss_topic: $gnss_topic"
echo "pointcloud_topic: $pointcloud_topic"
echo "lio_topic: $lio_topic"
echo "run_lio: $run_lio"

sleep 3

#------- LIOの起動 -------
if [ "x${run_lio}" = "xtrue" ]; then
 echo "Launching hokuyo_lio..."
 ros2 launch hokuyo_navigation2 hokuyo_lio_node_with_yaml_ros2.xml sync_enable:=true &
 pids+=($!)
fi

sleep 2

#------- ROSBAGの再生時間取得 -------
BAG_FILE=$(find "$INPUT_BAG_DIR" -name "*.db3" | head -n 1)
if [ -z "$BAG_FILE" ]; then
    echo "エラー: 指定されたディレクトリ '$INPUT_BAG_DIR' 内に .db3 ファイルが見つかりません。"
    exit 1
fi
echo "解析対象の ROS 2 bag ファイル: $BAG_FILE"

DURATION_RAW=$(ros2 bag info "$BAG_FILE" | grep "Duration:" | awk '{ gsub(/[()s]/, "", $2); print $2 }' | head -n 1)
if [ -z "$DURATION_RAW" ]; then
    echo "エラー: ros2 bag の duration を取得できませんでした。ファイルが有効な ROS 2 bag であるか確認してください。"
    exit 1
fi

BAG_DURATION=$(echo "scale=0; ${DURATION_RAW}/1" | bc)
echo "元の ROS 2 bag の Duration (整数化): ${BAG_DURATION}秒"

# 録画時間から10秒を引く
RECORD_DURATION=$((BAG_DURATION - 10))
if [ "$RECORD_DURATION" -le 0 ]; then
    echo "警告: 計算された録画時間 ($RECORD_DURATION 秒) が0以下です。ros2 bag record は実行されません。"
    echo "元のrosbagのdurationが10秒以下の場合は、この警告が表示されます。"
    exit 0
fi

#------- プログレスバー表示 -------
progress_bar() {
    TOTAL_SECONDS=$1
    (
        START_TIME=$(date +%s)
        END_TIME=$((START_TIME + TOTAL_SECONDS))
        CURRENT_TIME=$(date +%s)
        while [ "$CURRENT_TIME" -le "$END_TIME" ]; do
            ELAPSED_TIME=$((CURRENT_TIME - START_TIME))
            REMAINING_SECONDS=$((TOTAL_SECONDS - ELAPSED_TIME))
            PERCENTAGE=$(echo "scale=2; ($ELAPSED_TIME * 100) / $TOTAL_SECONDS" | bc | cut -d'.' -f1)
            echo "#rosbag処理中。残り時間: ${REMAINING_SECONDS} 秒"
            echo "$PERCENTAGE"
            sleep 1
            CURRENT_TIME=$(date +%s)
        done
        echo "#rosbag処理中。残り時間: 0 秒"
        echo "100"
    ) | zenity --progress --title="Rosbag Processing" --auto-close &
}

echo "Starting progress bar for ${BAG_DURATION} seconds."
progress_bar ${BAG_DURATION}

#------- ROSBAGの記録 -------
rosbag_dir="$HOKUYO_NAV2_PKG_PATH/rosbag"
OUTPUT_BAG_NAME="$2"

echo "Recording for ${RECORD_DURATION} seconds..."
echo "Output bag will be saved in: $rosbag_dir/$OUTPUT_BAG_NAME"

cd "$rosbag_dir"
ros2 bag record -o "$OUTPUT_BAG_NAME" $gnss_topic $pointcloud_topic $lio_topic /hokuyo3d/imu /hokuyo_lio/lidar_odom /tf /gga &
pids+=($!)

#------- ROSBAGの再生 -------
echo "Playing rosbag: $INPUT_BAG_DIR"
ros2 bag play "$INPUT_BAG_DIR" &
pids+=($!)

#------- 処理の待機と完了 -------
echo "Waiting for ${RECORD_DURATION} seconds to record..."
sleep ${RECORD_DURATION}

echo "Recording finished. Cleaning up processes."

# cleanup関数がtrapによって自動で呼ばれる

# 完了フラグファイルの作成
COMPLETION_FLAG_FILE="$rosbag_dir/$OUTPUT_BAG_NAME.SYNC_DONE"
echo "Creating completion flag file: $COMPLETION_FLAG_FILE"
touch "$COMPLETION_FLAG_FILE"

echo "Processing complete."
exit 0