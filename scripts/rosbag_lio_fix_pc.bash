#!/bin/bash

#------- 環境変数の確認 -------
# ros1 echo ros workspace: ${ROS_WORKSPACE:?ROS_WORKSPACE is Undefined}

#------- 引数の確認 -------
# 第1引数
if [ -z "$1" ]; then
  echo "Error: 引数が不足しています <arg1>"
  exit 1
fi
# 第2引数
if [ -z "$2" ]; then
  echo "Error: 引数が不足しています <arg2>"
  exit 1
fi

#------- カレントディレクトリの取得 -------
CURRENT=$HOME/colcon_ws/src/hokuyo_navigation2
echo current dir: $CURRENT
rosbag_dir=$CURRENT/rosbag;
echo rosbag dir: $rosbag_dir
echo "All args are checked."

# ファイルが存在するかチェック
DIR="$1" # $1 はディレクトリパスとして扱う
if [ ! -d "$DIR" ]; then
  echo "Error: directory $DIR does not exist."
  exit 1
else
  echo "rosbag directory $DIR exists."
fi

#------- config.csv 読み込み -------
if [ "$3" = "" ]; then
  # ヘッダー行をスキップするために tail -n +2 を追加
  options=(`cat config/config.csv | tail -n +2`)
  echo "option (from config/config.csv): ${options[@]}"
else
  options=(`cat "$3" | tail -n +2`)
  echo "option (from $3): ${options[@]}"
fi

for i in ${!options[@]}; do
 if [ $i -ge 0 ]; then # 0以上のインデックスで処理
  option_arr[$i]=`echo ${options[$i]} | cut -d ',' -f 2`
  fi
done

gnss_topic="${option_arr[0]}";
pointcloud_topic="${option_arr[1]}";
lio_topic="${option_arr[2]}";
run_lio="${option_arr[3]}";

sleep 3

source /opt/ros/$ROS_DISTRO/setup.bash
source /home/colcon_ws/install/setup.bash

#gnome-terminal --tab -t "Tab 0" -- bash -c "roscore; bash"
#sleep 2
if [ "x${run_lio}" = "xtrue" ]; then
 gnome-terminal --tab -t "hokuyo_lio" -- bash -c "ros2 launch hokuyo_navigation2 hokuyo_lio_node_with_yaml_ros2.xml sync_enable:=true; bash"
fi
gnome-terminal --tab -t "ros2 bag play" -- bash -c "cd ${CURRENT}; ros2 bag play \"$1\"; bash"

# --- 修正箇所: ros2 bag info の duration を確実に整数に変換 ---
# rosbagディレクトリ内の .db3 ファイルを検索し、そのファイルパスを使用
BAG_FILE=$(find "$1" -name "*.db3" | head -n 1) # $1 はディレクトリパスと想定

if [ -z "$BAG_FILE" ]; then
    echo "エラー: 指定されたディレクトリ '$1' 内に .db3 ファイルが見つかりません。"
    exit 1
fi

echo "解析対象の ROS 2 bag ファイル: $BAG_FILE"

# ros2 bag info から Duration の秒数（小数点以下を含む）を抽出
# grep で Duration の行を取得し、awk で括弧内の数値を抽出します
# awk '{ gsub(/[()s]/, "", $2); print $2 }' は '(123.456s)' から '123.456' を抽出します
DURATION_RAW=$(ros2 bag info "$BAG_FILE" | grep "Duration:" | awk '{ gsub(/[()s]/, "", $2); print $2 }' | head -n 1)

# 抽出した秒数（浮動小数点数）を整数に変換（小数点以下切り捨て）
# bash の組み込み算術計算では浮動小数点数を扱えないため、bc コマンドを使用します。
# `scale=0` で小数点以下を0桁に設定し、`int()` で整数部分を取得します。
if [ -z "$DURATION_RAW" ]; then
    echo "エラー: ros2 bag の duration を取得できませんでした。ファイルが有効な ROS 2 bag であるか確認してください。"
    exit 1
fi

BAG_DURATION=$(echo "scale=0; ${DURATION_RAW}/1" | bc)

echo "元の ROS 2 bag の Duration (整数化): ${BAG_DURATION}秒"

# 録画時間から10秒を引く
RECORD_DURATION=$((BAG_DURATION - 10))

# 録画時間が0以下にならないようにチェック
if [ "$RECORD_DURATION" -le 0 ]; then
    echo "警告: 計算された録画時間 ($RECORD_DURATION 秒) が0以下です。ros2 bag record は実行されません。"
    echo "元のrosbagのdurationが10秒以下の場合は、この警告が表示されます。"
    exit 0
fi
#gnome-terminal --tab -t "ros2 bag record" -- bash -c "cd ${CURRENT}; echo 'Recording for ${RECORD_DURATION} seconds'; timeout ${RECORD_DURATION} ros2 bag record -o \"$2\" $gnss_topic $pointcloud_topic $lio_topic; bash"
gnome-terminal --tab -t "ros2 bag record" -- bash -c "$CURRENT/scripts/rosbag_record_with_timeout.bash $2 ${rosbag_dir} ${RECORD_DURATION} ${gnss_topic} ${pointcloud_topic} ${lio_topic}; bash"
gnome-terminal --tab -t "progress bar" -- bash -c "$CURRENT/scripts/progress_bar.bash ${BAG_DURATION}; bash"