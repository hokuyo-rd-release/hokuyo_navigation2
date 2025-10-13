#!/bin/bash

# $1: rosbagファイル名 (例: my_synced_bag)
# $2: rosbagディレクトリ (例: /home/colcon_ws/src/hokuyo_navigation2/rosbag)
# $3: recordする時間
# $4: gnss_topic
# $5: pointcloud_topic
# $6: lio_topic

cd $2
sleep 3
echo "Recording for $3 seconds"
sleep 3
ros2 bag record -o $1 $4 $5 $6 &
BAG_PID=$!
echo "ROS Bag Record PID: $BAG_PID"

# 指定された時間待機
sleep $3

# 録画プロセスを終了
echo "Stopping ROS Bag Record process $BAG_PID..."
kill ${BAG_PID}

# 🚨 完了フラグファイルの作成 🚨
# ファイル名は出力ROS Bag名と関連付け、rosbagディレクトリ内に配置
COMPLETION_FLAG_FILE="$2/$1.SYNC_DONE"
echo "Creating completion flag file: $COMPLETION_FLAG_FILE"
# $2 は rosbag_dir であり、DOWNLOAD_FOLDER と同じパスを指すことを想定
touch "$COMPLETION_FLAG_FILE"

cd -
exit 0