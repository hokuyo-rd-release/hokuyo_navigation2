#!/bin/bash

# $1: rosbagファイル名
# $2: rosbagディレクトリ
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
sleep $3
kill ${BAG_PID}
cd -
