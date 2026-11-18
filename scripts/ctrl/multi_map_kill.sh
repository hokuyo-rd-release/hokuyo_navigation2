#!/bin/bash

# マップ切替・再試行のためにナビゲーション関連のプロセスへ終了シグナルを送る。
# 終了の完了はここでは待たない。呼び出し側で nav_common.sh の wait_for_nodes_exit を使って確認する。

echo "Stopping rosbag play or record..."
echo "Closing rosbag process!"
pkill -f "bag"
echo "Closing rsf process!"
pkill -2 -f "hokuyo_rsf"
echo "Closing bringup launch process!"
pkill -f "hokuyo_nav2_bringup_launch.xml"
echo "Closing motor_drive process!"
pkill -f "icart_mini_drive_launch.xml"
echo "Closing Navigation process!"
pkill -f "waypoint_manager"
echo "Stopping kill_all_rosnode.sh script..."
pkill -f "kill_all_rosnode.sh"
#ps aux | grep ros2 | grep -v grep | grep -v vizanti | grep -v rosapi | grep -v rosbridge_websocket | grep -v server | awk '{ print "kill -9", $2 }' | sh
exit 0
