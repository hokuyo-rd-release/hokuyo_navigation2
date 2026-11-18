#!/bin/bash

# このスクリプトは、Web UIなどから呼び出され、
# ナビゲーション関連のすべてのプロセスを緊急停止することを目的としています。

echo "--- Emergency Stop initiated by Web Interface ---"

echo "Stopping multi-map navigation script..."
pkill -f "nav_multi_map.sh"
sleep 1s

echo "stopping single-map navigation script..."
pkill -f "nav_single_map.sh"
sleep 1s

echo "Stopping rosbag play or record..."
pkill -f "bag"
sleep 1s

echo "Closing rsf process!"
pkill -2 -f "hokuyo_rsf"

echo "Closing bringup launch process!"
pkill -f "hokuyo_nav2_bringup_launch.xml"
sleep 1s

echo "Closing motor_drive process!"
pkill -f "icart_mini_drive_launch.xml"
sleep 1s

echo "Closing Navigation & Waypoint process!"
pkill -f "waypoint_manager"
sleep 1s
echo "kill all ros2 nodes!"
# "kill_all_rosnode.sh" だとこのスクリプト自身 (web_kill_all_rosnode.sh) にも一致して止まるので、ディレクトリ名から指定する
pkill -f "ctrl/kill_all_rosnode.sh"
sleep 1s

# 終了したノードが残した Fast DDS の共有メモリを掃除する (使用中のものは消さない)。
# ros2 launch は子ノードを SIGINT から最大 10 秒かけて止めるので、少し待ってから実行する
echo "Cleaning stale shared memory..."
sleep 5s
"/opt/ros/${ROS_DISTRO:-humble}/bin/fastdds" shm clean || echo "Warning: fastdds shm clean failed."
echo "--- All navigation processes have been requested to terminate. ---"
exit 0