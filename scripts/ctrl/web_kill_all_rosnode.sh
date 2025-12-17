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
pkill -f "kill_all_rosnode.sh"
sleep 1s
echo "--- All navigation processes have been requested to terminate. ---"
exit 0