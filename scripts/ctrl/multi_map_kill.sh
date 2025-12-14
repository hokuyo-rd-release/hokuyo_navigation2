#!/bin/bash

echo "Stopping rosbag play or record..."
echo "Closing rosbag process!"
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
echo "Closing Navigation process!"
pkill -f "waypoint_manager"
sleep 1s
echo "Closing Waypoint process!"
pkill -f "waypoint_manager"
sleep 1s
echo "Stopping kill_all_rosnode.sh script..."
pkill -f "kill_all_rosnode.sh"
#ps aux | grep ros2 | grep -v grep | grep -v vizanti | grep -v rosapi | grep -v rosbridge_websocket | grep -v server | awk '{ print "kill -9", $2 }' | sh

echo "Clearing cache memory..."
# sudoersでパスワードなし実行が許可されたキャッシュ解放スクリプトを呼び出します
sudo /usr/local/bin/clear_memory_cache.sh
echo "Cache memory has been cleared."
exit 0