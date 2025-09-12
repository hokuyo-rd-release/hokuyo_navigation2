#!/bin/bash

echo "OKが選択されました。rosbag record を停止します..."
echo "Closing rosbag terminal!"
pkill -f bag
sleep 1s
echo "Closing launch terminal!"
pkill -f bringup
sleep 1s
echo "Closing motor_drive terminal!"
pkill -f icart_mini
sleep 1s
echo "Closing Navigation terminal!"
pkill -f waypoint
sleep 1s
echo "Closing Waypoint terminal!"
pkill -f waypoint
sleep 1s
echo "kill all ros2 nodes!"
pkill -f kill_all_rosnode.sh
#ps aux | grep ros2 | grep -v grep | grep -v vizanti | grep -v rosapi | grep -v rosbridge_websocket | grep -v server | awk '{ print "kill -9", $2 }' | sh
exit 0