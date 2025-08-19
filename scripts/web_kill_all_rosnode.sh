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
ps aux | grep ros | grep -v grep | grep -v vizanti | awk '{ print "kill -9", $2 }' | sh
exit 0