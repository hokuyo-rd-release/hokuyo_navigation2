#!/bin/bash
echo "Kill all ros nodes!"
echo "Closing rosbag terminal!"
pkill -f rosbag
sleep 1s
echo "Closing launch terminal!"
pkill -f wizurg_start
sleep 1s
echo "Closing roscore terminal!"
pkill -f roscore
sleep 1s
echo "Closing ypspur terminal!"
pkill -f ypspur
sleep 1s
#ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
exit 0
