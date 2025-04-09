#!/bin/bash

sleep 5
wmctrl -a TitleB -b add,above,sticky

while true; do
    # zenity で停止確認
    if zenity --question --title="ロボットプログラムの停止確認" --text="ロボットプログラムを停止しますか？" --width=800 2>/dev/null; then
        echo "OKが選択されました。rosbag record を停止します..."
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

        echo "Closing Navigation terminal!"
        pkill -f wizurg_navigation
        sleep 1s

        echo "Closing Waypoint terminal!"
        pkill -f wizurg_waypoint
        sleep 1s
        # ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
        exit 0
    fi
done