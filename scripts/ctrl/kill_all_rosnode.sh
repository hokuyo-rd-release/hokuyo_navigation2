#!/bin/bash

sleep 5
wmctrl -a TitleB -b add,above,sticky

while true; do
    # zenity で停止確認
    if zenity --question --title="ロボットプログラムの停止確認" --text="ロボットプログラムを停止しますか？" --width=800 2>/dev/null; then
        echo "OKが選択されました。rosbag record を停止します..."
        echo "Closing rosbag terminal!"
        pkill -f bag
        sleep 1s
        echo "Closing rsf process!"
        pkill -2 -f "hokuyo_rsf"
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
        echo "Stopping multi-map navigation script..."
        pkill -f "nav_multi_map.sh"
        sleep 1s
        echo "stopping single-map navigation script..."
        pkill -f "nav_single_map.sh"
        sleep 1s
        echo "kill all ros2 nodes!"
        ps aux | grep ros2 | grep -v grep | grep -v vizanti | grep -v rosapi | grep -v rosbridge_websocket | grep -v server | awk '{ print "kill -9", $2 }' | sh
        exit 0
    fi
done
