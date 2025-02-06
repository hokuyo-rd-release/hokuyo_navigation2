#!/bin/bash

mapfile=ATC_1;
wayfile=ATC_1;

#-------オプション入力情報を格納----------
while getopts :M:W: OPTN; do
  case $OPTN in
     M) mapfile=$OPTARG;;
     W) wayfile=$OPTARG;;
     :) echo "$OPTARGに引数が指定されていません";;
     ?) echo "$OPTARGは定義されていません";;
  esac
done

echo "mapfile=$mapfile";
echo "wayfile=$wayfile";

#-----------------------------------------
cd ~/catkin_ws && source devel/setup.bash && cd -

gnome-terminal -- roslaunch wizurg wizurg_start.launch use_loader:=true use_sensor:=false use_driver:=false map_file:=${mapfile}
sleep 3

cd ~/catkin_ws/src/wizurg_ros1/waypoints
rosrun wizurg wizurg_waypoint_editor.py ${wayfile}.json
