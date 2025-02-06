#!/bin/bash

mapfile="ATC_1";
wayfile="ATC_1";
bagfile="ATC_1";
bagdir="~/catkin_ws/src/wizurg_ros1/rosbag";

#-------オプション入力情報を格納----------
while getopts :M:W:B OPTN; do
  case $OPTN in
     M) mapfile=$OPTARG;;
     W) wayfile=$OPTARG;;
     B) bagfile=$OPTARG;;
     :) echo "$OPTARGに引数が指定されていません";;
     ?) echo "$OPTARGは定義されていません";;
  esac
done

echo "mapfile=$mapfile";
echo "wayfile=$wayfile";
echo "bagdir=$bagdir";
echo "bagfile=$bagfile";

#-----------------------------------------
cd ~/catkin_ws && source devel/setup.bash && cd -

gnome-terminal -- roslaunch wizurg waypoint_maker.launch map_file:=${mapfile} way_file:=${wayfile}

#cd ~/catkin_ws/src/wizurg/waypoints
#gnome-terminal -- rosrun wizurg wizurg_waypoint_maker.py ${wayfile}.json


dummy=0
echo "rosbagスタート(enter)"
read dummy

cd "${bag_dir}"
gnome-terminal -- rosbag play ${bagfile}.bag --clock -r 2

