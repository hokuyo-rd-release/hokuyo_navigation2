#!/bin/bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
source devel/setup.bash

roscd expo_wizurg

#--2024/10/30 Mapping と Navigation を分けるように追記--

wizurg_opt="wizurg_opt";                        #-- wizurg_optを追記 --

while getopts MNWEBCPL OPTN; do
  case $OPTN in
     M) wizurg_opt="map_opt";;                  #-- -M で map_opt.csv が入る --
     N) wizurg_opt="nav_opt";;                  #-- -N で nav_opt.csv が入る --
     W) wizurg_opt="way_opt";;                  #-- -W で way_opt.csv が入る (岡本11/6追記) --
     E) wizurg_opt="edit_opt";;                 #-- -E で edit_opt.csv が入る (岡本11/6追記)--
     B) wizurg_opt="sensor_rosbag";;            #-- -B で sensor_rosbag.csv が入る (高橋11/6追記)--
     C) wizurg_opt="control_opt";;              #-- -C で control_opt.csv が入る (岡本11/10追記)--
     P) wizurg_opt="plural_opt";;               #-- -P で plural_opt.csv が入る (岡本11/10追記)--
     L) wizurg_opt="${wizurg_opt}_lio";;        #-- -L で 〇〇_lio.csv が入る（岡本11/12追記）--
     :) echo "$OPTARGに引数が指定されていません";;
     ?) echo "$OPTARGは定義されていません";;
  esac
done

echo "wizurg_opt=${wizurg_opt}";

options=(`cat ./config/wizurg_opts/99_${wizurg_opt}.csv`)   #-- params/wizurg_opts/を新規作成（岡本11/12追記）--

#--2024/10/30 追記ここまで--

map_names=(`cat ./config/maps_and_waypoints.csv`) 
Rmapfile=()
R3Dmapfile=()
Rwayfile=()


#-------オプション入力情報を格納----------
#-------maps_and_waypoints の配列のデータを読む。------#
for i in ${!options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  option_arr[$j]=`echo ${options[$i]} | cut -d ',' -f 2`
  fi
done

#------複数マップ名、ウェイポイントファイル名の読み込み------
#------上の結果のデータを読む。(True) データがなければ空読みする。(False) ------
for i in ${!map_names[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  Rmapfile[$j]=`echo ${map_names[$i]} | cut -d ',' -f 1`
  R3Dmapfile[$j]=`echo ${map_names[$i]} | cut -d ',' -f 2`
  Rwayfile[$j]=`echo ${map_names[$i]} | cut -d ',' -f 3`
 fi
done

use_joy="${option_arr[0]}";
mapping="${option_arr[1]}";
navigation="${option_arr[2]}";
sensor="${option_arr[3]}";
icart="${option_arr[4]}";
use_lio="${option_arr[5]}";
use_unity="${option_arr[6]}";
ypspur="${option_arr[7]}";
multi_map="${option_arr[8]}";
mapfile="${option_arr[9]}";
wayfile="${option_arr[10]}";
rosbag_record="${option_arr[11]}";
rosbag_dir="${option_arr[12]}";
loader="${option_arr[13]}";
editor="${option_arr[14]}";


#-------ypspur-coordinator起動------------
if [ "x${ypspur}" = "xtrue" ]; then
 gnome-terminal -- bash -c "/usr/local/bin/ypspur-coordinator -d /dev/ttyUSB0 --blvr -p ~/catkin_ws/src/expo_wizurg/config/icart/iCart3_100W.param ; bash"
 #---------spur待機---------------
 sleep 1
fi


#============= wizurg_satrt.launch起動 =============
gnome-terminal -- bash -c "roscore" # 25/1/16 岡本追記
sleep 1

#----複数マップ----
if [ "x${multi_map}" = "xtrue" ]; then
 for i in ${!Rmapfile[@]}; do
  echo "kill all_nodes"
  rosnode kill -a
  sleep 8s
  echo "sleep 8"
  if [ "x${rosbag_record}" = "xtrue" ]; then
    echo "rosbag record"
    gnome-terminal -- bash -c "sleep 2; cd ${rosbag_dir}; rosbag record -a -o ${Rmapfile[$i]}; bash"
  fi
  gnome-terminal -- bash -c "roslaunch expo_wizurg expo_wizurg_start.launch use_joy:=${use_joy} use_mapping:=${mapping} use_navigation:=${navigation} use_loader:=${loader} use_editor:=${editor} use_sensor:=${sensor} use_icart:=${icart} use_lio:=${use_lio} use_unity_sim:=${use_unity} map_file:=${Rmapfile[$i]} 3dmap_file:=${R3Dmapfile[$i]} ;bash"
  sleep 2s
  echo "sleep 2"
  echo "start wizurg_navigation ${Rwayfile[$i]}"
  cd ~/catkin_ws/src/expo_wizurg/waypoints; rosrun expo_wizurg wizurg_navigation.py ${Rwayfile[$i]}.json once
  echo "finish map"
  cd -
 done
#----単一マップ---
else
  # ------rosbag record----------------
  if [ "x${rosbag_record}" = "xtrue" ]; then
     echo "rosbag record"
     gnome-terminal -- bash -c "sleep 2; cd ${rosbag_dir}; rosbag record -a -o ${mapfile}; bash"
  fi
#-------------------------------------
 gnome-terminal -- bash -c "roslaunch expo_wizurg expo_wizurg_start.launch use_joy:=${use_joy} use_mapping:=${mapping} use_navigation:=${navigation} use_loader:=${loader} use_editor:=${editor} use_sensor:=${sensor} use_icart:=${icart}  use_lio:=${use_lio} use_unity_sim:=${use_unity} use_sensor:=${sensor} use_icart:=${icart} map_file:=${mapfile} 3dmap_file:=${map3dfile} ;bash"
 sleep 1
 if [ "x${loader}" = "xtrue" ]; then
    cd ~/catkin_ws/src/expo_wizurg/waypoints
    # ----waypoint_maker(岡本11/6追記)----
    #gnome-terminal -- bash -c "roslaunch wizurg waypoint_maker_accessory.launch map_file:=${mapfile}; bash"
    cd ~/catkin_ws/src/expo_wizurg/waypoints; rosrun expo_wizurg wizurg_waypoint_maker.py ${wayfile}.json
    cd -
 fi
 # ----waypoint_editor(岡本11/6追記)------
 if [ "x${editor}" = "xtrue" ]; then
    cd ~/catkin_ws/src/expo_wizurg/waypoints; rosrun expo_wizurg wizurg_waypoint_editor.py ${wayfile}.json
    cd -
 fi
 # ---------------------------------------
 
 if [ "x${navigation}" = "xtrue" ]; then
    echo "navigation_true"
    echo "wayfile = ${wayfile}.json"
    cd ~/catkin_ws/src/expo_wizurg/waypoints; rosrun expo_wizurg wizurg_navigation.py ${wayfile}.json
    cd -
 fi

fi



#========マップsave(入力待ち)=======
if [ "x${mapping}" = "xtrue" ]; then
 gnome-terminal -- bash -c "cd ~/github/hokuyo_slam"
 str="none"
 while [ "x${str}" != "xsave" ]; do
  echo "マップセーブ(save)"
  read str
 done
 cd ~/catkin_ws/src/expo_wizurg/map && rosrun map_server map_saver -f ${mapfile}
fi
#==================================





