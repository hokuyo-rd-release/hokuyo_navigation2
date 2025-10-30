#!/bin/bash

# ROS 2 環境設定
source "$(dirname "$0")/../setup_ros_env.sh"

# --- 引数からナビゲーション設定を取得 ---
mapfile_arg="$1"
wayfile_arg="$2"
nav_type_arg="$3"

use_gnss_switch="false"
use_lio="true"
stop_uam_manage="false"

if [ "${nav_type_arg}" = "gnss" ]; then
    echo "ナビゲーションタイプ: GNSS"
    use_gnss_switch="true"
elif [ "${nav_type_arg}" = "loc" ]; then
    echo "ナビゲーションタイプ: LIO (Localization)"
    use_gnss_switch="false"
else
    echo "警告: 不明なナビゲーションタイプです: '${nav_type_arg}'。デフォルト設定を使用します。"
fi

# ------------------------------------

EXITCODE=$?
echo "EXITCODE=$EXITCODE"

cd ${HOKUYO_NAV2_PKG_PATH}

wizurg_opt="nav_opt_lio";
inbagname="none";
p2obagname="none";
p2odir="none";
p2omapname="none";
liomapname="none";

echo "args: $1 $2 ";
echo "wizurg_opt=${wizurg_opt}";

#============================

options=(`cat ./config/wizurg_opts/${wizurg_opt}.csv`)   #-- params/wizurg_opts/を新規作成（岡本11/12追記）--

#-------オプション入力情報を格納----------
#-------options の配列のデータを読む。------#
for i in ${!options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  option_arr[$j]=`echo ${options[$i]} | cut -d ',' -f 2`
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
mapfile=${mapfile_arg:-${option_arr[9]}}; # 引数がなければCSVの値を使用
wayfile=${wayfile_arg:-${option_arr[10]}}; # 引数がなければCSVの値を使用
rosbag_record="${option_arr[11]}";
rosbag_dir="${option_arr[12]}";
loader="${option_arr[13]}";
editor="${option_arr[14]}";

echo "use_joy:${use_joy}";
echo "mapping:${mapping}";
echo "navigation:${navigation}";
echo "sensor:${sensor}";
echo "icart:${icart}";
echo "use_lio:${use_lio}";
echo "use_unity:${use_unity}";
echo "ypspur:${ypspur}";
echo "multi_map:${multi_map}";
echo "mapfile:${mapfile}";
echo "wayfile:${wayfile}";
echo "rosbag_record:${rosbag_record}";
echo "rosbag_dir:${rosbag_dir}";
echo "loader:${loader}";
echo "editor:${editor}";
#-------kill_all_rosnode起動--------------
gnome-terminal -- bash -c "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/kill_all_rosnode.sh"
#-------ypspur-coordinator起動------------
if [ "x${ypspur}" = "xtrue" ]; then
gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 icart_mini_drive_launch.xml ; bash"
# gnome-terminal -- bash -c "/usr/local/bin/ypspur-coordinator -d /dev/ttyUSB0 --blvr -p ~/colcon_ws/src/hokuyo_navigation2/config/icart/iCart3_100W.param ; bash"
 #---------spur待機---------------
 sleep 1
fi

#----単一マップ---

IFS_BACKUP=$IFS
IFS=$'\n'
i=0
for init_pose in `cat ${HOKUYO_NAV2_PKG_PATH}/data/${mapfile}/init_pose.txt`
do
  i=`expr $i + 1`
#   echo ${init_pose}
done
j=0
for init_latlon in `cat ${HOKUYO_NAV2_PKG_PATH}/data/${mapfile}/init_lat_lon_alt.txt`
do
  j=`expr $j + 1`
done
IFS=$IFS_BACKUP

pose_arr=( `echo ${init_pose} | tr -s ',' ' '`)
latlon_arr=( `echo ${init_latlon} | tr -s ',' ' '`)
pose1=${pose_arr[0]}
pose2=${pose_arr[1]}
pose3=${pose_arr[2]}
pose4=${pose_arr[3]}
pose5=${pose_arr[4]}
pose6=${pose_arr[5]}
pose7=${pose_arr[6]}
latlon1=${latlon_arr[0]}
latlon2=${latlon_arr[1]}
latlon3=${latlon_arr[2]}

echo ${pose1} ${pose2} ${pose3} ${pose4} ${pose5} ${pose6} ${pose7}

#-------------------------------------
gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml use_joy:=${use_joy} use_mapping:=${mapping} use_navigation:=${navigation} use_loader:=${loader} use_editor:=${editor} use_sensor:=${sensor} use_icart:=${icart}  use_lio:=${use_lio} use_unity_sim:=${use_unity} use_gnss_switch:=${use_gnss_switch} stop_uam_manage:=${stop_uam_manage} use_sensor:=${sensor} use_icart:=${icart} map_file:=${mapfile} initial_pose:="${pose1},${pose2},${pose3},${pose4},${pose5},${pose6},${pose7}" latlon_pose:="${latlon1},${latlon2},${latlon3}" ;bash"
sleep 1

echo "wayfile = ${wayfile}.json"
sleep 7.0s

gnome-terminal -- bash -c "ros2 run waypoint_manager waypoint_manager ${HOKUYO_NAV2_PKG_PATH}/waypoints/${wayfile}.json --ros-args -p use_gnss_switch:=${use_gnss_switch} -p cmd_vel_topic:=wizurg/cmd_vel"

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done