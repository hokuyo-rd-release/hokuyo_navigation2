#!/bin/bash

# ROS 2 環境設定
source "$(dirname "$0")/../setup_ros_env.sh"

# --- 引数から設定を取得 ---
csv_file_arg="$1"

# 固定値
use_gnss_switch="false" # multi_mapではGNSS切り替えは利用しない想定
use_lio="true"
stop_uam_manage="false"
# ------------------------

EXITCODE=$?

cd ${HOKUYO_NAV2_PKG_PATH}

wizurg_opt="plural_opt";
inbagname="none";
p2obagname="none";
p2odir="none";
p2omapname="none";
liomapname="none";

echo "args: $1 ";
echo "wizurg_opt=${wizurg_opt}";

#============================

options=(`cat ./config/wizurg_opts/${wizurg_opt}.csv`)   #-- params/wizurg_opts/を新規作成（岡本11/12追記）--
#--2024/10/30 追記ここまで--

# --- CSVファイルのパスを決定 ---
if [ -n "$csv_file_arg" ]; then
    csv_file_path="$csv_file_arg"
else
    csv_file_path="${HOKUYO_NAV2_PKG_PATH}/config/maps_and_waypoints.csv" # デフォルトパス
fi
map_names=(`cat ${csv_file_path}`) 
Rmapfile=()
Rwayfile=()
Rnavtype=()

#-------オプション入力情報を格納----------
#-------options の配列のデータを読む。------#
for i in ${!options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  option_arr[$j]=`echo ${options[$i]} | cut -d ',' -f 2`
  fi
done

#------複数マップ名、ウェイポイントファイル名の読み込み------
#------map_and_waypoints のデータを読む。(True) データがなければ空読みする。(False) ------
for i in ${!map_names[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  Rmapfile[$j]=`echo ${map_names[$i]} | cut -d ',' -f 1`
  Rwayfile[$j]=`echo ${map_names[$i]} | cut -d ',' -f 2`
  Rnavtype[$j]=`echo ${map_names[$i]} | cut -d ',' -f 3`
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

#----複数マップ----
for i in ${!Rmapfile[@]}; do
  # --- ナビゲーションタイプに応じて use_gnss_switch を設定 ---
  current_nav_type=${Rnavtype[$i]}
  if [ "${current_nav_type}" = "gnss" ]; then
      echo "ナビゲーションタイプ: GNSS"
      use_gnss_switch="true"
  elif [ "${current_nav_type}" = "loc" ]; then
      echo "ナビゲーションタイプ: LIO (Localization)"
      use_gnss_switch="false"
  else
      echo "警告: 不明なナビゲーションタイプです: '${current_nav_type}'。デフォルト(false)を使用します。"
      use_gnss_switch="false"
  fi
  # init_pose.txt と init_lat_lon_alt.txt から最終行を読み込む
  Rinit_pose=$(tail -n 1 "${HOKUYO_NAV2_PKG_PATH}/data/${Rmapfile[$i]}/init_pose.txt" 2>/dev/null)
  Rinit_latlon=$(tail -n 1 "${HOKUYO_NAV2_PKG_PATH}/data/${Rmapfile[$i]}/init_lat_lon_alt.txt" 2>/dev/null)

  # ファイルが存在しない、または空の場合のデフォルト値を設定
  if [ -z "$Rinit_pose" ]; then
    Rinit_pose="0.0,0.0,0.0,0.0,0.0,0.0,1.0"
    echo "Warning: init_pose.txt for ${Rmapfile[$i]} not found or empty. Using default."
  fi
  if [ -z "$Rinit_latlon" ]; then
    Rinit_latlon="35.0,135.0,40.0" # デフォルトの緯度経度高度
    echo "Warning: init_lat_lon_alt.txt for ${Rmapfile[$i]} not found or empty. Using default."
  fi

  Rpose_arr=( `echo ${Rinit_pose} | tr -s ',' ' '`)
  Rpose1=${Rpose_arr[0]}
  Rpose2=${Rpose_arr[1]}
  Rpose3=${Rpose_arr[2]}
  Rpose4=${Rpose_arr[3]}
  Rpose5=${Rpose_arr[4]}
  Rpose6=${Rpose_arr[5]}
  Rpose7=${Rpose_arr[6]}

  Rlla_arr=( `echo ${Rinit_latlon} | tr -s ',' ' '`)
  latlon1=${Rlla_arr[0]}
  latlon2=${Rlla_arr[1]}
  latlon3=${Rlla_arr[2]}

  sleep 8s
  echo "sleep 8"
  echo "Starting navigation for map: ${Rmapfile[$i]}"

  gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml use_joy:=${use_joy} use_mapping:=${mapping} use_navigation:=${navigation} use_loader:=${loader} use_editor:=${editor} use_sensor:=${sensor} use_icart:=${icart} use_lio:=${use_lio} use_unity_sim:=${use_unity} use_gnss_switch:=${use_gnss_switch} stop_uam_manage:=${stop_uam_manage} map_file:=${Rmapfile[$i]} initial_pose:=\"${Rpose1},${Rpose2},${Rpose3},${Rpose4},${Rpose5},${Rpose6},${Rpose7}\" latlon_pose:=\"${latlon1},${latlon2},${latlon3}\";bash"
  sleep 2s
  echo "sleep 2"
  echo "start wizurg_navigation ${Rwayfile[$i]}"
  cd ${HOKUYO_NAV2_PKG_PATH}/waypoints; ros2 run waypoint_manager waypoint_manager ${Rwayfile[$i]}.json --once --ros-args -p use_gnss_switch:=${use_gnss_switch} -p cmd_vel_topic:=wizurg/cmd_vel
  echo "finish map"
  cd -
  # 次のマップに行く前にノードを終了させる
  echo "Killing ROS nodes before starting next map..."
  ros2 node list | grep -v 'gnome-terminal' | xargs ros2 node kill
done

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done