#!/bin/bash

source /opt/ros/humble/setup.bash
cd /home/colcon_ws
source install/setup.bash
source ~/.bashrc

WIZURG_OPTIONS=9
# WIZURG_OPTIONS=$(zenity --list --title="WIZURGの起動コマンド" --text="1つ選択してください" \
#     --width=800 --height=400 \
#     --print-column=1 --separator= \
#     --column="番号" --column="オプション" --column="オプションの説明" \
#     1 "control_opt" " ... manual operation only" \
#     2 "sensor_rosbag" " ... manual operation and rosbag record" \
#     3 "rosbag_filter" " ... rosbag filtering gui" \
#     4 "get_rosbag" " ... get rosbag for sync_odom fix hokuyo_cloud2" \
#     5 "hokuyo_slam" " ... run hokuyo_slam to make pcd file" \
#     6 "map_opt" " ... mapping pcd to pgm" \
#     7 "way_opt" " ... make waypoints" \
#     8 "edit_opt" " ... edit waypoints" \
#     9 "nav_opt" " ... single_map navigation" \
#     10 "plural_opt" " ... multi_map navigation" \
#     11 "pcd_opt" " ... generate hokuyo_lio raw_map " 2>/dev/null)
 
EXITCODE=$?
echo "EXITCODE=$EXITCODE"
echo "WIZURG_OPTIONS=$WIZURG_OPTIONS"

if [ -z "$WIZURG_OPTIONS" ]; then
  echo "終了します。>"
  exit 1
fi

ROS2_WS="/home/colcon_ws"
HOKUYO_NAV2_PKG_PATH="/home/colcon_ws/src/hokuyo_navigation2"

cd ${HOKUYO_NAV2_PKG_PATH}

# yad --file \  --multiple \  --separator="," \  --add-preview \  --quoted-output

#--2024/10/30 Mapping と Navigation を分けるように追記--

wizurg_opt="wizurg_opt";                        #-- wizurg_optを追記 --
inbagname="none";
p2obagname="none";
p2odir="none";
p2omapname="none";
liomapname="none";

#========入力待ち1=======
operation_str=-1
operation_str=${WIZURG_OPTIONS}

case $operation_str in
  1) wizurg_opt="control_opt";;              #-- -C で control_opt.csv が入る (岡本11/10追記)--
  2) wizurg_opt="sensor_rosbag";;            #-- -B で sensor_rosbag.csv が入る (高橋11/6追記)--
  3) gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; source ~/.bashrc; cd /home/colcon_ws; source install/setup.bash; cd ${HOKUYO_NAV2_PKG_PATH}; python3 src/MainWindow.py bash"; exit;;
  4) tree -L 1 -a ${HOKUYO_NAV2_PKG_PATH}/rosbag ; inbagname=$(basename "$(zenity --file-selection --directory --title='choose your rosbag directory' --filename='/home/colcon_ws/src/hokuyo_navigation2/rosbag')") || { echo "エラー: ディレクトリの選択がキャンセルされました。" >&2; exit 1; } ; p2obagname=$(zenity --entry --title="input_rosbag" --text="input p2o rosbagfile name:" --entry-text "new file" \ map1) && [ -n "$p2obagname" ] || { echo "エラー: 入力がキャンセルされたか、空白です。" >&2; exit 1; } ; cd ${HOKUYO_NAV2_PKG_PATH}; scripts/get_rosbag.bash ${inbagname} rosbag/${p2obagname}; exit;;
  5) tree -L 1 -a ${HOKUYO_NAV2_PKG_PATH}/rosbag ; p2obagname=$(basename "$(zenity --file-selection --directory --title='choose your directory' --filename='/home/colcon_ws/src/hokuyo_navigation2/rosbag')") || { echo "エラー: ディレクトリの選択がキャンセルされました。" >&2; exit 1; } ; p2omapname=$(zenity --entry --title="input_rosbag" --text="input p2o rosbagfile name:" --entry-text "new file") && [ -n "$p2omapname" ] || { echo "エラー: 入力がキャンセルされたか、空白です。" >&2; exit 1; } ; cd ${HOKUYO_NAV2_PKG_PATH}; scripts/hokuyo_slam.bash ${p2obagname} ${p2omapname}; exit;;
  6) wizurg_opt="map_opt";;                  #-- -M で map_opt.csv が入る --
  7) wizurg_opt="way_opt";;                  #-- -W で way_opt.csv が入る (岡本11/6追記) --
  8) wizurg_opt="edit_opt";;                 #-- -E で edit_opt.csv が入る (岡本11/6追記)--
  9) wizurg_opt="nav_opt";;                  #-- -N で nav_opt.csv が入る --
  10) wizurg_opt="plural_opt";;              #-- -P で plural_opt.csv が入る (岡本11/10追記)--
  11) liomapname=$(zenity --entry --title="input lio raw map name" --text="input lio raw map name:" --entry-text "new file" \ map1) && [ -n "$liomapname" ] || { echo "エラー: 入力がキャンセルされたか、空白です。" >&2; exit 1; } ; cd ${ROS2_WS} ; ros2 launch hokuyo_navigation2 hlio_make_pcd_launch.xml map_name:=${liomapname}; exit;;
esac

wizurg_opt="${wizurg_opt}_lio"

#============================
echo "wizurg_opt=${wizurg_opt}";

options=(`cat ./config/wizurg_opts/99_${wizurg_opt}.csv`)   #-- params/wizurg_opts/を新規作成（岡本11/12追記）--
common_options=(`cat ./config/wizurg_opts/99_common_opt.csv`)   #-- 新規作成（岡本2025/2/28追記）--

#--2024/10/30 追記ここまで--

map_names=(`cat ${HOKUYO_NAV2_PKG_PATH}/config/maps_and_waypoints.csv`) 
Rmapfile=()
Rwayfile=()

#-------オプション入力情報を格納----------
#-------options の配列のデータを読む。------#
for i in ${!options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  option_arr[$j]=`echo ${options[$i]} | cut -d ',' -f 2`
  fi
done

#-------共通オプションを格納(岡本 2025/2/28)----------
#-------common_options のデータを読む。-----------
#-------入力値"x"以外を上書きする。------#
for i in ${!common_options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  option_tmp=`echo ${common_options[$i]} | cut -d ',' -f 2`
  if [ "x${option_tmp}" != "xx" ]; then
   option_arr[$j]=${option_tmp}
  fi
fi
done


#------複数マップ名、ウェイポイントファイル名の読み込み------
#------map_and_waypoints のデータを読む。(True) データがなければ空読みする。(False) ------
for i in ${!map_names[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  Rmapfile[$j]=`echo ${map_names[$i]} | cut -d ',' -f 1`
  Rwayfile[$j]=`echo ${map_names[$i]} | cut -d ',' -f 2`
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
gnome-terminal -- bash -c "${HOKUYO_NAV2_PKG_PATH}/scripts/kill_all_rosnode.sh"

#-------ypspur-coordinator起動------------
if [ "x${ypspur}" = "xtrue" ]; then
gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 icart_mini_drive_launch.xml ; bash"
# gnome-terminal -- bash -c "/usr/local/bin/ypspur-coordinator -d /dev/ttyUSB0 --blvr -p ~/colcon_ws/src/hokuyo_navigation2/config/icart/iCart3_100W.param ; bash"
 #---------spur待機---------------
 sleep 1
fi

#============= wizurg_satrt.launch起動 =============
#gnome-terminal -- bash -c "roscore" # 25/1/16 岡本追記
#sleep 1

#----複数マップ----
if [ "x${multi_map}" = "xtrue" ]; then
 for i in ${!Rmapfile[@]}; do
  IFS_BACKUP=$IFS
  IFS=$'\n'
  for Rinit_pose in `cat ${HOKUYO_NAV2_PKG_PATH}/data/${Rmapfile[$i]}/init_pose.txt`
  do
  i=`expr $i + 1`
  #   echo ${Rinit_pose}
  done
  IFS=$IFS_BACKUP

  Rpose_arr=( `echo ${Rinit_pose} | tr -s ',' ' '`)
  Rpose1=${Rpose_arr[0]}
  Rpose2=${Rpose_arr[1]}
  Rpose3=${Rpose_arr[2]}
  Rpose4=${Rpose_arr[3]}
  Rpose5=${Rpose_arr[4]}
  Rpose6=${Rpose_arr[5]}
  Rpose7=${Rpose_arr[6]}

  echo "kill all_nodes"
  rosnode kill -a
  sleep 8s
  echo "sleep 8"
  echo "map_count:${Rmapfile[$i-1]}"
  if [ "x${rosbag_record}" = "xtrue" ]; then
    echo "ros2 bag record"
    gnome-terminal -- bash -c "sleep 2; cd ${rosbag_dir}; ros2 bag record -a -o ${Rmapfile[$i-1]}; bash"
  fi
  gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml use_joy:=${use_joy} use_mapping:=${mapping} use_navigation:=${navigation} use_loader:=${loader} use_editor:=${editor} use_sensor:=${sensor} use_icart:=${icart} use_lio:=${use_lio} use_unity_sim:=${use_unity} map_file:=${Rmapfile[$i-1]} initial_pose:="${Rpose1},${Rpose2},${Rpose3},${Rpose4},${Rpose5},${Rpose6},${Rpose7}";bash"
  sleep 2s
  echo "sleep 2"
  echo "start wizurg_navigation ${Rwayfile[$i-1]}"
  cd ${HOKUYO_NAV2_PKG_PATH}/waypoints; ros2 run hokuyo_navigation2 waypoint_manager -r ${Rwayfile[$i-1]}.json once
  echo "finish map"
  cd -
 done

#----単一マップ---
else
  IFS_BACKUP=$IFS
  IFS=$'\n'
  i=0
  for init_pose in `cat ${HOKUYO_NAV2_PKG_PATH}/data/${mapfile}/init_pose.txt`
  do
    i=`expr $i + 1`
  #   echo ${init_pose}
  done
  IFS=$IFS_BACKUP

  pose_arr=( `echo ${init_pose} | tr -s ',' ' '`)
  pose1=${pose_arr[0]}
  pose2=${pose_arr[1]}
  pose3=${pose_arr[2]}
  pose4=${pose_arr[3]}
  pose5=${pose_arr[4]}
  pose6=${pose_arr[5]}
  pose7=${pose_arr[6]}

  echo ${pose1} ${pose2} ${pose3} ${pose4} ${pose5} ${pose6} ${pose7}

  # ------rosbag record----------------
  if [ "x${rosbag_record}" = "xtrue" ]; then
     echo "rosbag record"
     gnome-terminal -- bash -c "sleep 2; cd ${rosbag_dir}; ros2 bag record -a -o ${mapfile}; bash"
  fi
#-------------------------------------
 gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml use_joy:=${use_joy} use_mapping:=${mapping} use_navigation:=${navigation} use_loader:=${loader} use_editor:=${editor} use_sensor:=${sensor} use_icart:=${icart}  use_lio:=${use_lio} use_unity_sim:=${use_unity} use_sensor:=${sensor} use_icart:=${icart} map_file:=${mapfile} initial_pose:="${pose1},${pose2},${pose3},${pose4},${pose5},${pose6},${pose7}" ;bash"
 sleep 1
 if [ "x${loader}" = "xtrue" ]; then
    gnome-terminal -- bash -c "cd ${rosbag_dir}; ros2 bag play ${mapfile}" # rosbag play → ./remap.sh
    gnome-terminal -- bash -c "ros2 run waypoint_manager waypoint_manager -w ${HOKUYO_NAV2_PKG_PATH}/waypoints/${wayfile}.json; bash"
 fi
 # ----waypoint_editor(岡本11/6追記)------
 if [ "x${editor}" = "xtrue" ]; then
    ros2 run waypoint_manager waypoint_manager -e ${HOKUYO_NAV2_PKG_PATH}/waypoints/${wayfile}.json
 fi
 # ---------------------------------------
 
 if [ "x${navigation}" = "xtrue" ]; then
    echo "navigation_true"
    echo "wayfile = ${wayfile}.json"
    sleep 7.0s
    cd ${HOKUYO_NAV2_PKG_PATH}/waypoints; ros2 run waypoint_manager waypoint_manager -r ${HOKUYO_NAV2_PKG_PATH}/waypoints/${wayfile}.json
    cd -
 fi

fi

#========マップsave(入力待ち)=======
if [ "x${mapping}" = "xtrue" ]; then
  # Zenityを使用して確認ダイアログを表示し、ユーザーの選択を取得
  if zenity --question --text="マップを保存しますか？"; then
      # 「はい」が押された場合の処理
      echo "マップを保存します..."
      cd ${HOKUYO_NAV2_PKG_PATH}/map && ros2 run nav2_map_server map_saver_cli -f ${mapfile}
      echo "マップの保存が完了しました。"
  else
      # 「いいえ」が押された場合の処理
      echo "マップは保存をキャンセルしました。"
  fi
fi
#==================================

# 全ての gnome-terminal ウィンドウの ID を取得して最小化
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done