#!/bin/bash
cd ~/catkin_ws/src/wizurg_ros1/scripts
#------ファイル名記述ファイルの読み込み------
names=(`cat bag_map_names.csv`)

cd ~/catkin_ws && source devel/setup.bash && cd -
cd ~/catkin_ws/src/wizurg_ros1

#------複数ファイル名の読み込み------
for i in ${!names[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  
  Rbagdir[$j]=`echo ${names[$i]} | cut -d ',' -f 1`
  Rbagfile[$j]=`echo ${names[$i]} | cut -d ',' -f 2`
  RNbagfile[$j]=`echo ${names[$i]} | cut -d ',' -f 3`
  Rmintime[$j]=`echo ${names[$i]} | cut -d ',' -f 4`
  Rmaxtime[$j]=`echo ${names[$i]} | cut -d ',' -f 5`
  
  echo "${Rbagdir[$j]},${Rbagfile[$j]},${RNbagfile[$j]},${Rmintime[$j]},${Rmaxtime[$j]}"
  
 fi
done

#----ファイル毎にrosbagの必須トピック抽出----
for i in ${!Rbagfile[@]}; do

 gnome-terminal -- roslaunch rosbag_editor edit_odom.launch input_bag_dir:=${Rbagdir[$i]} input_bag_file:=${Rbagfile[$i]}.bag output_bag_file:=${RNbagfile[$i]}.bag start_time:=${Rmintime[$i]} end_time:=${Rmaxtime[$i]} node_name:=${RNbagfile[$i]}_editor
 
 sleep 2
 
done

#各ターミナルが終わるまで入力待ち
dummy="a"
read dummy
#念の為kill_nodes
rosnode kill -a
 
#----ファイル名毎にマッピング----
for i in ${!RNbagfile[@]}; do

 #mapping_launch
 echo "start mapping ${i}";\
 roslaunch wizurg start_mapping_bag.launch &
 sleep 10
 
 #rosbag_play
 cd rosbag
 rosbag play ${RNbagfile[$i]}.bag --clock
 cd -
 
 #map_save
 cd map
 rosrun map_server map_saver -f ${RNbagfile[$i]}
 cd -
   
 #kill_nodes
 rosnode kill -a
 sleep 10 
 
done

