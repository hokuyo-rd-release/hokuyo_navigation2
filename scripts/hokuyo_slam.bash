#!/bin/bash

# Docker環境かどうかを判定する
# コンテナの起動時に -e DOCKER_ENV=1 を指定することで、Docker環境とみなすことができます。
if [ -n "$DOCKER_ENV" ]; then
    source /opt/ros/humble/setup.bash
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_SLAM_WS="${HOME}/github/hokuyo_slam_ros2"
    HOKUYO_NAV2_PKG_PATH=${ROS2_WS}/src/hokuyo_navigation2
else
    source /opt/ros/humble/setup.bash
    # ワークスペースのパスもホストOSのものに合わせる
    # 実際のホストOSのワークスペースパスに置き換えてください
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_SLAM_WS="${HOME}/github/hokuyo_slam_ros2"
    HOKUYO_NAV2_PKG_PATH=${ROS2_WS}/src/hokuyo_navigation2
fi

# 実行方法
# ./hokuyo_slam.bash <rosbagファイル> <ディレクトリ名> <option>
# 第三引数はconfig/config.csvが読み込まれるため、必要に応じてcsvを編集することで
# ディレクトリは data/に作られる。
# rosbag 下に配置したrosbag はスクリプト実行時にdata/データ名/に移動する。

export CMAKE_PREFIX_PATH=/opt/vtk8
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vtk8/lib
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/pcl

# HOKUYO_SLAM_WS="/home/github/hokuyo_slam_ros2"
echo $HOKUYO_SLAM_WS

if [ -z "$1" ]; then
  echo "Error: 引数が不足しています <フォルダ名>"
  exit 1
fi

# 第2引数
if [ -z "$2" ]; then
  echo "Error: 引数が不足しています <arg2>"
  exit 1
fi

# チェックするパス
PATH_TO_CHECK="${HOKUYO_NAV2_PKG_PATH}/rosbag/$1"

# ファイルが存在するかチェック
if [ -f "$PATH_TO_CHECK" ]; then
  echo "rosbag file: $PATH_TO_CHECK exists."
elif [ -d "$PATH_TO_CHECK" ]; then
  echo "rosbag folder: $PATH_TO_CHECK exists."
else
  echo "Error: Path $PATH_TO_CHECK does not exist (neither file nor folder)."
  exit 1
fi

#------- カレントディレクトリの取得 -------
# CURRENT=$(cd $(dirname $0);pwd)
CURRENT=$HOKUYO_NAV2_PKG_PATH
echo current dir: $CURRENT
rosbag_dir=$CURRENT/rosbag;
map_dir=$CURRENT/map;
echo rosbag dir: $rosbag_dir
echo 'ouput directory_name: '"$2"
echo 'rosbag file: ' "$1"
echo "All args are checked."

#------- config.csv 読み込み -------
if [ "$3" = "" ]; then
  options=(`cat ${CURRENT}/config/config.csv`)
  echo option: $options
else
  options=(`cat $3`)
  echo option: $options
fi

for i in ${!options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  option_arr[$j]=`echo ${options[$i]} | cut -d ',' -f 2`
  fi
done

gnss_topic="${option_arr[0]}";
pointcloud_topic="${option_arr[1]}";
lio_topic="${option_arr[2]}";
gnss_cov_thre="${option_arr[4]}";

echo 'gnss_topic: '${gnss_topic}
echo 'pointcloud_topic: '${pointcloud_topic}
echo 'lio_topic: '${lio_topic}
echo 'gnss_cov_thre: '${gnss_cov_thre}
sleep 1

cd $HOKUYO_NAV2_PKG_PATH

# ディレクトリ作成
rm -rf data/$2
mkdir -p data/$2
mkdir -p data/$2/PCDs

# rosbag 移動
mv rosbag/$1 data/$2

sleep 1

# gnssのログを確認する。
bash -c "python3 src/p2o_gnsslog_from_rosbag_ros2.py data/$2/$1 gnss_log/${2}_gnss_cov_${gnss_cov_thre}.csv $gnss_topic $gnss_cov_thre"
gnss_opt=(`cat gnss_log/${2}_gnss_cov_${gnss_cov_thre}.csv`)

sleep 1

for i in ${!gnss_opt[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  gnss_opt_arr[$j]=`echo ${gnss_opt[$i]} | cut -d ',' -f 2`
  fi
done

fix_rate1=`echo "${gnss_opt_arr[0]} < 40.0" | bc`
fix_rate=`echo "${gnss_opt_arr[0]} >= 40.0" | bc`

if [ ${fix_rate1} -eq 1 ] ; then
  echo 'fix トピックの共分散のfix率が'${gnss_opt_arr}'%です。gnss_cov_threの値を大きくしてください。'

elif [ ${fix_rate} -eq 1 ] ; then
  echo 'p2o 開始'

  sleep 1
  # p2o　正常終了の場合のみ処理を実行したい。
  bash -c "python3 src/p2o_from_rosbag_ros2.py data/$2/$1 $lio_topic $gnss_topic $gnss_cov_thre data/$2/center_lat_lon_alt.txt data/$2/center_utm.txt data/$2/lio_edge_timestamps.txt > data/$2/output.p2o" # 引数2 input.bag
  result=$?

  echo 'error status:' ${result}

  if [ ${result} -eq 0 ] ; then
    bash -c "${HOKUYO_SLAM_WS}/build/run_p2o data/$2/center_utm.txt data/$2/output.p2o"
    #bash -c "gnuplot atc_odom_gnss.plt"

    # p2o_fastlio_util
    cd ${HOKUYO_NAV2_PKG_PATH}/data/$2/PCDs 

    bash -c "python3 ../../../src/extract_pcd_ros2.py ../$1 $pointcloud_topic $HOKUYO_NAV2_PKG_PATH/data/$2/lio_edge_timestamps.txt" # ~/p2o_fastlio_util/extract_pcd 引数1 + 引数2

    # p2o_fastlio_util におけるファイル整理
    cd ./..
    tail -n +2 output.p2o_out.txt > poses.txt
    find . | grep pcd > clouds.txt
    sort clouds.txt > sorted_clouds.txt
    paste sorted_clouds.txt poses.txt > concat.txt
    bash -c "${HOKUYO_SLAM_WS}/build/rearrange_pointcloud concat.txt $2"

    # 絶対座標を相対座標に変換
    cd ../..
    bash -c "python3 src/pcd_to_Rcord.py data/$2/${2}_Acord.pcd data/$2/${2}_Rcord.pcd data/$2/output.p2o_out.txt data/$2/init_pose.txt data/$2/init_lat_lon_alt.txt"
    bash -c "mv data/$2/${2}_Rcord.pcd map"
    bash -c "mv map/${2}_Rcord.pcd map/${2}.pcd"
  elif [ ${result} -eq 1 ] ; then
    echo 'rosbag play でfixメッセージがあるかの確認と、gnss_logで共分散の値を確認してください。'
  fi
fi