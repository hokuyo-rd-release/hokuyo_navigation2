#!/bin/bash

# Docker環境かどうかを判定する
# コンテナの起動時に -e DOCKER_ENV=1 を指定することで、Docker環境とみなすことができます。
if [ -n "$DOCKER_ENV" ]; then
    source /opt/ros/humble/setup.bash
    cd /home/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="/home/colcon_ws"
    HOKUYO_SLAM_WS="/home/github/hokuyo_slam_ros2"
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

# 実行方法 (server.pyとstart_mapping.shの変更後):
# ./hokuyo_slam.bash <rosbagベース名> <マップ名> <MAP_DIR> <FLAG_FILE_NAME> <option>
# $1: rosbagのベース名 (例: sync_bag)
# $2: マップ名 (例: final_map)
# $3: MAP_DIR (例: /home/hokuyo/colcon_ws/src/hokuyo_navigation2/map)
# $4: FLAG_FILE_NAME (例: final_map.P2O_DONE)
# $5: WP_DIR (waypoint出力先ディレクトリ)
# $6: option (configファイルパス)

# 🌟 サーバー側から渡された新しい引数を変数に格納 🌟

if [ "$3" = "" ]; then
  MAP_DIR="$HOKUYO_NAV2_PKG_PATH/map"         # server.py の MAP_DIR

else
  MAP_DIR="$3"         # server.py の MAP_DIR
fi

if [ "$4" = "" ]; then
  FLAG_FILE_NAME="$2.P2O_DONE"  # server.py の $OUTPUT_MAP_NAME.P2O_DONE
else
  FLAG_FILE_NAME="$4"  # server.py の $OUTPUT_MAP_NAME.P2O_DONE
fi
# ----------------------------------------------------

export CMAKE_PREFIX_PATH=/opt/vtk8
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vtk8/lib
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/pcl

# HOKUYO_SLAM_WS="/home/github/hokuyo_slam_ros2"
echo $HOKUYO_SLAM_WS

if [ -z "$1" ]; then
  echo "Error: 引数が不足しています <rosbagベース名>"
  exit 1
fi

# 第2引数: マップ名
if [ -z "$2" ]; then
  echo "Error: 引数が不足しています <マップ名>"
  exit 1
fi

# 第3引数: MAP_DIR (必須)
if [ -z "$MAP_DIR" ]; then
  echo "Error: 引数が不足しています <MAP_DIR>"
  exit 1
fi

# 第4引数: FLAG_FILE_NAME (必須)
if [ -z "$FLAG_FILE_NAME" ]; then
  echo "Error: 引数が不足しています <FLAG_FILE_NAME>"
  exit 1
fi

# チェックするパス
PATH_TO_CHECK="${HOKUYO_NAV2_PKG_PATH}/rosbag/$1"

# ファイルが存在するかチェック (このスクリプトが呼ばれる時点で、ROS Bagはまだ移動されていない)
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
map_dir=$CURRENT/map; # <-- MAP_DIRは $3 で上書きされるためここでは使わない
echo rosbag dir: $rosbag_dir
echo 'ouput directory_name (Map Name): '"$2"
echo 'rosbag file: ' "$1"
echo "PCD Output Directory: $MAP_DIR"
echo "Flag File Name: $FLAG_FILE_NAME"
echo "All args are checked."

#------- config.csv 読み込み -------
# 第5引数 (オプション)がconfigファイルパスとして使用される
if [ "$6" = "" ]; then
  options=(`cat ${CURRENT}/config/config.csv`)
  echo option: $options
else
  options=(`cat $6`)
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
    bash -c "${HOKUYO_SLAM_WS}/build/rearrange_pointcloud concat.txt $2 $5/${2}.json"

    # 絶対座標を相対座標に変換
    cd ../..
    bash -c "python3 src/pcd_to_Rcord.py data/$2/${2}_Acord.pcd data/$2/${2}_Rcord.pcd data/$2/output.p2o_out.txt data/$2/init_pose.txt data/$2/init_lat_lon_alt.txt"
    
    # 🌟 PCDファイルの移動先を $MAP_DIR に変更 🌟
    bash -c "mv data/$2/${2}_Rcord.pcd $MAP_DIR"
    bash -c "mv $MAP_DIR/${2}_Rcord.pcd $MAP_DIR/${2}.pcd"
    
    # 🌟 完了フラグ作成の追記とパスの修正 🌟
    FLAG_PATH="${MAP_DIR}/${FLAG_FILE_NAME}" # $MAP_DIR と $FLAG_FILE_NAME を結合
    touch "$FLAG_PATH"
    echo "P2O SLAM completion flag created: $FLAG_PATH"
    # ---------------------------
  elif [ ${result} -eq 1 ] ; then
    echo 'rosbag play でfixメッセージがあるかの確認と、gnss_logで共分散の値を確認してください。'
  fi
fi