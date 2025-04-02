#!/bin/bash

# 実行方法
# ./hokuyo_slam.bash <rosbagファイル> <ディレクトリ名> <option>
# 第三引数はconfig/config.csvが読み込まれるため、必要に応じてcsvを編集することで
# ディレクトリは data/に作られる。
# rosbag 下に配置したrosbag はスクリプト実行時にdata/データ名/に移動する。
export CMAKE_PREFIX_PATH=/opt/vtk8
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/vtk8/lib
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/pcl

if [ -z "$1" ]; then
  echo "Error: 引数が不足しています <フォルダ名>"
  exit 1
fi

# 第2引数
if [ -z "$2" ]; then
  echo "Error: 引数が不足しています <arg2>"
  exit 1
fi

# ファイルが存在するかチェック
FILE=$ROS_WORKSPACE/src/expo_wizurg/rosbag/$1
if [ ! -f "$FILE" ]; then
  echo "Error: File $FILE does not exist."
  exit 1
else
  echo "rosbag file: $FILE exists."
fi

#------- カレントディレクトリの取得 -------
CURRENT=$ROS_WORKSPACE/src/expo_wizurg
echo current dir: $CURRENT
rosbag_dir=$CURRENT/rosbag;
map_dir=$CURRENT;
echo rosbag dir: $rosbag_dir
echo 'ouput directory_name: '"$2"
echo 'rosbag file: ' "$1"
echo "All args are checked."

#------- config.csv 読み込み -------
if [ "$3" = "" ]; then
  options=(`cat ~/catkin_ws/src/expo_wizurg/config/config.csv`)
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

# ディレクトリ作成
mkdir -p $CURRENT/data/$2
mkdir -p $CURRENT/data/$2/PCDs

# rosbag 移動
mv $rosbag_dir/$1 $CURRENT/data/$2

# gnssのログを確認する。
bash -c "python3 ${CURRENT}/src/p2o_gnsslog_from_rosbag.py ${CURRENT}/data/$2/$1 ${CURRENT}/gnss_log/${2}_gnss_cov_${gnss_cov_thre}.csv $gnss_cov_thre"
gnss_opt=(`cat ${CURRENT}/gnss_log/${2}_gnss_cov_${gnss_cov_thre}.csv`)

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
  bash -c "python3 ${CURRENT}/src/p2o_from_rosbag.py ${CURRENT}/data/$2/$1 $lio_topic $gnss_topic $gnss_cov_thre > ${CURRENT}/data/$2/output.p2o" # 引数2 input.bag
  result=$?

  echo 'error status:' ${result}

  if [ ${result} -eq 0 ] ; then
    bash -c "~/github/hokuyo_slam/build/run_p2o ${CURRENT}/data/$2/output.p2o"
    #bash -c "gnuplot atc_odom_gnss.plt"

    # p2o_fastlio_util
    cd ${CURRENT}/data/$2/PCDs 

    bash -c "python3 ../../../src/extract_pcd.py ../$1 $pointcloud_topic" # ~/p2o_fastlio_util/extract_pcd 引数1 + 引数2

    # p2o_fastlio_util におけるファイル整理
    cd ./..
    tail -n +2 output.p2o_out.txt > poses.txt
    find . | grep pcd > clouds.txt
    sort clouds.txt > sorted_clouds.txt
    paste sorted_clouds.txt poses.txt > concat.txt
    bash -c "~/github/hokuyo_slam/build/rearrange_pointcloud concat.txt $2"

    # 絶対座標を相対座標に変換
    cd ../..
    bash -c "python3 src/pcd_to_Rcord.py ${CURRENT}/data/$2/${2}_Acord.pcd ${CURRENT}/data/$2/${2}_Rcord.pcd ${CURRENT}/data/$2/output.p2o_out.txt ${CURRENT}/data/$2/init_pose.txt ${CURRENT}/data/$2/init_lat_lon_alt.txt"
    bash -c "mv ${CURRENT}/data/$2/${2}_Rcord.pcd ${CURRENT}/map"
    bash -c "mv ../../map/${2}_Rcord.pcd ../../map/${2}.pcd"
  elif [ ${result} -eq 1 ] ; then
    echo 'rosbag play でfixメッセージがあるかの確認と、gnss_logで共分散の値を確認してください。'
  fi
fi