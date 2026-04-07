#!/bin/bash

# ROS 2 環境設定
source "$(dirname "$0")/../setup_ros_env.sh"

# HOKUYO_SLAM のバイナリディレクトリを動的に検索
echo "hokuyo_slam (run_p2o) のバイナリを検索しています..."
HOKUYO_SLAM_BIN_DIR=$(find "/" -type f -name "run_p2o" -executable -print -quit 2>/dev/null | xargs -I {} dirname {})

if [ -z "$HOKUYO_SLAM_BIN_DIR" ]; then
    echo "エラー: 'run_p2o' 実行ファイルが見つかりませんでした。" >&2
    echo "hokuyo_slam_ros2 プロジェクトが正しくビルドされているか確認してください。" >&2
    exit 1
fi
echo "hokuyo_slam のバイナリディレクトリが見つかりました: ${HOKUYO_SLAM_BIN_DIR}"

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

#------- hokuyo_slam_topics_cfg.csv 読み込み -------
# 第5引数 (オプション)がconfigファイルパスとして使用される
if [ "$6" = "" ]; then
  options=(`cat ${CURRENT}/config/hokuyo_slam_topics_cfg.csv`)
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
imu_topic="${option_arr[5]}";
slam_mode="${option_arr[6]}";

echo 'gnss_topic: '${gnss_topic}
echo 'pointcloud_topic: '${pointcloud_topic}
echo 'lio_topic: '${lio_topic}
echo 'gnss_cov_thre: '${gnss_cov_thre}
echo 'imu_topic: '${imu_topic}
echo 'slam_mode: '${slam_mode}
sleep 1

cd $HOKUYO_NAV2_PKG_PATH

# ディレクトリ作成
rm -rf data/$2
mkdir -p data/$2
mkdir -p data/$2/PCDs
sleep 1

# ------- Gravity SLAM Mode (IMU Gravity を使用した推定) -------
if [ "$slam_mode" = "gravity" ]; then
    echo "--> IMU Gravity ベースの SLAM モードを開始します。"
    
    # 1. 点群データの出力 (dump_lidar_pointcloud.py)
    echo "Extracting PCD files from bag..."
    python3 src/dump_lidar_pointcloud.py \
        --bag "rosbag/$1" \
        --topic "$pointcloud_topic" \
        --outdir "data/$2/PCDs/"
    
    # 2. ダミーの原点ファイル作成 (run_p2o の実行に必要)
    echo "0.0 0.0 0.0" > "data/$2/center_utm.txt"
    echo "0.0 0.0 0.0" > "data/$2/center_lat_lon_alt.txt"
    
    # 3. IMU重力情報を利用した p2o グラフの生成
    echo "Generating p2o graph with gravity edges..."
    python3 src/dump_p2o_with_imufilter_hokuyo_lio.py \
        "rosbag/$1" \
        --odom-topic "$lio_topic" \
        --imu-topic "$imu_topic" \
        --pcd-dir "data/$2/PCDs" \
        --stride 10 \
        --out "data/$2/output.p2o"

    # 4. グラフ最適化の実行
    echo "Optimizing pose graph (run_p2o)..."
    "${HOKUYO_SLAM_BIN_DIR}/run_p2o" "data/$2/center_utm.txt" "data/$2/output.p2o"
    
    # 5. 最適化結果から点群結合用のリスト (concat.txt) を作成
    # 形式: [pcd_path] [id] [x] [y] [z] [qx] [qy] [qz] [qw]
    # dump_p2o スクリプトが VERTEX 行の末尾に PCD パスを付与していることを利用します。
    grep "VERTEX_SE3:QUAT" "data/$2/output.p2o_out.txt" | \
        awk '$10 != "" {print $10, $2, $3, $4, $5, $6, $7, $8, $9}' > "data/$2/concat.txt"

    # 6. 点群の再配置と結合 (PCDマップとウェイポイントの生成)
    cd "data/$2"
    "${HOKUYO_SLAM_BIN_DIR}/rearrange_pointcloud" "concat.txt" "$2" "$5/${2}.json"
    cd ../..

    # 7. 絶対座標から相対座標への変換
    python3 src/pcd_to_Rcord.py \
        "data/$2/${2}_Acord.pcd" "data/$2/${2}_Rcord.pcd" "data/$2/output.p2o_out.txt" \
        "data/$2/init_pose.txt" "data/$2/init_lat_lon_alt.txt"
    
    # 8. 結果の移動と完了フラグ生成
    mv "data/$2/${2}_Rcord.pcd" "$MAP_DIR/${2}.pcd"
    FLAG_PATH="${MAP_DIR}/${FLAG_FILE_NAME}"
    touch "$FLAG_PATH"
    
    echo "Gravity SLAM completed. Map and flag created at: $FLAG_PATH"
    exit 0
fi

# gnssのログを確認する。
bash -c "python3 src/p2o_gnsslog_from_rosbag_ros2.py rosbag/$1 gnss_log/${2}_gnss_cov_${gnss_cov_thre}.csv $gnss_topic $gnss_cov_thre"
gnss_opt=(`cat gnss_log/${2}_gnss_cov_${gnss_cov_thre}.csv`)

sleep 1

for i in ${!gnss_opt[@]}; do
  if [ $i -gt 0 ]; then
    j=$((${i}-1))
    gnss_opt_arr[$j]=`echo ${gnss_opt[$i]} | cut -d ',' -f 2`
  fi
done

fix_rate1=0
if [ -n "${gnss_opt_arr[0]}" ]; then
    fix_rate1=`echo "${gnss_opt_arr[0]} < 40.0" | bc`
    fix_rate=`echo "${gnss_opt_arr[0]} >= 40.0" | bc`
fi

if [ ${fix_rate1} -eq 1 ] ; then
  echo 'fix トピックの共分散のfix率が'${gnss_opt_arr}'%です。gnss_cov_threの値を大きくしてください。'
  echo 'Fix率が低いため、Z軸拘束(擬似観測)を追加してSLAMを続行します。'
fi

if [ ${fix_rate1} -eq 1 ] || [ ${fix_rate} -eq 1 ] ; then
  echo 'p2o 開始'

  sleep 1
  # p2o　正常終了の場合のみ処理を実行したい。
  echo 'p2o_from_rosbag'
  bash -c "python3 src/p2o_from_rosbag_ros2.py rosbag/$1 $lio_topic $gnss_topic $gnss_cov_thre data/$2/center_lat_lon_alt.txt data/$2/center_utm.txt data/$2/lio_edge_timestamps.txt > data/$2/output.p2o" # 引数2 input.bag
  result=$?

  echo 'error status:' ${result}

  if [ ${result} -eq 0 ] ; then
    # fix_rate1 (fix率 < 40%) の場合、Z軸拘束を追加
    if [ ${fix_rate1} -eq 1 ]; then
        echo 'Applying pseudo Z0 observations...'
        mv data/$2/output.p2o data/$2/output_raw.p2o
        bash -c "python3 src/add_pseudo_z0_obs.py data/$2/output_raw.p2o > data/$2/output.p2o"
    fi

    echo 'run_p2o'
    bash -c "${HOKUYO_SLAM_BIN_DIR}/run_p2o data/$2/center_utm.txt data/$2/output.p2o"
    #bash -c "gnuplot atc_odom_gnss.plt"

    # p2o_fastlio_util
    cd ${HOKUYO_NAV2_PKG_PATH}/data/$2/PCDs 

    bash -c "python3 ../../../src/extract_pcd_ros2.py ../../../rosbag/$1 $pointcloud_topic $HOKUYO_NAV2_PKG_PATH/data/$2/lio_edge_timestamps.txt" # ~/p2o_fastlio_util/extract_pcd 引数1 + 引数2

    # p2o_fastlio_util におけるファイル整理
    cd ./..
    tail -n +2 output.p2o_out.txt > poses.txt
    find . | grep pcd > clouds.txt
    sort clouds.txt > sorted_clouds.txt
    paste sorted_clouds.txt poses.txt > concat.txt
    bash -c "${HOKUYO_SLAM_BIN_DIR}/rearrange_pointcloud concat.txt $2 $5/${2}.json"

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