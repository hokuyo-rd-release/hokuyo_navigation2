#!/bin/bash
# --------------------------------------------------------------------------
# ROS 2 環境設定
source "$(dirname "$0")/../setup_ros_env.sh"

# コマンドライン引数を取得 (すべてダブルクォーテーションで受け取ることを推奨)
# $1: rosbagファイル名 (例: my_synced_bag)
# $2: 出力マップ名 (例: final_map)
# $3: PCDの出力先ディレクトリ (例: /path/to/map)
# $4: 完了フラグファイルの絶対パス (例: /path/to/map/final_map.LIO_RAW_DONE)
inbagname="$1"
liomapname="$2"
pcd_output_dir="$3"
wp_output_dir="$4"
flag_file_name="$5"
config_file="$6"

echo "lio_raw.bash $0 $1 $2 $3 $4 $5 $6"

#------- CSV設定ファイル読み込み -------
# デフォルト値の設定
pointcloud_topic="/hokuyo3d/hokuyo_cloud2"
lio_topic="/rsf/lio_lidar_rate_odom"
pc_save_distance="1.0"
wp_save_distance="4.0"
tf_topic="/tf"
orig_frame="yvt"
target_frame="lio_odom"

if [ -z "$config_file" ]; then
  # 引数がない場合はデフォルトのパスを試行
  config_file="${HOKUYO_NAV2_PKG_PATH}/config/hokuyo_slam_topics_cfg.csv"
fi

if [ -f "$config_file" ]; then
    echo "Loading config from: $config_file"
    options=(`cat $config_file`)
    for i in ${!options[@]}; do
     if [ $i -gt 0 ]; then
      j=$((${i}-1))
      option_arr[$j]=`echo ${options[$i]} | cut -d ',' -f 2`
      fi
    done
    pointcloud_topic="${option_arr[1]:-$pointcloud_topic}"
    lio_topic="${option_arr[2]:-$lio_topic}"
    pc_save_distance="${option_arr[7]:-$pc_save_distance}"
    wp_save_distance="${option_arr[8]:-$wp_save_distance}"
    tf_topic="/dummy_tf"
    orig_frame="${option_arr[12]:-$orig_frame}"
    target_frame="${option_arr[13]:-$target_frame}"
else
    echo "WARNING: Config file not found at $config_file. Using default values."
fi

# 1. マップディレクトリを作成し、初期ポーズファイルを生成
# mkdir -p の引数も引用符で囲み、堅牢性を高めます。
mkdir -p "${HOKUYO_NAV2_PKG_PATH}/data/${liomapname}"
# 初期ポーズファイルは必須ではないが、以前のロジックを踏襲
echo "0.0,0.0,0.0,0.0,0.0,0.0,1.0" > "${HOKUYO_NAV2_PKG_PATH}/data/${liomapname}/init_pose.txt"
echo "35.0,135.0,40.0" > "${HOKUYO_NAV2_PKG_PATH}/data/${liomapname}/init_lat_lon_alt.txt"

# 2. pcd_tf_extractor.py を実行してLIO-RAW処理とPCDファイル抽出を同時に行う
echo "LIO-RAW処理とPCDファイル抽出を開始します... (入力Bag: ${inbagname}, 出力PCD: ${liomapname}.pcd)"

# 実行ディレクトリに移動
cd "${HOKUYO_NAV2_PKG_PATH}"

python3 src/pcd_tf_extractor.py \
    "rosbag/${inbagname}" \
    "${pointcloud_topic}" \
    "${lio_topic}" \
    dummy_pub_topic \
    "${orig_frame}" \
    "${target_frame}" \
    "${pcd_output_dir}" \
    "${liomapname}.pcd" \
    "${wp_output_dir}"\
    "${pc_save_distance}" \
    "${wp_save_distance}" \
    "${tf_topic}"

# 正常終了チェック
if [ $? -ne 0 ]; then
    echo "ERROR: pcd_tf_extractor.py がエラーコード $? で終了しました。完了フラグは出力されません。"
    # 処理失敗時は非ゼロで終了
    exit 1
fi

# 4. ROS 2 パッケージの再ビルド (PGM/YAMLの更新をシステムに反映)
cd "${ROS2_WS}"
echo "Building package hokuyo_navigation2 to include new map files..."
colcon build --symlink-install --packages-select hokuyo_navigation2

BUILD_STATUS=$?
if [ $BUILD_STATUS -ne 0 ]; then
    echo "ERROR: colcon build がエラーコード $BUILD_STATUS で失敗しました。マップが正しく読み込めない可能性があります。"
    exit 1
fi

echo "LIO-RAW処理とPCDファイル抽出が完了しました。"

# 3. 処理完了フラグファイルを生成
# 🌟 修正: $3 (PCD出力ディレクトリ) と $4 (フラグファイル名) を結合する 🌟
# $3: /home/hokuyo/colcon_ws/src/hokuyo_navigation2/map
# $4: _processed_2.LIO_RAW_DONE
COMPLETION_FLAG_PATH="${pcd_output_dir}/${flag_file_name}" # 🌟 絶対パスを構築 🌟
echo "Creating completion flag file at: ${COMPLETION_FLAG_PATH}"

touch "${COMPLETION_FLAG_PATH}"

# 正常終了
exit 0