#!/bin/bash
# --------------------------------------------------------------------------
# pcd2pgm.bash: PCDファイルからPGMマップを生成するPythonスクリプトを実行する
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# ROS 2 環境設定
# --------------------------------------------------------------------------
source "$(dirname "$0")/../setup_ros_env.sh"

# --------------------------------------------------------------------------
# 引数の取得 (引数順が変更されています)
# --------------------------------------------------------------------------
# start_mapping.sh から渡される引数順 (変更後):
# $1: input_pcd_filename (例: my_map.pcd)
# $2: output_map_name (PGMファイルのベース名, 例: my_map_pgm)
# $3: MAP_DIR (PCD, PGM, YAMLの出力先ディレクトリ)
# $4: waypoint_filename (例: my_waypoints.json)
# $5: loop_waypoints_flag (ウェイポイントをループとして扱うか: true/false) <-- 【新規/変更】
# $6: FLAG_FILE_NAME (完了フラグファイル名, 例: my_map_pgm.PCD2PGM_DONE) <-- 【変更】

input_pcd_filename="$1"
output_map_name="$2"
pgm_output_dir="$3"
waypoint_filename="$4"
loop_waypoints_flag="$5"  # 新しい5番目の引数
flag_file_name="$6"       # 6番目にずらされた引数

# --------------------------------------------------------------------------
# パス設定
# --------------------------------------------------------------------------
PCD_FILE_PATH="${pgm_output_dir}/${input_pcd_filename}"
# ウェイポイントファイルが存在するディレクトリのパス
WP_DIR="${HOKUYO_NAV2_PKG_PATH}/waypoints" 
OUTPUT_BASE_PATH="${pgm_output_dir}/${output_map_name}"
PYTHON_SCRIPT="${HOKUYO_NAV2_PKG_PATH}/src/pcd2pgm_converter.py"
COMPLETION_FLAG_PATH="${pgm_output_dir}/${flag_file_name}"

echo "--> [1] PCDファイルからPGMマップへのPython変換を開始します。"
echo "    入力PCD: ${PCD_FILE_PATH}"
echo "    出力PGMベース名: ${OUTPUT_BASE_PATH}"
echo "    ウェイポイントファイル名: ${waypoint_filename}"
echo "    ループ処理フラグ: ${loop_waypoints_flag}"
echo "    完了フラグ: ${COMPLETION_FLAG_PATH}"

# 1. 入力ファイルとPythonスクリプトの存在確認
if [ ! -f "${PCD_FILE_PATH}" ]; then
    echo "ERROR: 入力PCDファイルが見つかりません: ${PCD_FILE_PATH}"
    exit 1
fi
if [ ! -f "${PYTHON_SCRIPT}" ]; then
    echo "ERROR: 変換スクリプトが見つかりません: ${PYTHON_SCRIPT}"
    exit 1
fi

# ウェイポイントファイルパスの設定と存在チェック
WP_FULL_PATH=""
if [ -n "${waypoint_filename}" ]; then
    # ファイル名から前後の空白を削除
    CLEANED_WP_FILENAME=$(echo "${waypoint_filename}" | tr -d '\040\011\012\015')
    
    # パスを結合
    WP_FULL_PATH="${WP_DIR}/${CLEANED_WP_FILENAME}"
    
    # ファイルが存在しない場合は、Pythonに渡さない
    if [ ! -f "${WP_FULL_PATH}" ]; then
        echo "WARNING: ウェイポイントファイルが見つかりません。ウェイポイントなしで続行します: ${WP_FULL_PATH}"
        WP_FULL_PATH=""
    fi
fi

# ウェイポイントパスを条件付きで渡すための引数文字列
CMD_WAYPOINTS=""
if [ -n "${WP_FULL_PATH}" ]; then
    CMD_WAYPOINTS="--waypoints_file ${WP_FULL_PATH}"
fi

# 【新規】ループフラグの引数文字列
CMD_LOOP_FLAG=""
# 引数 $5 が "true" や "True" などの場合に --loop_waypoints フラグを設定する
# Python側は action="store_true" なので、フラグが存在するだけで True になる
if [[ "${loop_waypoints_flag}" =~ ^[Tt]rue$ ]]; then
    CMD_LOOP_FLAG="--loop_waypoints"
fi

# 2. pcd2pgm_converter.py の実行
echo "Running: python3 ${PYTHON_SCRIPT} \
--thre_z_min 0.5 \
--thre_z_max 7.0 \
--flag_pass_through False \
--thre_radius 0.1 \
--map_resolution 0.05 \
--thres_point_count 1 \
--odom_to_lidar_odom 0.0 0.0 0.0 0.0 0.0 0.0 \
${CMD_WAYPOINTS} \
${CMD_LOOP_FLAG} \
\"${PCD_FILE_PATH}\" \
\"${OUTPUT_BASE_PATH}\""

# 【重要】Pythonの引数規則に従い、位置引数（PCD/OUTPUTパス）を最後に配置する
python3 "${PYTHON_SCRIPT}" \
    --thre_z_min -1.0 \
    --thre_z_max 7.0 \
    --flag_pass_through False \
    --thre_radius 0.1 \
    --map_resolution 0.05 \
    --thres_point_count 1 \
    --odom_to_lidar_odom 0.0 0.0 0.0 0.0 0.0 0.0 \
    ${CMD_WAYPOINTS} \
    ${CMD_LOOP_FLAG} \
    "${PCD_FILE_PATH}" \
    "${OUTPUT_BASE_PATH}"

# 3. 正常終了チェック
CONVERT_STATUS=$?

if [ $CONVERT_STATUS -ne 0 ]; then
    echo "ERROR: pcd2pgm_converter.py がエラーコード $CONVERT_STATUS で終了しました。完了フラグは出力されません。"
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


echo "PCD to PGM 変換が正常に完了しました。"

# 5. 処理完了フラグファイルを生成
echo "Creating completion flag file at: ${COMPLETION_FLAG_PATH}"

# ファイルをタッチし、全ユーザーが読み書きできるようにパーミッションを設定
touch "${COMPLETION_FLAG_PATH}"

echo "Waiting 5 seconds for file system sync..."
sleep 5 

# 正常終了
exit 0