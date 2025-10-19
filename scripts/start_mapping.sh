#!/bin/bash
# --------------------------------------------------------------------------
# start_mapping.sh: Webインターフェースからのマッピング/変換処理を起動する
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# ROS 2 環境設定
# --------------------------------------------------------------------------

# Docker環境かどうかを判定する
# コンテナの起動時に -e DOCKER_ENV=1 を指定することで、Docker環境とみなすことができます。
if [ -n "$DOCKER_ENV" ]; then
    source /opt/ros/humble/setup.bash
    cd /home/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="/home/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${ROS2_WS}/src/hokuyo_navigation2"
else
    source /opt/ros/humble/setup.bash
    # ワークスペースのパスもホストOSのものに合わせる
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${ROS2_WS}/src/hokuyo_navigation2"
fi

# コマンドライン引数を取得
MAPPING_OPTION="$1"
echo "Mapping Option: ${MAPPING_OPTION}"

# --------------------------------------------------------------------------
# 処理の分岐
# --------------------------------------------------------------------------

case "${MAPPING_OPTION}" in
    "sync")
        echo "--> [1] トピック同期処理を開始します。"
        # $2: 入力ROS Bag名 (ディレクトリ名/ファイル名)
        # $3: 出力ROS Bag名 (ディレクトリ名)
        
        # NOTE: get_rosbag.bash がフルパスを期待する場合があるため、/rosbag/ を付けて渡す
        inbagname="${HOKUYO_NAV2_PKG_PATH}/rosbag/$2" 
        outbagname="$3"                               

        # scripts/get_rosbag.bash を gnome-terminal で実行
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/get_rosbag.bash ${inbagname} ${outbagname} ; bash"; exit
        
        ;;

    "p2o")
        echo "--> [2] p2o でマッピングを開始します。"
        # $2: 入力ROS Bag名 (ディレクトリ名)
        # $3: 出力マップ名 (pcd名)
        # $4: MAP_DIR (完了フラグとPCDの出力先ディレクトリ)
        # $5: FLAG_FILE_NAME (完了フラグファイル名)
        
        inbagname="$2"
        p2omapname="$3"
        pcd_output_dir="$4"
        wp_output_dir="$5"
        flag_file_name="$6"
        
        # scripts/hokuyo_slam.bash に引数を渡して実行
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/hokuyo_slam.bash ${inbagname} ${p2omapname} ${pcd_output_dir} ${flag_file_name} ${wp_output_dir}; bash"; exit
        
        ;;

    "lio_raw")
        echo "--> [3] LIO-RAW マッピングを開始します。"
        # $2: 入力ROS Bag名 (ディレクトリ名)
        # $3: 出力マップ名 (pcd名)
        # $4: MAP_DIR (完了フラグとPCDの出力先ディレクトリ)
        # $5: FLAG_FILE_NAME (完了フラグファイル名)
        
        inbagname="$2"
        liomapname="$3"
        pcd_output_dir="$4"
        wp_output_dir="$5"
        flag_file_name="$6"
        
        # scripts/lio_raw.bash に引数を渡して実行
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/lio_raw.bash ${inbagname} ${liomapname} ${pcd_output_dir} ${wp_output_dir} ${flag_file_name} ; bash"; exit
        
        ;;
    
    "pcd2pgm")
        echo "--> [4] PCDファイルからPGMマップへの変換を開始します。"
        
        # $2: 入力PCDファイル名 (例: my_map.pcd)
        # $3: 出力PGMベース名 (例: my_map_pgm)
        # $4: MAP_DIR (PCD, PGM, YAMLの出力先ディレクトリ)
        # $5: FLAG_FILE_NAME (完了フラグファイル名)
        
        input_pcd_filename="$2"
        output_pgm_name="$3"
        pgm_output_dir="$4"
        flag_file_name="$5"
        
        # scripts/pcd2pgm.bash に引数を渡して実行
        # 引数にスペースが含まれる可能性を考慮し、ダブルクォーテーションで囲むのが安全
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/pcd2pgm.bash \
            \"${input_pcd_filename}\" \
            \"${output_pgm_name}\" \
            \"${pgm_output_dir}\" \
            \"${flag_file_name}\" ; bash"; exit
        
        ;;

    *)
        # 引数が指定されない、または上記以外の場合のデフォルト処理
        echo "--> [X] 無効なマッピングオプションです: ${MAPPING_OPTION}"
        echo "    使用可能なオプション: sync, p2o, lio_raw, pcd2pgm"
        exit 1
        ;;
esac

echo "--- 処理終了 ---"