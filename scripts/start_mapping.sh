#!/bin/bash
# --------------------------------------------------------------------------
# start_mapping.sh: Webインターフェースからのマッピング/変換処理を起動する
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# ROS 2 環境設定
# --------------------------------------------------------------------------

source "$(dirname "$0")/setup_ros_env.sh"

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
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/mapping/sync_topic.bash ${inbagname} ${outbagname} ; bash"; exit
        
        ;;

    "p2o")
        echo "--> [2] p2o でマッピングを開始します。"
        # $2: 入力ROS Bag名 (ディレクトリ名)
        # $3: 出力マップ名 (pcd名)
        # $4: MAP_DIR (完了フラグとPCDの出力先ディレクトリ)
        # $5: WP_DIR (ウェイポイントの出力先ディレクトリ)
        # $6: FLAG_FILE_NAME (完了フラグファイル名)
        
        inbagname="$2"
        p2omapname="$3"
        pcd_output_dir="$4"
        wp_output_dir="$5"
        flag_file_name="$6"
        
        # scripts/hokuyo_slam.bash に引数を渡して実行
        # NOTE: scripts/hokuyo_slam.bash の引数の順番も確認し、適切に渡すこと
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/mapping/hokuyo_slam.bash ${inbagname} ${p2omapname} ${pcd_output_dir} ${flag_file_name} ${wp_output_dir}; bash"; exit
        
        ;;

    "lio_raw")
        echo "--> [3] LIO-RAW マッピングを開始します。"
        # $2: 入力ROS Bag名 (ディレクトリ名)
        # $3: 出力マップ名 (pcd名)
        # $4: MAP_DIR (完了フラグとPCDの出力先ディレクトリ)
        # $5: WP_DIR (ウェイポイントの出力先ディレクトリ)
        # $6: FLAG_FILE_NAME (完了フラグファイル名)
        
        inbagname="$2"
        liomapname="$3"
        pcd_output_dir="$4"
        wp_output_dir="$5"
        flag_file_name="$6"
        
        # scripts/lio_raw.bash に引数を渡して実行
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/mapping/lio_raw.bash ${inbagname} ${liomapname} ${pcd_output_dir} ${wp_output_dir} ${flag_file_name} ; bash"; exit
        
        ;;
    
    "pcd2pgm")
        echo "--> [4] PCDファイルからPGMマップへの変換を開始します。"
        
        # Webインターフェースからの引数を取得
        # $2: 入力PCDファイル名 (例: my_map.pcd)
        # $3: 出力PGMベース名 (例: my_map_pgm)
        # $4: MAP_DIR (PCD, PGM, YAMLの出力先ディレクトリ)
        # $5: WAYPOINT_FILENAME (ウェイポイントファイル名)
        # $6: LOOP_WAYPOINTS_FLAG (ループ処理フラグ: true/false) <--- 【新規】
        # $7: FLAG_FILE_NAME (完了フラグファイル名) <--- 【インデックス変更】
        
        input_pcd_filename="$2"
        output_pgm_name="$3"
        pgm_output_dir="$4"
        waypoint_filename="$5" 
        loop_waypoints_flag="$6" # 👈 新しい引数
        flag_file_name="$7"      # 👈 インデックスが変更

        
        # scripts/pcd2pgm.bash に引数を渡して実行
        # pcd2pgm.bash の引数順: $1(PCD_FILE), $2(MAP_NAME), $3(MAP_DIR), $4(WP_FILE), $5(LOOP_FLAG), $6(FLAG_FILE)
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/mapping/pcd2pgm.bash \
            \"${input_pcd_filename}\" \
            \"${output_pgm_name}\" \
            \"${pgm_output_dir}\" \
            \"${waypoint_filename}\" \
            \"${loop_waypoints_flag}\" \
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