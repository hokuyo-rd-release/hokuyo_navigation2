#!/bin/bash
# --------------------------------------------------------------------------
# ROS 2 環境設定
# --------------------------------------------------------------------------

# Docker環境かどうかを判定する
# コンテナの起動時に -e DOCKER_ENV=1 を指定することで、Docker環境とみなすことができます。
if [ -n "$DOCKER_ENV" ]; then
    source /opt/ros/humble/setup.bash
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${HOME}/colcon_ws/src/hokuyo_navigation2"
else
    source /opt/ros/humble/setup.bash
    # ワークスペースのパスもホストOSのものに合わせる
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${HOME}/colcon_ws/src/hokuyo_navigation2"
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
        # $2: 入力ROS Bag名 (ディレクトリ名), $3: 出力ROS Bag名 (ディレクトリ名)
        inbagname="${HOKUYO_NAV2_PKG_PATH}/rosbag/$2" # 実行時に $2 は Bagのベース名 (例: raw_bag)
        outbagname="$3"                               # 実行時に $3 は 出力ディレクトリ名 (例: sync_bag)
        
        # scripts/get_rosbag.bash は $1(inbagpath) $2(outbagname) を受け取る
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/get_rosbag.bash ${inbagname} ${outbagname} ; bash"; exit
        
        ;;

    "p2o")
        echo "--> [2] p2o でマッピングを開始します。"
        # server.pyから渡される新しい引数:
        # $2: 入力ROS Bag名 (ディレクトリ名)
        # $3: 出力マップ名 (pcd名)
        # $4: MAP_DIR (完了フラグとPCDの出力先ディレクトリ)
        # $5: FLAG_FILE_NAME (完了フラグファイル名)
        
        inbagname="$2"                                # 例: sync_bag
        p2omapname="$3"                               # 例: final_map
        pcd_output_dir="$4"                           # 例: /home/hokuyo/colcon_ws/src/hokuyo_navigation2/map
        flag_file_name="$5"                           # 例: final_map.P2O_DONE
        
        # scripts/hokuyo_slam.bash に新しい引数 ($4, $5) を追加して渡す
        # hokuyo_slam.bash が以下の引数を受け取るように変更が必要です:
        # $1(inbagpath) $2(p2omapname) $3(pcd_output_dir) $4(flag_file_name)
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/hokuyo_slam.bash ${inbagname} ${p2omapname} ${pcd_output_dir} ${flag_file_name}; bash"; exit
        
        ;;

    "lio_raw")
        echo "--> [3] LIO-RAW マッピングを開始します。"
        # ここに LIO-RAW マッピングのための ROS 2 起動コマンドを記述
        inbagname="$2"
        liomapname="$3"
        pcd_output_dir="$4"
        flag_file_name="$5"
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/lio_raw.bash ${inbagname} ${liomapname} ${pcd_output_dir} ${flag_file_name} ; bash"; exit
        # 例: ros2 launch hokuyo_navigation2 lio_raw_mapping_launch.py

        ;;
    
    "pcd2pgm")
        echo "--> [4] PCDファイルからPGMマップへの変換を開始します。"
        # ここに PCD to PGM 変換のための処理を記述
        # 例: ros2 run map_server map_saver_cli -f map_name --pcd map.pcd
        
        ;;

    *)
        # 引数が指定されない、または上記以外の場合のデフォルト処理
        echo "--> [X] 無効なマッピングオプションです: ${MAPPING_OPTION}"
        echo "    使用可能なオプション: sync, p2o, lio_raw, pcd2pgm"
        exit 1
        ;;
esac

echo "--- 処理終了 ---"