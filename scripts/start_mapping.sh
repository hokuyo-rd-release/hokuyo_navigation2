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
# 実行ログの出力先 (ブラウザGUIから渡される)
# --------------------------------------------------------------------------
# hokuyo_navigation2_gui の server.py が環境変数 HOKUYO_GUI_LOG_FILE に
# ログファイルのパスを入れて起動する。実処理は別ターミナルで動くため、
# ここで tee を挟んでおかないとブラウザ側に何も表示できない。
GUI_LOG_FILE="${HOKUYO_GUI_LOG_FILE:-}"

# 実処理コマンドを、ログ記録付きで実行する。
# $1: bash に渡すコマンド文字列
run_mapping_command() {
    local inner_command="$1"
    # 終了コードを目印付きで残す。GUI はこの行で成功・失敗を判定する。
    # 本処理はサブシェル () で囲む。{} だと処理側の exit が
    # 終了コードを書き出す前にシェルごと終了させてしまう。
    local wrapped="( ${inner_command} ) 2>&1 ; echo \"[HOKUYO_GUI] exit_code=\$?\""

    if [ -n "${GUI_LOG_FILE}" ]; then
        # 本処理と終了コードの両方をログに残すため、全体を { } でまとめてから
        # tee に渡す。まとめないと最後の echo だけがログに書かれてしまう。
        wrapped="{ ${wrapped} ; } | tee -a \"${GUI_LOG_FILE}\""
    fi

    # 画面のないサーバやコンテナでは端末を開けないため、
    # gnome-terminal が使えるかどうかで実行方法を切り替える。
    # (この関数は server.py がバックグラウンドスレッドで起動しているため、
    #  端末なしでその場で実行しても GUI の応答は止まらない)
    if command -v gnome-terminal >/dev/null 2>&1 \
       && { [ -n "${DISPLAY}" ] || [ -n "${WAYLAND_DISPLAY}" ]; }; then
        # 従来どおり別ターミナルを開き、処理後もターミナルを残す。
        if gnome-terminal -- bash -c "${wrapped}; bash"; then
            return 0
        fi
        echo "WARNING: ターミナルを開けなかったため、ターミナルなしで実行します。"
    else
        echo "NOTE: 画面が利用できないため、ターミナルを開かずに実行します。"
    fi

    # ターミナルを使わない場合、tee の出力がこのスクリプトの標準出力にも流れる。
    # server.py はその標準出力も同じログファイルに書き写すため、
    # 捨てておかないとログが二重に記録されてしまう。
    if [ -n "${GUI_LOG_FILE}" ]; then
        bash -c "${wrapped}" >/dev/null
    else
        bash -c "${wrapped}"
    fi
}

# --------------------------------------------------------------------------
# 処理の分岐
# --------------------------------------------------------------------------

case "${MAPPING_OPTION}" in
    "p2o")
        echo "--> [1] p2o でマッピングを開始します。"
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
        config_file="$7"
        
        # scripts/hokuyo_slam.bash に引数を渡して実行
        # NOTE: scripts/hokuyo_slam.bash の引数の順番も確認し、適切に渡すこと
        run_mapping_command "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/mapping/hokuyo_slam.bash \
            \"${inbagname}\" \
            \"${p2omapname}\" \
            \"${pcd_output_dir}\" \
            \"${flag_file_name}\" \
            \"${wp_output_dir}\" \
            \"${config_file}\""; exit
        
        ;;

    "lio_raw")
        echo "--> [2] LIO-RAW マッピングを開始します。"
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
        config_file="$7"
        
        # scripts/lio_raw.bash に引数を渡して実行
        run_mapping_command "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/mapping/lio_raw.bash \
            \"${inbagname}\" \
            \"${liomapname}\" \
            \"${pcd_output_dir}\" \
            \"${wp_output_dir}\" \
            \"${flag_file_name}\" \
            \"${config_file}\""; exit
        
        ;;
    
    "pcd2pgm")
        echo "--> [3] PCDファイルからPGMマップへの変換を開始します。"
        
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
        config_file="$8"         # 👈 追加

        
        # scripts/pcd2pgm.bash に引数を渡して実行
        # pcd2pgm.bash の引数順: $1(PCD_FILE), $2(MAP_NAME), $3(MAP_DIR), $4(WP_FILE), $5(LOOP_FLAG), $6(FLAG_FILE)
        run_mapping_command "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/mapping/pcd2pgm.bash \
            \"${input_pcd_filename}\" \
            \"${output_pgm_name}\" \
            \"${pgm_output_dir}\" \
            \"${waypoint_filename}\" \
            \"${loop_waypoints_flag}\" \
            \"${flag_file_name}\" \
            \"${config_file}\""; exit
        
        ;;

    *)
        # 引数が指定されない、または上記以外の場合のデフォルト処理
        echo "--> [X] 無効なマッピングオプションです: ${MAPPING_OPTION}"
        echo "    使用可能なオプション: p2o, lio_raw, pcd2pgm"
        exit 1
        ;;
esac

echo "--- 処理終了 ---"