#!/bin/bash

# ROS 2 環境設定
# ros2 run で実行された場合、このスクリプトは install ディレクトリに配置される。
# そのため、dirname $0 ではソースディレクトリ内の他のスクリプトを見つけられない。
# ros2 pkg prefix を使ってパッケージのパスを特定し、そこからスクリプトのパスを解決する。
PKG_DIR=$(python3 -c "from ament_index_python.packages import get_package_prefix; import os; pkg_name = 'hokuyo_navigation2'; install_prefix = get_package_prefix(pkg_name); ws_root = os.path.dirname(os.path.dirname(install_prefix)); src_path = os.path.join(ws_root, 'src', pkg_name); print(src_path if os.path.isdir(src_path) else install_prefix, end='')")
SCRIPT_DIR="${PKG_DIR}/hokuyo_navigation2/scripts" # install先のスクリプトディレクトリ
source "${SCRIPT_DIR}/setup_ros_env.sh"

SELECTED_OPTION=$(zenity --list --title="Hokuyo Navigation2 Coordinator" --text="実行したい機能を選択してください" \
    --width=800 --height=400 \
    --print-column=1 --separator= \
    --column="番号" --column="機能" --column="説明" \
    1 "start_getting_rosbag" "センサーデータをROS Bagファイルとして記録します。" \
    2 "start_mapping" "ROS Bagから3D/2Dマップを作成します。" \
    3 "start_navigation" "作成したマップとウェイポイントを使用して自律走行します。" 2>/dev/null)
 
if [ -z "$SELECTED_OPTION" ]; then
  echo "終了します。>"
  exit 1
fi

case "$SELECTED_OPTION" in
  1) # start_getting_rosbag
    echo "--> [1] start_getting_rosbag.sh を実行します..."
    gnome-terminal -- bash -c "${SCRIPT_DIR}/start_getting_rosbag.sh; bash"
    ;;

  2) # start_mapping
    echo "--> [2] start_mapping.sh を実行します..."
    MAPPING_OPTION=$(zenity --list --title="マッピングの種類を選択" --text="実行したいマッピング処理を選択してください" \
        --width=600 --height=300 \
        --print-column=1 --separator= \
        --column="ID" --column="機能" --column="説明" \
        "p2o" "P2O" "ROS BagからP2Oで3Dマップを作成します。" \
        "lio_raw" "LIO-RAW" "ROS BagからLIOベースで3Dマップを作成します。" \
        "pcd2pgm" "PCDからPGMへ変換" "3Dマップ(.pcd)を2Dナビゲーションマップ(.pgm)に変換します。" 2>/dev/null)

    if [ -z "$MAPPING_OPTION" ]; then
      echo "マッピング処理がキャンセルされました。"
      exit 1
    fi

    case "$MAPPING_OPTION" in
      "p2o" | "lio_raw")
        IN_BAG_DIR=$(zenity --file-selection --directory --title="入力ROS Bagディレクトリを選択" --filename="${HOKUYO_NAV2_PKG_PATH}/rosbag/" 2>/dev/null | xargs basename)
        MAP_NAME=$(zenity --entry --title="出力マップ名" --text="出力するマップのベース名を入力してください (例: my_map)" 2>/dev/null)

        # 出力するWPファイル名を入力
        WP_FILE=$(zenity --entry --title="出力ウェイポイントファイル名" --text="出力するウェイポイントファイル名を入力 (.json)" --entry-text "${MAP_NAME}_wp.json" 2>/dev/null)

        if [ -n "$IN_BAG_DIR" ] && [ -n "$MAP_NAME" ] && [ -n "$WP_FILE" ]; then # 全ての入力が揃っているか確認
          MAP_DIR="${HOKUYO_NAV2_PKG_PATH}/map"
          WP_DIR="${HOKUYO_NAV2_PKG_PATH}/waypoints"
          FLAG_FILE_NAME="${MAP_NAME}.$(echo "$MAPPING_OPTION" | tr '[:lower:]' '[:upper:]')_DONE"
          # 実行前に古い完了フラグファイルを削除
          if [ -f "${MAP_DIR}/${FLAG_FILE_NAME}" ]; then
            rm -f "${MAP_DIR}/${FLAG_FILE_NAME}"
          fi
          "${SCRIPT_DIR}/start_mapping.sh" "$MAPPING_OPTION" "$IN_BAG_DIR" "$MAP_NAME" "$MAP_DIR" "$WP_DIR" "$FLAG_FILE_NAME" "$WP_FILE"
        else
          echo "入力がキャンセルされました。"
        fi
        ;;
      "pcd2pgm")
        PCD_FILE=$(zenity --file-selection --title="入力PCDファイルを選択" --filename="${HOKUYO_NAV2_PKG_PATH}/map/" --file-filter="*.pcd" 2>/dev/null | xargs basename)
        if [ -n "$PCD_FILE" ]; then
          PGM_NAME=$(zenity --entry --title="出力PGMマップ名" --text="出力するPGMマップのベース名を入力してください (例: my_map_pgm)" 2>/dev/null)
          MAP_DIR="${HOKUYO_NAV2_PKG_PATH}/map"
          WP_FILE=""
          LOOP_WAYPOINTS="false"

          # ウェイポイントを使用するか確認
          if zenity --question --text="ウェイポイントを使用して通行可能領域を指定しますか？" --ok-label="はい" --cancel-label="いいえ"; then
            WP_FILE=$(zenity --file-selection --title="ウェイポイントファイルを選択 (.json)" --filename="${HOKUYO_NAV2_PKG_PATH}/waypoints/" --file-filter="*.json" 2>/dev/null | xargs basename)
            if [ -n "$WP_FILE" ]; then
                # ウェイポイントが選択された場合のみループ設定を質問
                LOOP_FLAG=$(zenity --question --text="ウェイポイントをループとして扱いますか？" --ok-label="はい" --cancel-label="いいえ"; echo $?)
                [ "$LOOP_FLAG" -eq 0 ] && LOOP_WAYPOINTS="true"
            fi
          fi

          FLAG_FILE_NAME="${PGM_NAME}.PCD2PGM_DONE"
          # 実行前に古い完了フラグファイルを削除
          if [ -f "${MAP_DIR}/${FLAG_FILE_NAME}" ]; then
            rm -f "${MAP_DIR}/${FLAG_FILE_NAME}"
          fi
          "${SCRIPT_DIR}/start_mapping.sh" "pcd2pgm" "$PCD_FILE" "$PGM_NAME" "$MAP_DIR" "$WP_FILE" "$LOOP_WAYPOINTS" "$FLAG_FILE_NAME"
        else
          echo "入力がキャンセルされました。"
        fi
        ;;
    esac
    ;;

  3) # start_navigation
    echo "--> [3] start_navigation.sh を実行します..."
    NAV_OPTION=$(zenity --list --title="ナビゲーションの種類を選択" --text="実行したいナビゲーション処理を選択してください" \
        --width=600 --height=300 \
        --print-column=1 --separator= \
        --column="ID" --column="機能" --column="説明" \
        "single_map_gnss" "単一マップ (GNSS)" "指定したマップとウェイポイントでGNSSを利用して走行します。" \
        "single_map_loc" "単一マップ (LIDAR)" "指定したマップとウェイポイントでLIDAR自己位置推定で走行します。" \
        "multi_map" "複数マップ" "CSVファイルで定義された複数マップを連続走行します。" 2>/dev/null)

    if [ -z "$NAV_OPTION" ]; then
      echo "ナビゲーション処理がキャンセルされました。"
      exit 1
    fi

    case "$NAV_OPTION" in
      "single_map_gnss" | "single_map_loc")
        MAP_FILE=$(zenity --file-selection --title="マップファイルを選択 (.yaml)" --filename="${HOKUYO_NAV2_PKG_PATH}/map/" --file-filter="*.yaml" 2>/dev/null | xargs basename)
        if [ -n "$MAP_FILE" ]; then
          WP_FILE=$(zenity --file-selection --title="ウェイポイントファイルを選択 (.json)" --filename="${HOKUYO_NAV2_PKG_PATH}/waypoints/" --file-filter="*.json" 2>/dev/null | xargs basename)
          if [ -n "$WP_FILE" ]; then
            MAP_FILE_BASE=${MAP_FILE%.yaml} # .yaml拡張子を削除
            WP_FILE_BASE=${WP_FILE%.json}   # .json拡張子を削除
            "${SCRIPT_DIR}/start_navigation.sh" "$NAV_OPTION" "$MAP_FILE_BASE" "$WP_FILE_BASE"
          fi
        else
          echo "入力がキャンセルされました。"
        fi
        ;;
      "multi_map")
        CSV_FILE=$(zenity --file-selection --title="CSV設定ファイルを選択" --filename="${HOKUYO_NAV2_PKG_PATH}/config/" --file-filter="*.csv" 2>/dev/null | xargs basename)
        if [ -n "$CSV_FILE" ]; then
          "${SCRIPT_DIR}/start_navigation.sh" "multi_map" "$CSV_FILE"
        else
          echo "入力がキャンセルされました。"
        fi
        ;;
    esac
    ;;

  *)
    echo "未実装のオプションです: $SELECTED_OPTION"
    ;;
esac

echo "処理を終了します。"
exit 0