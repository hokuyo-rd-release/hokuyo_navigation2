#!/bin/bash
# --------------------------------------------------------------------------
# start_navigation.sh: Webインターフェースからのナビゲーション処理を起動する
# --------------------------------------------------------------------------

# --------------------------------------------------------------------------
# ROS 2 環境設定
# --------------------------------------------------------------------------

source "$(dirname "$0")/setup_ros_env.sh"

# コマンドライン引数を取得
NAV_OPTION="$1"
echo "Navigation Option: ${NAV_OPTION}"

# --------------------------------------------------------------------------
# 処理の分岐
# --------------------------------------------------------------------------

case "${NAV_OPTION}" in
    "single_map_gnss")
        echo "--> [1] single_map gnss でNavigationを開始します。"
        # $2: mapfile
        # $3: wpfile
        mapfile="$2"
        wpfile="$3"
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/navigation/nav_single_map.sh ${mapfile} ${wpfile} gnss; bash"; exit
        ;;

    "single_map_loc")
        echo "--> [2] single_map loc でNavigationを開始します。"
        # $2: mapfile
        # $3: wpfile
        mapfile="$2"
        wpfile="$3"
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/navigation/nav_single_map.sh ${mapfile} ${wpfile} loc; bash"; exit
        ;;

    "multi_map")
        echo "--> [3] multi_map でNavigationを開始します。"
        # $2: csv_file
        csv_file="$2"
        gnome-terminal -- bash -c "cd ${HOKUYO_NAV2_PKG_PATH}; scripts/navigation/nav_multi_map.sh ${csv_file}; bash"; exit
        ;;

    *)
        # 引数が指定されない、または上記以外の場合のデフォルト処理
        echo "--> [X] 無効なNavigationオプションです: ${NAV_OPTION}"
        echo "    使用可能なオプション: single_map_gnss, single_map_loc, multi_map"
        exit 1
        ;;
esac

echo "--- 処理終了 ---"