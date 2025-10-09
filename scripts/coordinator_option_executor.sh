#!/bin/bash

# --- 1. ROS環境のセットアップとパスの設定 ---
# Docker環境かどうかを判定する
if [ -n "$DOCKER_ENV" ]; then
    source /opt/ros/humble/setup.bash
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${HOME}/colcon_ws/src/hokuyo_navigation2"
else
    source /opt/ros/humble/setup.bash
    # 環境は server.py の設定に合わせて、スクリプトの実行ディレクトリを想定します
    ROS2_WS="${HOME}/colcon_ws" # ホストOSの実際のパスに置き換えてください
    HOKUYO_NAV2_PKG_PATH="${HOME}/colcon_ws/src/hokuyo_navigation2" # ホストOSの実際のパスに置き換えてください
fi

# スクリプトを実行するディレクトリをワークスペースのルートに変更
cd ${HOKUYO_NAV2_PKG_PATH}
source ${ROS2_WS}/install/setup.bash
source ~/.bashrc

# --- 2. 引数の取得 ---
OPERATION_NUM=$1
use_gnss_switch=$2
stop_uam_manage=$3

if [ -z "$OPERATION_NUM" ]; then
    echo "エラー: オプション番号が指定されていません。"
    exit 1
fi

echo "WIZURGオプション番号: ${OPERATION_NUM}"
echo "use_gnss_switch: ${use_gnss_switch}"
echo "stop_uam_manage: ${stop_uam_manage}"

# --- 3. オプション番号から wizurg_opt 名を決定 ---
case $OPERATION_NUM in
  1) wizurg_opt="control_opt";;
  2) wizurg_opt="sensor_rosbag";;
  6) wizurg_opt="map_opt";;
  7) wizurg_opt="way_opt";;
  8) wizurg_opt="edit_opt";;
  9) wizurg_opt="nav_opt";;
  10) wizurg_opt="plural_opt";;
  *) echo "エラー: サポートされていないオプション番号 ${OPERATION_NUM} です。" ; exit 1;;
esac

wizurg_opt="${wizurg_opt}_lio"
echo "決定された wizurg_opt: ${wizurg_opt}"

# --- 4. CSVファイルの読み込みとパラメータの格納 ---

# wizurg_opts の読み込み
WIZURG_OPT_FILE="./config/wizurg_opts/99_${wizurg_opt}.csv"
if [ ! -f "$WIZURG_OPT_FILE" ]; then
    echo "エラー: オプションファイル ${WIZURG_OPT_FILE} が見つかりません。"
    exit 1
fi
options=($(cat "$WIZURG_OPT_FILE"))

# common_opt の読み込み
COMMON_OPT_FILE="./config/wizurg_opts/99_common_opt.csv"
if [ ! -f "$COMMON_OPT_FILE" ]; then
    echo "エラー: 共通オプションファイル ${COMMON_OPT_FILE} が見つかりません。"
    exit 1
fi
common_options=($(cat "$COMMON_OPT_FILE"))

# options の配列のデータを option_arr に格納
option_arr=()
for i in ${!options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  # 2列目 (パラメータ値) を取得
  option_arr[$j]=$(echo "${options[$i]}" | cut -d ',' -f 2)
  fi
done

# common_options で option_arr を上書き
for i in ${!common_options[@]}; do
 if [ $i -gt 0 ]; then
  j=$((${i}-1))
  option_tmp=$(echo "${common_options[$i]}" | cut -d ',' -f 2)
  if [ "x${option_tmp}" != "xx" ]; then
   option_arr[$j]=${option_tmp}
  fi
fi
done

# 変数への代入 (coordinator.shのロジックを再現)
use_joy="${option_arr[0]}"
mapping="${option_arr[1]}"
navigation="${option_arr[2]}"
sensor="${option_arr[3]}"
icart="${option_arr[4]}"
use_lio="${option_arr[5]}"
use_unity="${option_arr[6]}"
ypspur="${option_arr[7]}"
multi_map="${option_arr[8]}"
mapfile="${option_arr[9]}"
wayfile="${option_arr[10]}"
rosbag_record="${option_arr[11]}"
rosbag_dir="${option_arr[12]}"
loader="${option_arr[13]}"
editor="${option_arr[14]}"

echo "--- 実行パラメータ ---"
echo "use_joy: ${use_joy}"
echo "mapping: ${mapping}"
echo "navigation: ${navigation}"
# ... 他のパラメータも必要に応じて表示 ...
echo "mapfile: ${mapfile}"
echo "wayfile: ${wayfile}"
echo "multi_map: ${multi_map}"
echo "----------------------"

# --- 5. ypspur-coordinator起動（gnome-terminalで実行） ---
if [ "x${ypspur}" = "xtrue" ]; then
    echo "ypspur-coordinator を起動します..."
    gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 icart_mini_drive_launch.xml ; bash"
    # spur待機
    sleep 1
fi

# --- 6. 複数マップと単一マップの処理（ROS2 Launchの実行） ---

# ワークスペースディレクトリに移動 (ros2 launch コマンドのため)
cd ${HOKUYO_NAV2_PKG_PATH}

if [ "x${multi_map}" = "xtrue" ]; then
    # --- 複数マップ処理 ---
    # map_and_waypoints.csv を読み込むロジックを再現
    map_names=($(cat "${HOKUYO_NAV2_PKG_PATH}/config/maps_and_waypoints.csv"))
    Rmapfile=()
    Rwayfile=()
    for i in ${!map_names[@]}; do
        if [ $i -gt 0 ]; then
            j=$((${i}-1))
            Rmapfile[$j]=$(echo "${map_names[$i]}" | cut -d ',' -f 1)
            Rwayfile[$j]=$(echo "${map_names[$i]}" | cut -d ',' -f 2)
        fi
    done

    for i in ${!Rmapfile[@]}; do
        current_map_name=${Rmapfile[$i]}
        current_way_file=${Rwayfile[$i]}
        
        # init_pose.txt の読み込み
        INIT_POSE_FILE="${HOKUYO_NAV2_PKG_PATH}/data/${current_map_name}/init_pose.txt"
        Rinit_pose=$(cat "$INIT_POSE_FILE" | tail -n 1) # 最後の行を取得
        Rpose_arr=( $(echo ${Rinit_pose} | tr -s ',' ' ') )
        Rinitial_pose="${Rpose_arr[0]},${Rpose_arr[1]},${Rpose_arr[2]},${Rpose_arr[3]},${Rpose_arr[4]},${Rpose_arr[5]},${Rpose_arr[6]}"

        echo "--- Multi-Map Loop: ${current_map_name} ---"
        
        # rosbag record
        if [ "x${rosbag_record}" = "xtrue" ]; then
            echo "ros2 bag record を開始します: ${current_map_name}"
            gnome-terminal -- bash -c "sleep 2; cd ${rosbag_dir}; ros2 bag record -a -o ${current_map_name}; bash"
        fi
        
        # hokuyo_nav2_bringup_launch.xml の実行
        LAUNCH_COMMAND="ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml \
            use_joy:=${use_joy} use_mapping:=${mapping} use_navigation:=${navigation} \
            use_loader:=${loader} use_editor:=${editor} use_sensor:=${sensor} use_icart:=${icart} \
            use_lio:=${use_lio} use_unity_sim:=${use_unity} use_gnss_switch:=${use_gnss_switch} \
            stop_uam_manage:=${stop_uam_manage} map_file:=${current_map_name} \
            initial_pose:=\"${Rinitial_pose}\""
            
        gnome-terminal -- bash -c "${LAUNCH_COMMAND} ; bash"
        sleep 2s
        
        # waypoint_manager の実行
        if [ "x${navigation}" = "xtrue" ]; then
             echo "waypoint_manager を実行します: ${current_way_file}.json"
             cd ${HOKUYO_NAV2_PKG_PATH}/waypoints; ros2 run waypoint_manager waypoint_manager -x ${current_way_file}.json once --ros-args -p use_gnss_switch:=${use_gnss_switch} -p cmd_vel_topic:=wizurg/cmd_vel
             cd -
        fi
    done

else
    # --- 単一マップ処理 ---

    # init_pose.txt の読み込み
    INIT_POSE_FILE="${HOKUYO_NAV2_PKG_PATH}/data/${mapfile}/init_pose.txt"
    if [ -f "$INIT_POSE_FILE" ]; then
        init_pose=$(cat "$INIT_POSE_FILE" | tail -n 1) # 最後の行を取得
        pose_arr=( $(echo ${init_pose} | tr -s ',' ' ') )
        initial_pose="${pose_arr[0]},${pose_arr[1]},${pose_arr[2]},${pose_arr[3]},${pose_arr[4]},${pose_arr[5]},${pose_arr[6]}"
    else
        echo "警告: ${INIT_POSE_FILE} が見つかりません。デフォルトの初期姿勢を使用します。"
        initial_pose="0.0,0.0,0.0,0.0,0.0,0.0,1.0"
    fi

    # init_lat_lon_alt.txt の読み込み
    LATLON_FILE="${HOKUYO_NAV2_PKG_PATH}/data/${mapfile}/init_lat_lon_alt.txt"
    if [ -f "$LATLON_FILE" ]; then
        init_latlon=$(cat "$LATLON_FILE" | tail -n 1) # 最後の行を取得
        latlon_arr=( $(echo ${init_latlon} | tr -s ',' ' ') )
        latlon_pose="${latlon_arr[0]},${latlon_arr[1]},${latlon_arr[2]}"
    else
        latlon_pose="0.0,0.0,0.0"
    fi
    
    # rosbag record
    if [ "x${rosbag_record}" = "xtrue" ]; then
        echo "ros2 bag record を開始します: ${mapfile}"
        gnome-terminal -- bash -c "sleep 2; cd ${rosbag_dir}; ros2 bag record -a -o ${mapfile}; bash"
    fi
    
    # hokuyo_nav2_bringup_launch.xml の実行
    LAUNCH_COMMAND="ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml \
        use_joy:=${use_joy} use_mapping:=${mapping} use_navigation:=${navigation} \
        use_loader:=${loader} use_editor:=${editor} use_sensor:=${sensor} use_icart:=${icart} \
        use_lio:=${use_lio} use_unity_sim:=${use_unity} use_gnss_switch:=${use_gnss_switch} \
        stop_uam_manage:=${stop_uam_manage} map_file:=${mapfile} \
        initial_pose:=\"${initial_pose}\" latlon_pose:=\"${latlon_pose}\""
        
    gnome-terminal -- bash -c "${LAUNCH_COMMAND} ; bash"
    sleep 1s

    # rosbag play と waypoint_loader
    if [ "x${loader}" = "xtrue" ]; then
        # rosbag play
        gnome-terminal -- bash -c "cd ${rosbag_dir}; ros2 bag play ${mapfile}"
        # waypoint_manager (loader)
        gnome-terminal -- bash -c "ros2 run waypoint_manager waypoint_manager -w ${HOKUYO_NAV2_PKG_PATH}/waypoints/${wayfile}.json; bash"
    fi

    # waypoint_editor
    if [ "x${editor}" = "xtrue" ]; then
        echo "waypoint_editor を実行します: ${wayfile}.json"
        ros2 run waypoint_manager waypoint_manager -e ${HOKUYO_NAV2_PKG_PATH}/waypoints/${wayfile}.json
    fi
    
    # navigation 実行
    if [ "x${navigation}" = "xtrue" ]; then
        echo "ナビゲーションを開始します: ${wayfile}.json"
        sleep 7.0s
        cd ${HOKUYO_NAV2_PKG_PATH}/waypoints; ros2 run waypoint_manager waypoint_manager -x ${HOKUYO_NAV2_PKG_PATH}/waypoints/${wayfile}.json once --ros-args -p use_gnss_switch:=${use_gnss_switch} -p cmd_vel_topic:=wizurg/cmd_vel
        cd -
    fi
fi

# --- 7. マップ保存 (mappingがtrueの場合、ユーザー入力待ち) ---
if [ "x${mapping}" = "xtrue" ]; then
    # GUIから実行されているため、サーバー側のロジックでマップ保存の確認ダイアログを実装するか、
    # ここでは自動保存（または保存をスキップ）するかの判断が必要です。
    # coordinator.shの「マップを保存しますか？」の処理は、このシェルスクリプトでは再現が難しいため、
    # **GUI (server.py/JavaScript)** でユーザーに確認を取り、別のエンドポイントから
    # `ros2 run nav2_map_server map_saver_cli` を実行することを強く推奨します。

    # 一時的に、ここではマップ保存をスキップするか、自動で実行するかのいずれかを選択する必要があります。
    # GUI側で制御する前提で、ここでは処理をスキップします。
    echo "マップ保存処理はGUI側で実装してください。"
fi

# --- 8. 全ての gnome-terminal ウィンドウの最小化 ---
# GUI (Flask) 側からこのシェルスクリプトが実行されているため、
# xdotool が動作しない環境（例: Dockerコンテナやヘッドレス環境）ではエラーになる可能性があります。
# ホストOSのGUI環境で動作することを前提として、coordinator.sh のロジックをそのまま残します。
for id in $(xdotool search --class "gnome-terminal"); do
    xdotool windowminimize $id
done

echo "WIZURGオプションの実行が完了しました。(${wizurg_opt})"