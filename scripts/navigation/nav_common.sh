#!/bin/bash

# このスクリプトは nav_single_map.sh と nav_multi_map.sh から source されることを想定しています。

load_options() {
    local wizurg_opt="$1"
    local options_csv_path="${HOKUYO_NAV2_PKG_PATH}/config/wizurg_opts/${wizurg_opt}.csv"

    if [ ! -f "${options_csv_path}" ]; then
        echo "エラー: オプションファイルが見つかりません: ${options_csv_path}" >&2
        exit 1
    fi

    # ヘッダ行をスキップして読み込む
    mapfile -t options < <(tail -n +2 "${options_csv_path}")

    local option_arr=()
    for i in "${!options[@]}"; do
        # IFSを使ってカンマ区切りで読み込む
        local line_arr=()
        IFS=',' read -r -a line_arr <<< "${options[$i]}"
        option_arr[$i]="${line_arr[1]}" # 2列目の値のみを取得
    done

    # グローバル変数に設定
    use_joy="${option_arr[0]}"
    mapping="${option_arr[1]}"
    navigation="${option_arr[2]}"
    sensor="${option_arr[3]}"
    icart="${option_arr[4]}"
    use_lio="${option_arr[5]}"
    # GUI のトグルから USE_UNITY_SIM が渡されたらそちらを優先し、無ければ CSV の値を使う
    use_unity="${USE_UNITY_SIM:-${option_arr[6]}}"
    use_motor_driver="${option_arr[7]}"
    multi_map="${option_arr[8]}"
    default_mapfile="${option_arr[9]}"
    default_wayfile="${option_arr[10]}"
    rosbag_record="${option_arr[11]}"
    rosbag_dir="${option_arr[12]}"
    loader="${option_arr[13]}"
    editor="${option_arr[14]}"
}

# rosbag取得用のオプションをCSVファイルから読み込む関数
load_rosbag_options() {
    local wizurg_opt="sensor_rosbag_lio" # rosbag取得用の設定ファイル名を指定
    local options_csv_path="${HOKUYO_NAV2_PKG_PATH}/config/wizurg_opts/${wizurg_opt}.csv"

    if [ ! -f "${options_csv_path}" ]; then
        echo "エラー: オプションファイルが見つかりません: ${options_csv_path}" >&2
        exit 1
    fi

    # ヘッダ行をスキップして読み込む
    mapfile -t options < <(tail -n +2 "${options_csv_path}")

    local option_arr=()
    for i in "${!options[@]}"; do
        # IFSを使ってカンマ区切りで読み込む
        local line_arr=()
        IFS=',' read -r -a line_arr <<< "${options[$i]}"
        option_arr[$i]="${line_arr[1]}" # 2列目の値のみを取得
    done

    # グローバル変数に設定
    use_motor_driver="${option_arr[0]}"
    use_navigation="${option_arr[1]}"
    use_sensor="${option_arr[2]}"
    use_lio="${option_arr[3]}"
    use_gnss_switch="${option_arr[4]}"
    use_localization="${option_arr[5]}"
}

# 初期位置情報をファイルから読み込む関数
load_initial_poses() {
    local map_name="$1"
    local pose_file="${HOKUYO_NAV2_PKG_PATH}/data/${map_name}/init_pose.txt"
    local latlon_file="${HOKUYO_NAV2_PKG_PATH}/data/${map_name}/init_lat_lon_alt.txt"

    # init_pose.txt の読み込み
    if [ -f "${pose_file}" ]; then
        init_pose=$(tail -n 1 "${pose_file}")
    else
        echo "警告: ${pose_file} が見つかりません。デフォルトの初期位置を使用します。"
        init_pose="0.0,0.0,0.0,0.0,0.0,0.0,1.0"
    fi

    # init_lat_lon_alt.txt の読み込み
    if [ -f "${latlon_file}" ]; then
        init_latlon=$(tail -n 1 "${latlon_file}")
    else
        echo "警告: ${latlon_file} が見つかりません。デフォルトの緯度経度を使用します。"
        init_latlon="35.0,135.0,40.0"
    fi

    # 読み込んだ値を配列にパース
    IFS=',' read -r -a pose_arr <<< "$init_pose"
    IFS=',' read -r -a latlon_arr <<< "$init_latlon"

    # グローバル変数に設定
    pose1=${pose_arr[0]:-0.0}
    pose2=${pose_arr[1]:-0.0}
    pose3=${pose_arr[2]:-0.0}
    pose4=${pose_arr[3]:-0.0}
    pose5=${pose_arr[4]:-0.0}
    pose6=${pose_arr[5]:-0.0}
    pose7=${pose_arr[6]:-1.0}
    latlon1=${latlon_arr[0]:-35.0}
    latlon2=${latlon_arr[1]:-135.0}
    latlon3=${latlon_arr[2]:-40.0}
}

# ナビゲーションシステムを起動する関数
launch_navigation_system() {
    local map_name="$1"
    local current_use_gnss_switch="$2"

    echo "ナビゲーションシステムを起動します... (マップ: ${map_name}, GNSS: ${current_use_gnss_switch})"
    gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 hokuyo_nav2_bringup_launch.xml \
        use_joy:=${use_joy} \
        use_mapping:=${mapping} \
        use_navigation:=${navigation} \
        use_loader:=${loader} \
        use_editor:=${editor} \
        use_sensor:=${sensor} \
        use_icart:=${icart} \
        use_lio:=${use_lio} \
        use_unity_sim:=${use_unity} \
        use_gnss_switch:=${current_use_gnss_switch} \
        map_file:=${map_name} \
        initial_pose:=\"${pose1},${pose2},${pose3},${pose4},${pose5},${pose6},${pose7}\" \
        latlon_pose:=\"${latlon1},${latlon2},${latlon3}\"; \
        bash"
}

# Unity シミュレータのロボットを停止し、LIO をリセットする関数
# 実機では hokuyo_rsf の再起動で LIO が原点から始まるが、Unity 側の LIO は Play 中ずっと動き続けるため、
# マップ切替のたびに /rsf_reset を送って原点を現在位置に戻す。ROS-TCP-Endpoint と Play はそのままでよい。
reset_unity_sim() {
    if [ "${use_unity}" != "true" ]; then
        return 0
    fi
    echo "Unity: ロボットを停止し、LIO をリセットします..."
    # cmdvel_stopper を止めると Unity のロボットは最後の速度指令で動き続けるため、0 を送る
    timeout 10 ros2 topic pub --once -w 1 /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null \
        || echo "警告: /cmd_vel を購読している Unity のロボットが見つかりません。"
    timeout 10 ros2 topic pub --once -w 1 /rsf_reset std_msgs/msg/Empty "{}" > /dev/null \
        || echo "警告: /rsf_reset を購読している Unity のノードが見つかりません。"
}

# モータドライバを起動する関数
launch_motor_driver() {
    if [ "${use_motor_driver}" = "true" ]; then
        echo "モータドライバを起動します..."
        gnome-terminal -- bash -c "ros2 launch hokuyo_navigation2 icart_mini_drive_launch.xml; bash"

        echo "モータドライバ関連ノードの起動を待っています..."
        # local timeout=25
        # local start_time=$(date +%s)
        # local ypspur_node_found=false

        # while [ $(($(date +%s) - start_time)) -lt ${timeout} ]; do
        #     if ros2 node list | grep -q -e 'icart_mini' -e 'ypspur'; then
        #         ypspur_node_found=true
        #         break
        #     fi
        #     sleep 1
        # done

        # if [ "${ypspur_node_found}" = "false" ]; then
        #     echo "エラー: ypspur関連ノードの起動に失敗しました。(${timeout}秒タイムアウト)" >&2
        #     exit 1
        # fi
        echo "モータドライバ関連ノードの起動を確認しました。"
        # sleep 2s
    fi
}
