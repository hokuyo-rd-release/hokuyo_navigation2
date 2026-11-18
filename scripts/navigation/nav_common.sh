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
    # --once だと送った直後に終了し、ROS-TCP-Endpoint 経由では届かないことがあるので、数回送る
    echo "Unity: ロボットを停止します..."
    # cmdvel_stopper を止めると Unity のロボットは最後の速度指令で動き続けるため、0 を送る
    timeout 10 ros2 topic pub -t 3 -r 5 -w 1 /cmd_vel geometry_msgs/msg/Twist "{}" < /dev/null > /dev/null \
        || echo "警告: /cmd_vel を購読している Unity のロボットが見つかりません。"

    # リセットが届いたかは、LIO のオドメトリが原点付近から出直したことで確かめる。届くまで送り直す
    # (リセットされないまま次のマップを起動すると、前のマップの座標のまま自己位置推定が始まって地図から外れる)
    local attempt=1
    while true; do
        echo "Unity: LIO をリセットします (${attempt} 回目)..."
        timeout 10 ros2 topic pub -t 3 -r 5 -w 1 /rsf_reset std_msgs/msg/Empty "{}" < /dev/null > /dev/null \
            || echo "警告: /rsf_reset を購読している Unity のノードが見つかりません。Unity が Play 中か確認してください。"
        if wait_for_unity_lio_near_origin; then
            echo "Unity: LIO のリセットを確認しました。"
            return 0
        fi
        echo "警告: LIO のオドメトリが原点に戻っていません。リセットを送り直します。"
        attempt=$(( attempt + 1 ))
    done
}

# LIO のオドメトリが原点付近 (UNITY_LIO_ORIGIN_TOLERANCE 以内) で届くまで待つ関数。
# リセット直後は重力推定が終わるまでオドメトリが出ないので、UNITY_LIO_READY_TIMEOUT まで待つ
wait_for_unity_lio_near_origin() {
    local start=${SECONDS}
    local pos
    while (( SECONDS - start < UNITY_LIO_READY_TIMEOUT )); do
        pos=$(timeout 5 ros2 topic echo --once --field pose.pose.position /rsf/lio_imu_rate_odom < /dev/null 2> /dev/null) || continue
        if echo "${pos}" | awk -v tol="${UNITY_LIO_ORIGIN_TOLERANCE}" \
            '/^x:/ {x=$2} /^y:/ {y=$2} END {exit !(x != "" && sqrt(x*x + y*y) < tol)}'; then
            return 0
        fi
        sleep 0.5
    done
    return 1
}

# --- マップ切替時の待ち時間 [秒] ---
# ノード終了待ちの上限。ros2 launch は子ノードを SIGINT → SIGTERM (5秒後) → SIGKILL (さらに5秒後) の順に止めるので、それより長くする
NODE_EXIT_TIMEOUT=20
# ノード終了を確認した後の余裕時間 (実機センサとの接続解放などのため)
NODE_EXIT_MARGIN=2
# Unity の LIO がリセット後に原点付近のオドメトリを出すまでの待ち時間の上限 (超えたらリセットを送り直す)
UNITY_LIO_READY_TIMEOUT=15
# リセットできたとみなす LIO オドメトリの原点からの距離 [m]
UNITY_LIO_ORIGIN_TOLERANCE=1.0

# 終了待ちの対象。gnome-terminal の "bash -c 'ros2 launch ...; bash'" は ros2 launch の終了後も残るので、
# 実際の launch プロセス (python3 .../ros2 launch) とノード本体だけに一致させる
NAV_PROCESS_PATTERN='python3 [^ ]*ros2 launch hokuyo_navigation2 (hokuyo_nav2_bringup_launch|icart_mini_drive_launch)\.xml|lib/waypoint_manager/waypoint_manager|lib/hokuyo_rsf/hokuyo_rsf'

# ナビゲーション関連のプロセスが実際に終了するまで待つ関数
wait_for_nodes_exit() {
    local start=${SECONDS}
    echo "ノードの終了を待っています..."
    while pgrep -f "${NAV_PROCESS_PATTERN}" > /dev/null; do
        if (( SECONDS - start >= NODE_EXIT_TIMEOUT )); then
            echo "警告: ${NODE_EXIT_TIMEOUT} 秒以内に終了しなかったプロセスを強制終了します:"
            pgrep -a -f "${NAV_PROCESS_PATTERN}" || true
            pkill -9 -f "${NAV_PROCESS_PATTERN}" || true
            sleep 1
            break
        fi
        sleep 0.5
    done
    echo "ノードの終了を確認しました ($(( SECONDS - start )) 秒)。${NODE_EXIT_MARGIN} 秒待って次へ進みます。"
    sleep "${NODE_EXIT_MARGIN}"
}

# 終了したノードが残した Fast DDS の共有メモリ (/dev/shm/fastrtps_*) を掃除する関数。
# ファイルロックで使用中かを判定して不要なものだけ消すので、GUI や rosbridge など動き続けるノードには影響しない
# (rm -rf /dev/shm/fastrtps* だと使用中のものまで消して通信が途切れる)
clean_stale_shm() {
    fastdds shm clean > /dev/null 2>&1 || echo "警告: 共有メモリの掃除 (fastdds shm clean) に失敗しました。"
}

# マップ切替・再試行のためにナビゲーションを止め、次の起動の準備ができるまで待つ関数
shutdown_nodes_for_next_map() {
    "${HOKUYO_NAV2_PKG_PATH}/scripts/ctrl/multi_map_kill.sh"
    wait_for_nodes_exit
    # kill -9 で止めたノードの分も含め、終了を確認してから掃除する
    clean_stale_shm
    # 終了途中の cmdvel_stopper に速度 0 を上書きされないよう、ノードが止まってから送る
    reset_unity_sim
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
