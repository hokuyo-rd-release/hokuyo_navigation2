#!/bin/bash

# このスクリプトは nav_single_map.sh と nav_multi_map.sh から source されることを想定しています。

# =========================================================================
# ライフサイクル / 起動待ち関連
# =========================================================================

# --- 状態文字列だけを安全に抽出するヘルパー関数 (BrokenPipeError対策済み) ---
get_lifecycle_state() {
    local node_name="$1"
    # 一度変数に受けてから処理することで、PythonのBrokenPipeErrorを完全に回避する
    local output
    output=$(ros2 lifecycle get "$node_name" 2>/dev/null || echo "")

    local raw_state
    raw_state=$(echo "$output" | grep -E '(unconfigured|inactive|active)' | head -n 1 || echo "unknown")
    echo "$raw_state" | tr -d '[:space:]'
}

# --- Map Server の起動を安全に待つ関数 ---
# 第1引数: タイムアウト秒数 (省略時は 30 秒)
wait_for_map_server() {
    local timeout="${1:-30}"
    local start_time=$(date +%s)
    echo "map_server の起動を待っています..."

    while true; do
        local state
        state=$(get_lifecycle_state "/map_server")

        echo "現在の状態: ${state}"

        if [[ "$state" == active* ]]; then
            echo "map_server はすでに Active です。"
            return 0
        fi

        if [[ "$state" == inactive* ]]; then
            echo "Inactive 状態を検出しました。activate を実行します。"
            ros2 lifecycle set /map_server activate 2>/dev/null || true
            sleep 2
            continue
        fi

        if [[ "$state" == unconfigured* ]]; then
            echo "Unconfigured 状態を検出しました。configure と activate を実行します。"
            ros2 lifecycle set /map_server configure 2>/dev/null || true
            ros2 lifecycle set /map_server activate 2>/dev/null || true
            sleep 2
            continue
        fi

        local current_time=$(date +%s)
        if (( current_time - start_time > timeout )); then
            echo "エラー: map_server の起動確認がタイムアウトしました。"
            return 1
        fi

        sleep 1
    done
}

# --- Nav2の起動とライフサイクル、アクションサーバーの準備完了を監視する関数 ---
# 第1引数: タイムアウト秒数 (省略時は 60 秒)
wait_for_nav2_ready() {
    local timeout="${1:-60}"
    local start_time=$(date +%s)
    echo "Nav2のライフサイクル状態とアクションサーバーの準備を監視しています..."

    while true; do
        local state
        state=$(get_lifecycle_state "/bt_navigator")

        local action_exists
        action_exists=$(ros2 action list 2>/dev/null | grep -c "/navigate_to_pose" || echo "0")

        echo -ne "Debug: bt_navigator state is '${state}', action server ready: ${action_exists}... \r"

        if [[ "$state" == active* ]] && [ "$action_exists" -gt 0 ]; then
            echo -e "\nNav2システムおよびアクションサーバーの準備が完全に完了しました。"
            echo "コストマップの安定化を待っています (3秒)..."
            sleep 3
            return 0
        fi

        if [[ "$state" == unconfigured* ]]; then
            echo -e "\nUnconfigured状態を検出しました。configureを試行します..."
            ros2 lifecycle set /bt_navigator configure 2>/dev/null || true
            sleep 1
        fi

        if [[ "$state" == inactive* ]]; then
            echo -e "\nInactive状態を検出しました。activateを試行します..."
            ros2 lifecycle set /bt_navigator activate 2>/dev/null || true
            sleep 1
        fi

        local current_time=$(date +%s)
        if (( current_time - start_time > timeout )); then
            echo -e "\nエラー: Nav2の起動確認がタイムアウトしました。現在の状態: ${state}"
            return 1
        fi

        sleep 1
    done
}

# =========================================================================
# オプション読み込み関連
# =========================================================================

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
    use_unity="${option_arr[6]}"
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
    # ノードが検出されるまで待機
    echo "map_server の起動を待っています..."
    while ! ros2 node list | grep -q "^/map_server$"; do sleep 0.5; done

    # 現在の状態を取得
    CURRENT_STATE=$(ros2 lifecycle get /map_server)
    echo "現在の状態: $CURRENT_STATE"

    # 状態名、または状態番号[2]（Inactive）を判定してアクティブ化
    if echo "$CURRENT_STATE" | grep -E -q "unconfigured|\[1\]"; then
        echo "未設定のため、configure と activate を実行します。"
        ros2 lifecycle set /map_server configure && sleep 0.5 && ros2 lifecycle set /map_server activate
    elif echo "$CURRENT_STATE" | grep -E -q "Inactive|inactive|\[2\]"; then
        echo "Inactive 状態を検出しました。activate を実行します。"
        ros2 lifecycle set /map_server activate
    elif echo "$CURRENT_STATE" | grep -E -q "Active|active|\[3\]"; then
        echo "すでに Active（有効化済み）です。"
    else
        echo "想定外の状態ですが、強制的に activate を試みます。"
        ros2 lifecycle set /map_server activate
    fi

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
