#!/bin/bash

# ROS とウェブサーバーを起動するコンテナ名
CONTAINER_NAME="hokuyo_navigation2_dev"

# 終了時にプロセスをキルするための関数
cleanup() {
    echo "スクリプトを終了します。コンテナ内のバックグラウンドプロセスを停止します..."

    # 停止対象のプロセスをリストアップ
    PROCESS_PATTERNS=(
        "python3 .*hokuyo_navigation2_gui/server.py"
        "ros2 launch vizanti_server vizanti_server.launch.py"
        "ros2 run hokuyo_navigation2 coordinator.sh"
        # "python3 .*rosbridge_websocket"
        # "python3 .*vizanti_server/server.py"
        # "vizanti_cpp/tf_consolidator"
        # "python3 .*rosapi/rosapi_node"
        # "python3 .*vizanti_server/service_handler.py"
    )

    # すべての対象プロセスのPIDを取得し、降順にソート
    TARGET_PIDS=""
    for pattern in "${PROCESS_PATTERNS[@]}"; do
        PIDS=$(docker exec "$CONTAINER_NAME" ps aux | grep "$pattern" | grep -v "grep" | awk '{print $2}')
        if [ -n "$PIDS" ]; then
            TARGET_PIDS+=" $PIDS"
        fi
    done

    # PIDリストをユニークにして、数値で降順にソート
    SORTED_PIDS=$(echo "$TARGET_PIDS" | tr ' ' '\n' | sort -rn | uniq | tr '\n' ' ')

    # ソートされたPIDを一つずつ強制的にキル
    if [ -n "$SORTED_PIDS" ]; then
        echo "Killing processes in descending PID order: $SORTED_PIDS"
        for pid in $SORTED_PIDS; do
            echo "Killing PID: $pid"
            docker exec "$CONTAINER_NAME" kill -9 "$pid"
            sleep 1
        done
    else
        echo "No target processes found to kill."
    fi
    # ros2-daemon を安全に停止する
    echo "Attempting to stop ros2-daemon gracefully..."
    docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 daemon stop"
    sleep 1
}

# スクリプトが終了したときに cleanup 関数を実行するように設定
trap cleanup EXIT

# 1. X Window System のアクセスを許可 (必要に応じて)
xhost +local:docker

# 2. 既存のコンテナを起動
echo "Starting Docker container: $CONTAINER_NAME..."
docker start "$CONTAINER_NAME"

# 3. コンテナ内でウェブサーバーをバックグラウンドで起動
echo "Starting web server in container..."
docker exec -d "$CONTAINER_NAME" bash -c "python3 /home/github/hokuyo_navigation2_gui/server.py"
sleep 2s
# 4. コンテナ内でvizanti を起動
echo "Starting Vizanti server in container..."
docker exec -d "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /home/colcon_ws/install/setup.bash && ros2 launch vizanti_server vizanti_server.launch.py"
sleep 2s
# 5. コンテナ内のシェルに接続
# このコマンドはユーザーが exit するまで終了しない
echo "Starting nav2 coordinator in the container's shell. Type 'exit' to disconnect."
docker exec -it "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /home/colcon_ws/install/setup.bash && ros2 run hokuyo_navigation2 coordinator.sh; bash"
#docker exec -it "$CONTAINER_NAME" /bin/bash
