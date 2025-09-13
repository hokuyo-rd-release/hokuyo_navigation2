#!/bin/bash

# 対象のコンテナ名を指定
# ホスト環境では使用されません
CONTAINER_NAME="hokuyo_navigation2_dev"

# Docker環境かどうかを判定
# ターミナルで export DOCKER_ENV=1 として実行することで、Docker環境とみなします
# （あるいはDockerfile内で ENV DOCKER_ENV=1 を設定）
if [ -n "$DOCKER_ENV" ]; then
    echo "Docker環境で実行中です。コンテナ内のプロセスを停止します..."
    # 実行するコマンドの前につけるプレフィックス
    CMD_PREFIX="docker exec $CONTAINER_NAME"
else
    echo "ホストOS環境で実行中です。ホストのプロセスを停止します..."
    # ホストOSの場合、プレフィックスは不要
    CMD_PREFIX=""
fi

# 停止対象のプロセスをリストアップ
PROCESS_PATTERNS=(
    "python3 .*hokuyo_navigation2_gui/server.py"
    "ros2 launch vizanti_server vizanti_server.launch.py"
    # "python3 .*rosbridge_websocket"
    # "python3 .*vizanti_server/server.py"
    # "vizanti_cpp/tf_consolidator"
    # "python3 .*rosapi/rosapi_node"
    # "python3 .*vizanti_server/service_handler.py"
)

# すべての対象プロセスのPIDを取得
TARGET_PIDS=""
for pattern in "${PROCESS_PATTERNS[@]}"; do
    if [ -n "$DOCKER_ENV" ]; then
        PIDS=$(docker exec "$CONTAINER_NAME" ps aux | grep "$pattern" | grep -v "grep" | awk '{print $2}')
    else
        # ホストOSでは pgrep を使うのが便利です
        PIDS=$(pgrep -f "$pattern")
    fi
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
        if [ -n "$DOCKER_ENV" ]; then
            docker exec "$CONTAINER_NAME" kill -9 "$pid"
        else
            kill -9 "$pid"
        fi
        sleep 1
    done
else
    echo "No target processes found to kill."
fi

# ros2-daemon を安全に停止する
#echo "Attempting to stop ros2-daemon gracefully..."
#if [ -n "$DOCKER_ENV" ]; then
#    docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 daemon stop"
#else
#    source /opt/ros/humble/setup.bash && ros2 daemon stop
#fi
#sleep 1

# Docker環境の場合のみコンテナを停止
if [ -n "$DOCKER_ENV" ]; then
    echo "Stopping the container: $CONTAINER_NAME"
    docker stop "$CONTAINER_NAME"
fi
