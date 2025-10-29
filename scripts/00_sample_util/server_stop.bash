#!/bin/bash

# 対象のコンテナ名を指定
CONTAINER_NAME="hokuyo_navigation2_dev" # <-- コンテナ名を置き換えてください

echo "スクリプトを終了します。コンテナ内のバックグラウンドプロセスを停止します..."

# 停止対象のプロセスをリストアップ
PROCESS_PATTERNS=(
    "python3 .*hokuyo_navigation2_gui/server.py"
    "ros2 launch vizanti_server vizanti_server.launch.py"
    "python3 .*rosbridge_websocket"
    "python3 .*vizanti_server/server.py"
    "vizanti_cpp/tf_consolidator"
    "python3 .*rosapi/rosapi_node"
    "python3 .*vizanti_server/service_handler.py"
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
