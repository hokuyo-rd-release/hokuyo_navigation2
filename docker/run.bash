#!/bin/bash

IMAGE_NAME=hrjp/ros2:humble_cuda
CONTAINER_NAME=hokuyo_navigation2
SHARE_FOLDER_PATH=""
SHARE_FOLDER_CMD=""
GPU_CMD=""
CONTAINER_NAME_CMD="--name $CONTAINER_NAME"
NETHOST_CMD="--net=host"

usage_exit() {
        echo " " 1>&2
        echo " -----------------------------------------------------------------------------" 1>&2
        echo " OPTIONS              | DETAILS " 1>&2
        echo " -----------------------------------------------------------------------------" 1>&2
        echo " -g                   | GPU enabled" 1>&2
        echo " -r                   | remove when exit the container" 1>&2
        echo " -n CONTAINER_NAME    | container name (default : $CONTAINER_NAME )" 1>&2
        echo " -s SHARE_FOLDER_PATH | directory path shared with the inside of the container" 1>&2
        echo " -----------------------------------------------------------------------------" 1>&2
        exit 1
}

while getopts grwn:s:h OPT
do
    case $OPT in
        g )  GPU_CMD="--gpus all"
            echo " Using nvidia GPUs" 1>&2
            ;;
        r )  REMOVE_CMD="--rm"
            CONTAINER_NAME_CMD=""
            echo " Remove when exit this container" 1>&2
            ;;
        w )  NETHOST_CMD=""
            echo " Not using --net=host" 1>&2
            ;;
        n)  CONTAINER_NAME=$OPTARG
            CONTAINER_NAME_CMD="--name $CONTAINER_NAME"
            echo " CONTAINER_NAME = $OPTARG " 1>&2
            ;;
        s )  SHARE_FOLDER_PATH=$OPTARG
            SHARE_FOLDER_CMD="-v $SHARE_FOLDER_PATH:/home/share"
            echo " SHARE_FOLDER_PATH = $SHARE_FOLDER_PATH " 1>&2
            ;;
        h ) usage_exit
            ;;
        \? ) usage_exit
            ;;
    esac
done



if [ -z $REMOVE_CMD ]; then
    cd
    if [ ! -f $CONTAINER_NAME.bash ]; then
    	touch $CONTAINER_NAME.bash
    	sudo chmod 777 $CONTAINER_NAME.bash
    	echo '#!/bin/bash' >> "$CONTAINER_NAME.bash"
    	echo '' >> "$CONTAINER_NAME.bash"
    	echo 'gnome-terminal -- bash -c "$HOME/hokuyo_navigation2_docker_server.bash"' >> "$CONTAINER_NAME.bash"
    fi
    
    if [ ! -f $CONTAINER_NAME_docker_server.bash ]; then
        touch $CONTAINER_NAME_docker_server.bash
        sudo chmod 777 $CONTAINER_NAME_docker_server.bash
        echo '#!/bin/bash' > "$CONTAINER_NAME_docker_server.bash"
        echo '' >> "$CONTAINER_NAME_docker_server.bash"
        echo 'CONTAINER_NAME="hokuyo_navigation2_dev"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '' >> "$CONTAINER_NAME_docker_server.bash"
        echo 'cleanup() {' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    echo "スクリプトを終了します。コンテナ内のバックグラウンドプロセスを停止します..."' >> "$CONTAINER_NAME_docker_server.bash"
        echo '' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    PROCESS_PATTERNS=(' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        "python3 .*expo_gui/server.py"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        "ros2 launch vizanti_server vizanti_server.launch.py"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        "python3 .*rosbridge_websocket"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        "python3 .*vizanti_server/server.py"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        "vizanti_cpp/tf_consolidator"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        "python3 .*rosapi/rosapi_node"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        "python3 .*vizanti_server/service_handler.py"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        "python3 .*ros2cli/daemon/daemonize.py"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    )' >> "$CONTAINER_NAME_docker_server.bash"
        echo '' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    TARGET_PIDS=""' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    for pattern in "${PROCESS_PATTERNS[@]}"; do' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        PIDS=$(docker exec "$CONTAINER_NAME" ps aux | grep "$pattern" | grep -v "grep" | awk \'{print $2}\')' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        if [ -n "$PIDS" ]; then' >> "$CONTAINER_NAME_docker_server.bash"
        echo '            TARGET_PIDS+=" $PIDS"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        fi' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    done' >> "$CONTAINER_NAME_docker_server.bash"
        echo '' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    SORTED_PIDS=$(echo "$TARGET_PIDS" | tr '\'' '\''\\n'' | sort -rn | uniq | tr '\''\\n'' '\'' '')' >> "$CONTAINER_NAME_docker_server.bash"
        echo '' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    if [ -n "$SORTED_PIDS" ]; then' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        echo "Killing processes in descending PID order: $SORTED_PIDS"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        for pid in $SORTED_PIDS; do' >> "$CONTAINER_NAME_docker_server.bash"
        echo '            echo "Killing PID: $pid"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '            docker exec "$CONTAINER_NAME" kill -9 "$pid"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '            sleep 1' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        done' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    else' >> "$CONTAINER_NAME_docker_server.bash"
        echo '        echo "No target processes found to kill."' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    fi' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    echo "Attempting to stop ros2-daemon gracefully..."' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 daemon stop"' >> "$CONTAINER_NAME_docker_server.bash"
        echo '    sleep 1' >> "$CONTAINER_NAME_docker_server.bash"
        echo '}' >> "$CONTAINER_NAME_docker_server.bash"
        echo '' >> "$CONTAINER_NAME_docker_server.bash"
        echo 'trap cleanup EXIT' >> "$CONTAINER_NAME_docker_server.bash"
        echo '' >> "$CONTAINER_NAME_docker_server.bash"
        echo 'xhost +local:docker' >> "$CONTAINER_NAME_docker_server.bash"
        echo 'docker start "$CONTAINER_NAME"' >> "$CONTAINER_NAME_docker_server.bash"
        echo 'docker exec -d "$CONTAINER_NAME" bash -c "python3 /home/github/expo_software/takahashi/expo_gui/server.py"' >> "$CONTAINER_NAME_docker_server.bash"
        echo 'docker exec -d "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && source /home/colcon_ws/install/setup.bash && ros2 launch vizanti_server vizanti_server.launch.py"' >> "$CONTAINER_NAME_docker_server.bash"
        echo 'docker exec -it "$CONTAINER_NAME" /bin/bash' >> "$CONTAINER_NAME_docker_server.bash"
    fi
else
    CONTAINER_NAME=""
fi

xhost +

docker run -it  $CONTAINER_NAME_CMD\
            -v /dev:/dev \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v $HOME/.Xauthority:/root/.Xauthority:rw \
            -v /var/run/dbus:/var/run/dbus \
            $SHARE_FOLDER_CMD \
            -e DISPLAY=$DISPLAY \
            -e QT_X11_NO_MITSHM=1 \
            -e XAUTHORITY=$XAUTHORITY \
            -v $XAUTHORITY:$XAUTHORITY \
            -e DOCKER_ENV=1
            -p 5050:5050 \
            -p 5000:5000 \
            -p 8085:8085 \
            -p 5001:5001 \
            -p 9090:9090 \
            -p 9000:9000 \
            -p 8080:8080 \
            -p 8000:8000 \
            $GPU_CMD \
            $REMOVE_CMD \
            --privileged \
            $IMAGE_NAME /bin/bash

