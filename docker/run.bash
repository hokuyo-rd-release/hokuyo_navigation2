#!/bin/bash

IMAGE_NAME=hokuyo_navigation2:release
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
        echo -e "xhost + \n docker start $CONTAINER_NAME \n docker exec -it $CONTAINER_NAME /bin/bash" >>$CONTAINER_NAME.bash
    fi
else
    CONTAINER_NAME=""
fi

xhost +

docker run -it  $CONTAINER_NAME_CMD \
            -v /dev:/dev \
            -v /tmp/.X11-unix:/tmp/.X11-unix \
            -v $HOME/.Xauthority:/root/.Xauthority:rw \
            -v /var/run/dbus:/var/run/dbus \
            $SHARE_FOLDER_CMD \
            -e DISPLAY=$DISPLAY \
            -e QT_X11_NO_MITSHM=1 \
            -e XAUTHORITY=$XAUTHORITY \
            -v $XAUTHORITY:$XAUTHORITY \
            -e DOCKER_ENV=1 \
            -p 5050:5050 \
            -p 5000:5000 \
            -p 8085:8085 \
            -p 5001:5001 \
            -p 9090:9090 \
            -p 9000:9000 \
            -p 8080:8080 \
            -p 8000:8000 \
            -p 10940:10940 \
            -p 7400-7800:7400-7800/udp \
            $GPU_CMD \
            $REMOVE_CMD \
            --privileged \
            $IMAGE_NAME /bin/bash \
            -login

