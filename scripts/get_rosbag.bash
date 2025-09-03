# $1 playbag name $2 outbag name

#------- 環境変数の確認 -------
#echo ros workspace: ${ROS_WORKSPACE:?ROS_WORKSPACE is Undefined}

# Docker環境かどうかを判定する
# コンテナの起動時に -e DOCKER_ENV=1 を指定することで、Docker環境とみなすことができます。
if [ -n "$DOCKER_ENV" ]; then
    source /opt/ros/humble/setup.bash
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${HOME}/colcon_ws/src/hokuyo_navigation2"
else
    source /opt/ros/humble/setup.bash
    # ワークスペースのパスもホストOSのものに合わせる
    # 実際のホストOSのワークスペースパスに置き換えてください
    cd ${HOME}/colcon_ws
    source install/setup.bash
    source ~/.bashrc
    ROS2_WS="${HOME}/colcon_ws"
    HOKUYO_NAV2_PKG_PATH="${HOME}/colcon_ws/src/hokuyo_navigation2"
fi

#------- 引数の確認 -------
# 第1引数
if [ -z "$1" ]; then
  echo "Error: 引数が不足しています <arg1>"
  exit 1
fi
# 第2引数
if [ -z "$2" ]; then
  echo "Error: 引数が不足しています <arg2>"
  exit 1
fi

#------- カレントディレクトリの取得 -------
CURRENT=$HOKUYO_NAV2_PKG_PATH
echo current dir: $CURRENT
rosbag_dir=$CURRENT/rosbag;
echo rosbag dir: $rosbag_dir
echo "All args are checked."

# ファイルが存在するかチェック
DIR=$1
if [ ! -d "$DIR" ]; then
  echo "Error: directory: $DIR does not exist."
  exit 1
else
  echo "rosbag directory: $DIR exists."
fi

sleep 1

gnome-terminal -- bash -c "${HOKUYO_NAV2_PKG_PATH}/scripts/rosbag_lio_fix_pc.bash $1 $2 $3; bash"