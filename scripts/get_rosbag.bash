# $1 playbag name $2 outbag name

#------- 環境変数の確認 -------
#echo ros workspace: ${ROS_WORKSPACE:?ROS_WORKSPACE is Undefined}

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
CURRENT=$HOME/colcon_ws/src/hokuyo_navigation2
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

gnome-terminal -- bash -c "$CURRENT/scripts/rosbag_lio_fix_pc.bash $1 $2 $3; bash"