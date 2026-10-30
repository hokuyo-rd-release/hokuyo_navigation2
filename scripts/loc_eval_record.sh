#!/bin/bash
# loc モードの自己位置推定の検証用に rosbag を記録する (点群・IMU を含むため容量に注意).
# 生の点群と LIO を残すので、後から localization_node をオフラインで再生し直せる.
# 使い方: loc_eval_record.sh [保存先ディレクトリ]   (既定: ~/bags/loc_eval)
# launch から起動された場合、末尾に付く --ros-args 以降は無視する.

bag_dir="${1:-${HOME}/bags/loc_eval}"
if [ "${bag_dir}" = "--ros-args" ]; then
    bag_dir="${HOME}/bags/loc_eval"
fi
mkdir -p "${bag_dir}"

exec ros2 bag record \
    -o "${bag_dir}/loc_eval_$(date +%Y%m%d_%H%M%S)" \
    --max-bag-duration 300 \
    /tf /tf_static /rosout \
    /estimated_pose /loc_registered_cloud \
    /rsf/lio_imu_rate_odom /rsf/lio_lidar_rate_odom /rsf/aligned \
    /hokuyo3d/hokuyo_cloud2 /hokuyo3d/imu \
    /wizurg/cmd_vel /cmd_vel /plan /waypoints \
    /navigate_to_pose/_action/status
