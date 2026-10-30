#!/bin/bash
# GNSS モードの位置系統の調査用に rosbag を記録する (点群・IMU を含むため容量に注意).
# 使い方: rsf_bag_record.sh [保存先ディレクトリ]   (既定: ~/bags/rsf_chain)
# launch から起動された場合、末尾に付く --ros-args 以降は無視する.

bag_dir="${1:-${HOME}/bags/rsf_chain}"
if [ "${bag_dir}" = "--ros-args" ]; then
    bag_dir="${HOME}/bags/rsf_chain"
fi
mkdir -p "${bag_dir}"

exec ros2 bag record \
    -o "${bag_dir}/run_$(date +%Y%m%d_%H%M%S)" \
    --max-bag-duration 300 \
    /tf /tf_static /rosout \
    /rsf/rsf_fix /rsf/rsf_fix_state /rsf/rsf_fix_type \
    /rsf/rsf_odom_state /rsf/rsf_odom_type \
    /rsf/utm_coord_odom /rsf/rsf_fix_with_orient /rsf/rsf_odom_on_map \
    /rsf/diagnostics /wizurg/cmd_vel \
    /fix /gga /rsf/rsf_odom /rsf/lio_lidar_rate_odom /hokuyo3d/scan \
    /hokuyo3d/hokuyo_cloud2 /hokuyo3d/imu
