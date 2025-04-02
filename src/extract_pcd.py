# Copyright (C) 2024 Kiyoshi Irie
# MIT License

import sys
import numpy as np
import rosbag
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import open3d as o3d

def extract_and_save_pointcloud(rosbag_path, topic_name):
    bag = rosbag.Bag(rosbag_path, 'r')
    count = 1
    print(topic_name)
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        #if isinstance(msg, PointCloud2):
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if points.size == 0:
            print('error.')
            exit(0)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(f'cloud_{count:05}.pcd', pcd)
        count += 1
    bag.close()


extract_and_save_pointcloud(sys.argv[1], sys.argv[2])

