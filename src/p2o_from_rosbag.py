#!/usr/bin/env python3
# Generate p2o from rosbag file

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from tqdm import tqdm
import kgeom3d
from pyproj import Transformer
import sys

# parameters

# information matrix (upper triangle) for odometry observations
odom_infom = '1e2 0 0 0 0 0 1e2 0 0 0 0 1e2 0 0 0 1e2 0 0 1e2 0 1e2'
#gnss_cov_thre = 0.0001

def latlon_to_xyz(trans, lat, lon, alt):
    x, y = trans.transform(lat, lon)
    return x, y, alt


args = sys.argv
assert len(args)>=2, "you must specify a rosbag file"

gnss_cov_thre = float(args[4])
# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
lio_topic_name=os.path.normpath(os.path.join(os.getcwd(),args[2]))
gnss_topic_name=os.path.normpath(os.path.join(os.getcwd(),args[3]))

# Sample convert to Japan Plane Rectangular Coordinate System No. 6
transformer = Transformer.from_crs("epsg:4326", 'epsg:6674')

# read the bag file

bag = rosbag.Bag(filename)
prev_gnss_t = 0
vertices=[]
edges=[]
np_poses=None
np_gnss_list=None
np_lla_list=None
id = 0
for topic, msg, t in bag.read_messages():
    t_since_epoch = t.secs + t.nsecs * 1e-9
    if topic==lio_topic_name:
        id += 1
        pose = msg.pose.pose
        np_pose=np.zeros((1,8), dtype=np.float64)
        np_pose[0,0]=t_since_epoch
        np_pose[0,1]=pose.position.x
        np_pose[0,2]=pose.position.y
        np_pose[0,3]=pose.position.z
        np_pose[0,4]=pose.orientation.x
        np_pose[0,5]=pose.orientation.y
        np_pose[0,6]=pose.orientation.z
        np_pose[0,7]=pose.orientation.w
        if np_poses is None:
            np_poses=np_pose
        else:
            np_poses=np.append(np_poses,np_pose,axis=0)
        q = pose.orientation
        qstr = f'{q.x} {q.y} {q.z} {q.w}'
        vertices.append(f'VERTEX_SE3:QUAT {id} {pose.position.x} {pose.position.y} {pose.position.z} {qstr}')

    if topic==gnss_topic_name:
    	# print(msg.position_covariance[0]) # 2025/01/28 この共分散が高いと配列に要素が追加されていない。
        if (msg.status.status == 0 or msg.status.status == 2) and id > 0:
            if msg.position_covariance[0] < gnss_cov_thre and (t_since_epoch - prev_gnss_t > 3):
                #print(f'gnss status: {msg.status.status}', file=sys.stderr)
                x, y, z = latlon_to_xyz(transformer, msg.latitude, msg.longitude, msg.altitude)
                lat,lon,alt = msg.latitude, msg.longitude, msg.altitude
                np_gnss=np.zeros((1,8), dtype=np.float64)
                np_gnss[0,0]=t_since_epoch
                np_gnss[0,1]=id
                np_gnss[0,2]=x
                np_gnss[0,3]=y
                np_gnss[0,4]=z
                np_gnss[0,5]=msg.position_covariance[0]
                np_gnss[0,6]=msg.position_covariance[4]
                np_gnss[0,7]=msg.position_covariance[8]

                np_lla=np.zeros((1,8), dtype=np.float64)
                np_lla[0,0]=t_since_epoch
                np_lla[0,1]=id
                np_lla[0,2]=lat
                np_lla[0,3]=lon
                np_lla[0,4]=alt
                np_lla[0,5]=msg.position_covariance[0]
                np_lla[0,6]=msg.position_covariance[4]
                np_lla[0,7]=msg.position_covariance[8]
                if np_gnss_list is None:
                    np_gnss_list = np_gnss
                else:
                    np_gnss_list=np.append(np_gnss_list,np_gnss,axis=0)
                if np_lla_list is None:
                    np_lla_list = np_lla
                else:
                    np_lla_list=np.append(np_lla_list,np_lla,axis=0)
                prev_gnss_t = t_since_epoch

bag.close()


mean_gnss = np.mean(np_gnss_list, axis=0)
#mean_lla = np.mean(np_lla_list, axis=0)
#print(mean_gnss, file=sys.stderr)

vertices.insert(0, f'VERTEX_SE3:QUAT 0 {mean_gnss[3]} {mean_gnss[2]} {mean_gnss[4]} 0 0 0 1')

for i in range(1, len(np_poses)):
    p = np_poses[i,:]
    prevp = np_poses[i-1,:]
    dp = kgeom3d.ominus_se3(p[1:],prevp[1:])
    edges.append(f'EDGE_SE3:QUAT {i} {i+1} {dp[0]} {dp[1]} {dp[2]} {dp[3]} {dp[4]} {dp[5]} {dp[6]} {odom_infom}')

for i in range(len(np_gnss_list)):
    id = int(np_gnss_list[i,1])
    x  = np_gnss_list[i,2]-mean_gnss[2]
    y  = np_gnss_list[i,3]-mean_gnss[3]
    z  = np_gnss_list[i,4]-mean_gnss[4]
    xinfo = min(1.0, 1.0/np_gnss_list[i,5])
    yinfo = min(1.0, 1.0/np_gnss_list[i,6])
    zinfo = min(1.0, 1.0/np_gnss_list[i,7])
    gnss_infom = f'{xinfo} 0 0 {yinfo} 0 {zinfo}'

    edges.append(f'EDGE_LIN3D 0 {id} {y} {x} {z} {gnss_infom}')

for i in range(len(np_lla_list)):
    id = int(np_lla_list[i,1])
    lat = np_lla_list[i,2]
    lon = np_lla_list[i,3]
    alt = np_lla_list[i,4]

    edges.append(f'EDGE_LLA 0 {id} {lat} {lon} {alt}')

for v in vertices:
    print(v)

for e in edges:
    print(e)

