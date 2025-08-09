#!/usr/bin/env python3

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sys
import os
import numpy as np
import glob
import kgeom3d
from pyproj import Transformer
from rosbag2_py import SequentialReader, StorageFilter, ConverterOptions, StorageOptions

# parameters
odom_infom = '1e2 0 0 0 0 0 1e2 0 0 0 0 1e2 0 0 0 1e2 0 0 1e2 0 1e2'

def find_db_file(bag_folder):
    db_files = glob.glob(os.path.join(bag_folder, '*.db3'))
    if db_files:
        return db_files[0]
    else:
        return None

def read_all_messages_mcap(bag_path, topic_name):
    """指定された mcap ファイルから特定トピックのメッセージを読み込む"""
    reader = SequentialReader()
    try:
        storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
        converter_options = ConverterOptions()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file '{bag_path}': {e}")
        return [], None, None

    topic_types = reader.get_all_topics_and_types()
    target_type = None
    for topic_info in topic_types:
        if topic_info.name == topic_name:
            target_type = topic_info.type
            break

    if target_type is None:
        print(f"Topic '{topic_name}' not found in bag file.")
        return [], None, None

    storage_filter = StorageFilter()
    storage_filter.topics = [topic_name]
    reader.set_filter(storage_filter)

    timestamps = []
    messages = []
    while reader.has_next():
        try:
            (topic, data, t) = reader.read_next()
            if topic == topic_name:
                msg_type = get_message_mcap(target_type)
                if msg_type:
                    deserialized_msg = deserialize_message(data, msg_type)
                    timestamps.append(t)
                    messages.append(deserialized_msg)
                else:
                    print(f"Error: Could not get message type '{target_type}' for mcap.")
        except Exception as e:
            print(f"Error reading message: {e}")

    del reader
    return timestamps, messages, target_type

def get_message_mcap(type_name):
    """ROS メッセージの型名からメッセージクラスを取得する (mcap 用)"""
    try:
        # '/' を '.' に置換して直接インポートを試みる
        module_path = type_name.replace('/', '.')
        parts = module_path.split('.')
        if len(parts) >= 2:
            package_name = parts[0]
            message_name = parts[-1]
            module_name = '.'.join(parts[:-1])
            module = __import__(module_name, fromlist=[message_name])
            return getattr(module, message_name)
        else:
            print(f"Error: Invalid message type name '{type_name}'.")
            return None
    except ImportError as e:
        print(f"Error: Could not import message type '{type_name}': {e}")
        return None
    except AttributeError:
        print(f"Error: Could not find message class in module for '{type_name}'.")
        return None

def connect(sqlite_file):
    conn = sqlite3.connect(sqlite_file)
    c = conn.cursor()
    return conn, c

def close(conn):
    conn.close()

def getAllElements(cursor, table_name, print_out=False):
    cursor.execute('SELECT * from({})'.format(table_name))
    records = cursor.fetchall()
    if print_out:
        print("\nAll elements:")
        for row in records:
            print(row)
    return records

def isTopic(cursor, topic_name, print_out=False):
    boolIsTopic = False
    topicFound = []
    records = getAllElements(cursor, 'topics', print_out=False)
    for row in records:
        if(row[1] == topic_name):
            boolIsTopic = True
            topicFound = row
    if print_out:
        if boolIsTopic:
            print('\nTopic named', topicFound[1], ' exists at id ', topicFound[0] ,'\n')
        else:
            print('\nTopic', topic_name ,'could not be found. \n')
    return topicFound

def getAllMessagesInTopic(cursor, topic_name, print_out=False):
    timestamps = []
    messages = []
    topicFound = isTopic(cursor, topic_name, print_out=False)
    if not topicFound:
        print('Topic', topic_name ,'could not be found. \n')
    else:
        cursor.execute('SELECT timestamp, data FROM messages WHERE topic_id = ?', (topicFound[0],))
        records = cursor.fetchall()
        for row in records:
            timestamps.append(row[0])
            messages.append(row[1])
    return timestamps, messages

def getMsgType(cursor, topic_name, print_out=False):
    msg_type = None
    cursor.execute('SELECT type FROM topics WHERE name = ?', (topic_name,))
    result = cursor.fetchone()
    if result:
        msg_type = result[0]
        if print_out:
            print(f'\nMessage type in {topic_name} is {msg_type}')
    else:
        print(f'\nTopic {topic_name} not found.')
    return msg_type

def latlon_to_xyz(trans, lat, lon, alt):
    x, y = trans.transform(lat, lon)
    return x, y, alt

if __name__ == "__main__":
    args = sys.argv
    assert len(args) >= 5, "Usage: ros2 run your_package_name your_script_name <bag_folder> <lio_topic> <gnss_topic> <gnss_cov_threshold>"

    bag_folder = os.path.normpath(os.path.join(os.getcwd(), args[1]))
    lio_topic_name = args[2]
    gnss_topic_name = args[3]
    gnss_cov_thre = float(args[4])

    mcap_files = glob.glob(os.path.join(bag_folder, '*.mcap'))
    db_file = find_db_file(bag_folder)

    lio_timestamps = []
    lio_msgs = []
    lio_msg_type_str = None
    gnss_timestamps = []
    gnss_msgs = []
    gnss_msg_type_str = None

    if mcap_files:
        bag_file = mcap_files[0]
        #print(f"Processing ROS 2 bag file: {bag_file}")
        lio_timestamps, lio_msgs, lio_msg_type_str = read_all_messages_mcap(bag_file, lio_topic_name)
        gnss_timestamps, gnss_msgs, gnss_msg_type_str = read_all_messages_mcap(bag_file, gnss_topic_name)
        get_message_func = get_message_mcap
    elif db_file:
        #print(f"Processing ROS 1 bag file: {db_file}")
        conn, c = connect(db_file)
        lio_msg_type_str = getMsgType(c, lio_topic_name)
        gnss_msg_type_str = getMsgType(c, gnss_topic_name)

        if not lio_msg_type_str or not gnss_msg_type_str:
            close(conn)
            exit()

        lio_timestamps, lio_msgs_data = getAllMessagesInTopic(c, lio_topic_name)
        gnss_timestamps, gnss_msgs_data = getAllMessagesInTopic(c, gnss_topic_name)

        lio_msgs = [deserialize_message(msg, get_message(lio_msg_type_str)) for msg in lio_msgs_data]
        gnss_msgs = [deserialize_message(msg, get_message(gnss_msg_type_str)) for msg in gnss_msgs_data]

        close(conn)
        get_message_func = get_message
    else:
        print(f"Error: No .mcap or .db3 files found in '{bag_folder}'.")
        exit()

    if not lio_msgs or not gnss_msgs:
        print("Error: Could not retrieve LIO or GNSS messages.")
        exit()

    num_lio = len(lio_timestamps)
    vertices = [None] * (num_lio + 1)  # Pre-allocate list for vertices
    edges = []
    np_poses_list = [None] * num_lio
    id_counter = 0

    # Sample convert to Japan Plane Rectangular Coordinate System No. 6
    transformer = Transformer.from_crs("epsg:4326", 'epsg:6674')

    # Process LIO data
    for i in range(num_lio):
        timestamp = lio_timestamps[i] * 1e-9 if mcap_files else lio_timestamps[i] * 1e-9
        pose = lio_msgs[i].pose.pose

        id_counter += 1
        np_poses_list[i] = np.array([timestamp,
                                      pose.position.x, pose.position.y, pose.position.z,
                                      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

        q = pose.orientation
        qstr = f'{q.x} {q.y} {q.z} {q.w}'
        vertices[id_counter] = f'VERTEX_SE3:QUAT {id_counter} {pose.position.x} {pose.position.y} {pose.position.z} {qstr}'

        if i > 0:
            prev_pose_np = np_poses_list[i-1][1:]
            current_pose_np = np_poses_list[i][1:]
            dp = kgeom3d.ominus_se3(current_pose_np, prev_pose_np)
            edges.append(f'EDGE_SE3:QUAT {id_counter-1} {id_counter} {dp[0]} {dp[1]} {dp[2]} {dp[3]} {dp[4]} {dp[5]} {dp[6]} {odom_infom}')

    np_poses = np.array(np_poses_list)

    # Process GNSS data
    valid_gnss_data = []
    for i, timestamp in enumerate(gnss_timestamps):
        msg = gnss_msgs[i]
        if hasattr(msg, 'status') and hasattr(msg.status, 'status') and \
           (msg.status.status == 0 or msg.status.status == 2) and \
           hasattr(msg, 'position_covariance') and len(msg.position_covariance) >= 9 and \
           msg.position_covariance[0] < gnss_cov_thre:
            valid_gnss_data.append((timestamp * 1e-9 if mcap_files else timestamp * 1e-9, msg))
        elif not hasattr(msg, 'status') and hasattr(msg, 'position_covariance') and len(msg.position_covariance) >= 9 and msg.position_covariance[0] < gnss_cov_thre:
            # ROS 1 の場合 status がないことがあるため、covariance のみで判定
            valid_gnss_data.append((timestamp * 1e-9 if mcap_files else timestamp * 1e-9, msg))

    mean_gnss = np.zeros(3)
    if valid_gnss_data:
        gnss_positions = np.array([latlon_to_xyz(transformer, msg.latitude, msg.longitude, msg.altitude)
                                   for _, msg in valid_gnss_data])
        mean_gnss = np.mean(gnss_positions, axis=0)
        vertices[0] = f'VERTEX_SE3:QUAT 0 {mean_gnss[0]} {mean_gnss[1]} {mean_gnss[2]} 0 0 0 1'

        for timestamp, msg in valid_gnss_data:
            gnss_xyz = latlon_to_xyz(transformer, msg.latitude, msg.longitude, msg.altitude)
            x = gnss_xyz[0] - mean_gnss[0]
            y = gnss_xyz[1] - mean_gnss[1]
            z = gnss_xyz[2] - mean_gnss[2]
            xinfo = min(1.0, 1.0 / msg.position_covariance[0]) if msg.position_covariance[0] > 0 else 1.0
            yinfo = min(1.0, 1.0 / msg.position_covariance[4]) if msg.position_covariance[4] > 0 else 1.0
            zinfo = min(1.0, 1.0 / msg.position_covariance[8]) if msg.position_covariance[8] > 0 else 1.0
            gnss_infom = f'{xinfo} 0 0 {yinfo} 0 {zinfo}'

            closest_lio_id = 0
            min_diff = float('inf')
            gnss_t = timestamp
            for j, lio_t in enumerate(np_poses[:, 0]):
                diff = abs(lio_t - gnss_t)
                if diff < min_diff:
                    min_diff = diff
                    closest_lio_id = j + 1

            if closest_lio_id > 0 and closest_lio_id <= id_counter:
                edges.append(f'EDGE_LIN3D 0 {closest_lio_id} {y} {x} {z} {gnss_infom}')
                edges.append(f'EDGE_LLA 0 {closest_lio_id} {msg.latitude} {msg.longitude} {msg.altitude}')

    for v in vertices:
        if v is not None:
            print(v)

    for e in edges:
        print(e)