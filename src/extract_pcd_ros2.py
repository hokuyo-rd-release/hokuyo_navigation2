#!/usr/bin/env python3

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import sys
import os
import numpy as np
import glob
from sensor_msgs_py import point_cloud2
import open3d as o3d
from rosbag2_py import SequentialReader, StorageFilter, ConverterOptions, StorageOptions

def find_db_file(bag_folder):
    """指定されたフォルダ内の最初の .db3 ファイルを検索"""
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
        return [], None

    topic_types = reader.get_all_topics_and_types()
    target_type = None
    for topic_info in topic_types:
        if topic_info.name == topic_name:
            target_type = topic_info.type
            break

    if target_type is None:
        print(f"Topic '{topic_name}' not found in bag file.")
        return [], None

    if target_type != 'sensor_msgs/msg/PointCloud2':
        print(f"Error: Topic '{topic_name}' in mcap is not of type sensor_msgs/msg/PointCloud2. Found type: {target_type}")
        return [], None

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
    return timestamps, messages

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

def extract_and_save_pointcloud_mcap(bag_file, topic_name):
    timestamps, msgs = read_all_messages_mcap(bag_file, topic_name)
    if msgs is None:
        return

    count = 1
    print(f"Processing topic (mcap): {topic_name}")
    for i, msg in enumerate(msgs):
        # より効率的な点の抽出
        data = np.frombuffer(msg.data, dtype=np.uint8).view(dtype=np.float32)
        if msg.point_step != 0:
            points = data.reshape(-1, msg.point_step // 4)[:, :3]
        else:
            points = np.array([])

        if points.size == 0:
            print(f"Warning: No valid points found in message at timestamp {timestamps[i]}. Skipping.")
            continue

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(f'cloud_{count:05}.pcd', pcd)
        count += 1
    print(f"Saved {count - 1} point cloud files (mcap).")

def extract_and_save_pointcloud_db(db_file, topic_name):
    conn, c = connect(db_file)
    msg_type_str = getMsgType(c, topic_name)
    if not msg_type_str:
        close(conn)
        return

    if msg_type_str != 'sensor_msgs/msg/PointCloud2':
        print(f"Error: Topic '{topic_name}' in db is not of type sensor_msgs/msg/PointCloud2. Found type: {msg_type_str}")
        close(conn)
        return

    timestamps, msgs_data = getAllMessagesInTopic(c, topic_name)
    count = 1
    print(f"Processing topic (db): {topic_name}")
    for i, msg_data in enumerate(msgs_data):
        msg_type = get_message(msg_type_str)
        deserialized_msg = deserialize_message(bytes(msg_data), msg_type) # ROS 1 は bytes で渡す

        # より効率的な点の抽出
        data = np.frombuffer(deserialized_msg.data, dtype=np.uint8).view(dtype=np.float32)
        if deserialized_msg.point_step != 0:
            points = data.reshape(-1, deserialized_msg.point_step // 4)[:, :3]
        else:
            points = np.array([])

        if points.size == 0:
            print(f"Warning: No valid points found in message at timestamp {timestamps[i]}. Skipping.")
            continue

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(f'cloud_{count:05}.pcd', pcd)
        count += 1
    close(conn)
    print(f"Saved {count - 1} point cloud files (db).")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python your_script_name.py <bag_folder> <pointcloud_topic_name>")
        sys.exit(1)

    bag_folder = os.path.normpath(os.path.join(os.getcwd(), sys.argv[1]))
    topic_name = sys.argv[2]

    mcap_files = glob.glob(os.path.join(bag_folder, '*.mcap'))
    db_file = find_db_file(bag_folder)

    if mcap_files:
        print(f"Found MCAP files: {mcap_files}")
        extract_and_save_pointcloud_mcap(mcap_files[0], topic_name)
    elif db_file:
        print(f"Found DB file: {db_file}")
        extract_and_save_pointcloud_db(db_file, topic_name)
    else:
        print(f"Error: No .mcap or .db3 files found in '{bag_folder}'.")
        sys.exit(1)