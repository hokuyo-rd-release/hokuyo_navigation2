#!/usr/bin/env python3

import sys
import os
import csv
import numpy as np
import glob
import sqlite3
from rosidl_runtime_py.utilities import get_message as get_message_ros1
from rclpy.serialization import deserialize_message as deserialize_message_ros1
from rosbag2_py import SequentialReader, StorageFilter, ConverterOptions, StorageOptions
from rclpy.serialization import deserialize_message as deserialize_message_ros2
from rosidl_runtime_py.import_message import import_message_from_namespaced_type
import rclpy

args = sys.argv

# get path
bag_folder = os.path.normpath(os.path.join(os.getcwd(), args[1]))
output_csv = os.path.normpath(os.path.join(os.getcwd(), args[2]))
topic_name = args[3]
thre = float(args[4])

def find_db_file(bag_folder):
    """指定されたフォルダ内の最初の .db3 ファイルを検索"""
    db_files = glob.glob(os.path.join(bag_folder, '*.db3'))
    if db_files:
        return db_files[0]
    else:
        return None

def connect(sqlite_file):
    conn = sqlite3.connect(sqlite_file)
    c = conn.cursor()
    return conn, c

def close(conn):
    conn.close()

def get_ros1_msg_type(cursor, topic_name):
    """ROS 1 の .db3 ファイルからメッセージ型を取得"""
    cursor.execute('SELECT type FROM topics WHERE name = ?', (topic_name,))
    result = cursor.fetchone()
    return result[0] if result else None

def get_ros1_messages(cursor, topic_id):
    """ROS 1 の .db3 ファイルから指定トピックのメッセージデータとタイムスタンプを取得"""
    cursor.execute('SELECT timestamp, data FROM messages WHERE topic_id = ?', (topic_id,))
    return cursor.fetchall()

def get_ros1_topic_id(cursor, topic_name):
    """ROS 1 の .db3 ファイルからトピック ID を取得"""
    cursor.execute('SELECT id FROM topics WHERE name = ?', (topic_name,))
    result = cursor.fetchone()
    return result[0] if result else None

def process_ros1_bag(db_file, topic_name, thre, output_csv):
    """ROS 1 の .db3 ファイルを処理"""
    conn, c = connect(db_file)
    msg_type_str = get_ros1_msg_type(c, topic_name)
    if not msg_type_str:
        print(f"Error (ROS 1): Message type not found for topic '{topic_name}' in '{db_file}'.")
        close(conn)
        return

    topic_id = get_ros1_topic_id(c, topic_name)
    if topic_id is None:
        print(f"Error (ROS 1): Topic ID not found for topic '{topic_name}' in '{db_file}'.")
        close(conn)
        return

    msgs_with_timestamps = get_ros1_messages(c, topic_id)
    num_messages = len(msgs_with_timestamps)
    covariances = np.empty((num_messages, 9))

    for i, (timestamp, msg_data) in enumerate(msgs_with_timestamps):
        try:
            msg_type = get_message_ros1(msg_type_str)
            deserialized_msg = deserialize_message_ros1(bytes(msg_data), msg_type)
            if hasattr(deserialized_msg, 'position_covariance'):
                covariances[i] = deserialized_msg.position_covariance
            else:
                print(f"Warning (ROS 1): Message at timestamp {timestamp} does not have 'position_covariance'. Skipping.")
                covariances = covariances[:i]
                num_messages = i
                break
        except Exception as e:
            print(f"Error (ROS 1) deserializing message at timestamp {timestamp}: {e}")
            covariances = covariances[:i]
            num_messages = i
            break

    close(conn)

    if num_messages == 0:
        print(f"No valid messages with position covariance found in topic '{topic_name}' in '{db_file}'.")
        return

    # NumPy の機能を使って fix 率を計算
    north_fixed = covariances[:, 0] <= thre
    east_fixed = covariances[:, 4] <= thre
    vertical_fixed = covariances[:, 8] <= thre

    avg_north_fix_rate = np.mean(north_fixed) * 100 if north_fixed.size > 0 else 0
    avg_east_fix_rate = np.mean(east_fixed) * 100 if east_fixed.size > 0 else 0
    avg_vertical_fix_rate = np.mean(vertical_fixed) * 100 if vertical_fixed.size > 0 else 0

    # NumPy の機能を使ってばらつきの平均を計算
    avg_north_variance = np.mean(covariances[:, 0]) if covariances.size > 0 else 0
    avg_east_variance = np.mean(covariances[:, 4]) if covariances.size > 0 else 0
    avg_vertical_variance = np.mean(covariances[:, 8]) if covariances.size > 0 else 0

    with open(output_csv, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        # Write fix rates
        csv_writer.writerow(['北方向のfix率[%]', '東方向のfix率[%]', '鉛直方向のfix率[%]'])
        csv_writer.writerow([f'{avg_north_fix_rate:.10f}', f'{avg_east_fix_rate:.10f}', f'{avg_vertical_fix_rate:.10f}'])

        # Write average variances
        csv_writer.writerow(['北方向のばらつきの平均[m]', '東方向のばらつきの平均[m]', '鉛直方向のばらつきの平均[m]'])
        csv_writer.writerow([f'{avg_north_variance:.10f}', f'{avg_east_variance:.10f}', f'{avg_vertical_variance:.10f}'])

        # Write individual variances
        csv_writer.writerow(['北方向のばらつき[m]', '東方向のばらつき[m]', '鉛直方向のばらつき[m]'])
        for n_var, e_var, v_var in covariances[:, [0, 4, 8]]:
            csv_writer.writerow([f'{n_var:.10f}', f'{e_var:.10f}', f'{v_var:.10f}'])

    print(f"ROS 1 data from '{db_file}' written to {output_csv}")

def process_ros2_bag(bag_file, topic_name, thre, output_csv):
    """ROS 2 の .mcap ファイルを処理"""
    reader = SequentialReader()
    try:
        storage_options = StorageOptions(uri=bag_file, storage_id='mcap')
        converter_options = ConverterOptions()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error (ROS 2) opening bag file '{bag_file}': {e}")
        return

    topic_types = reader.get_all_topics_and_types()
    target_type = None
    for topic_info in topic_types:
        if topic_info.name == topic_name:
            target_type = topic_info.type
            break

    if target_type is None:
        print(f"Error (ROS 2): Topic '{topic_name}' not found in '{bag_file}'.")
        del reader
        return

    storage_filter = StorageFilter()
    storage_filter.topics = [topic_name]
    reader.set_filter(storage_filter)

    messages = []
    while reader.has_next():
        try:
            (topic, data, t) = reader.read_next()
            if topic == topic_name:
                if target_type == 'sensor_msgs/msg/NavSatFix':
                    from sensor_msgs.msg import NavSatFix
                    deserialized_msg = NavSatFix()
                    deserialized_msg = deserialize_message_ros2(data, NavSatFix)
                    if hasattr(deserialized_msg, 'position_covariance'):
                        messages.append(deserialized_msg.position_covariance)
                    else:
                        print(f"Warning (ROS 2): Message at timestamp {t} does not have 'position_covariance'. Skipping.")
                else:
                    print(f"Error (ROS 2): Unsupported message type '{target_type}'.")
        except Exception as e:
            print(f"Error (ROS 2) reading message: {e}")

    del reader

    if not messages:
        print(f"No valid messages with position covariance found in topic '{topic_name}' in '{bag_file}'.")
        return

    covariances = np.array(messages)
    num_messages = len(covariances)

    # NumPy の機能を使って fix 率を計算
    north_fixed = covariances[:, 0] <= thre
    east_fixed = covariances[:, 4] <= thre
    vertical_fixed = covariances[:, 8] <= thre

    avg_north_fix_rate = np.mean(north_fixed) * 100 if north_fixed.size > 0 else 0
    avg_east_fix_rate = np.mean(east_fixed) * 100 if east_fixed.size > 0 else 0
    avg_vertical_fix_rate = np.mean(vertical_fixed) * 100 if vertical_fixed.size > 0 else 0

    # NumPy の機能を使ってばらつきの平均を計算
    avg_north_variance = np.mean(covariances[:, 0]) if covariances.size > 0 else 0
    avg_east_variance = np.mean(covariances[:, 4]) if covariances.size > 0 else 0
    avg_vertical_variance = np.mean(covariances[:, 8]) if covariances.size > 0 else 0

    with open(output_csv, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)

        # Write fix rates
        csv_writer.writerow(['北方向のfix率[%]', '東方向のfix率[%]', '鉛直方向のfix率[%]'])
        csv_writer.writerow([f'{avg_north_fix_rate:.10f}', f'{avg_east_fix_rate:.10f}', f'{avg_vertical_fix_rate:.10f}'])

        # Write average variances
        csv_writer.writerow(['北方向のばらつきの平均[m]', '東方向のばらつきの平均[m]', '鉛直方向のばらつきの平均[m]'])
        csv_writer.writerow([f'{avg_north_variance:.10f}', f'{avg_east_variance:.10f}', f'{avg_vertical_variance:.10f}'])

        # Write individual variances
        csv_writer.writerow(['北方向のばらつき[m]', '東方向のばらつき[m]', '鉛直方向のばらつき[m]'])
        for n_var, e_var, v_var in covariances[:, [0, 4, 8]]:
            csv_writer.writerow([f'{n_var:.10f}', f'{e_var:.10f}', f'{v_var:.10f}'])

    print(f"ROS 2 data from '{bag_file}' written to {output_csv}")

if __name__ == "__main__":
    mcap_files = glob.glob(os.path.join(bag_folder, '*.mcap'))
    db_file = find_db_file(bag_folder)

    if mcap_files:
        bag_file = mcap_files[0]
        print(f"Found ROS 2 bag file (prioritized): {bag_file}")
        process_ros2_bag(bag_file, topic_name, thre, output_csv)
    elif db_file:
        print(f"Found ROS 1 bag file: {db_file}")
        process_ros1_bag(db_file, topic_name, thre, output_csv)
    else:
        print(f"Error: No .mcap or .db3 files found in '{bag_folder}'.")