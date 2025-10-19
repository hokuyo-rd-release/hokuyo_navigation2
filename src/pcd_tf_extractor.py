#!/usr/bin/env python3

import sqlite3
import sys
import os
import numpy as np
import glob
import math
import open3d as o3d 
import json 
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from sensor_msgs_py import point_cloud2

# TF変換のためのライブラリ
try:
    from transforms3d.quaternions import quat2mat
    from transforms3d.affines import compose
except ImportError:
    print("Error: 'transforms3d' library not found. Please run: pip install transforms3d")
    sys.exit(1)


# ROS Bag2の読み込み
try:
    from rosbag2_py import SequentialReader, StorageFilter, ConverterOptions, StorageOptions
except ImportError:
    print("Error: 'rosbag2_py' library not found. Ensure you are in a ROS 2 environment or have it installed.")
    sys.exit(1)


# --- 設定値 (C++ノードのパラメータに相当) ---
# ※ main関数で上書きされますが、初期値として設定
TOPICS = {
    "PCD": "hokuyo3d/hokuyo_cloud2",  # C++: sub_topic
    "ODOM": "hokuyo_lio/lidar_odom",  # C++: sub_odom_topic
    "TF": "/tf",                      # TFトピック (通常は/tf)
}
ORIG_FRAME = "Odometry"  # C++: orig_frame
TARGET_FRAME = "odom"    # C++: target_frame

# 地図関連の設定
MAP_DIR = "/home/hokuyo/colcon_ws/src/hokuyo_navigation2/map"    # C++: map_dir
MAP_NAME = "map.pcd"     # C++: map_name
PC_SAVE_DISTANCE = 1.0   # C++: pc_save_distance (メートル)

# Waypoint関連の設定
WP_DIR = "/home/hokuyo/colcon_ws/src/hokuyo_navigation2/waypoints" # Waypoint出力ディレクトリ
WP_SAVE_DISTANCE = 4.0   # Waypointを保存する移動距離 (メートル)
WP_FILE_NAME = "map.json"

# --- ヘルパー関数 ---

def find_db_file(bag_folder):
    """指定されたフォルダ内の最初の .db3 ファイルを検索"""
    db_files = glob.glob(os.path.join(bag_folder, '*.db3'))
    return db_files[0] if db_files else None

def get_message_class(type_name):
    """ROS メッセージの型名からメッセージクラスを取得する"""
    try:
        module_path = type_name.replace('/', '.')
        parts = module_path.split('.')
        if len(parts) >= 2:
            package_name = parts[0]
            message_name = parts[-1]
            module_name = '.'.join(parts[:-1])
            module = __import__(module_name, fromlist=[message_name])
            return getattr(module, message_name)
        else:
            return None
    except ImportError:
        return None
    except AttributeError:
        return None

def read_all_messages_from_bag(bag_path, storage_id, topic_list):
    """bagファイルから指定されたトピックの全メッセージを読み込む"""
    reader = SequentialReader()
    try:
        storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
        converter_options = ConverterOptions()
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag file '{bag_path}': {e}")
        return {}

    topic_types = {info.name: info.type for info in reader.get_all_topics_and_types()}
    
    topics_to_read = [t for t in topic_list if t in topic_types]
    if not topics_to_read:
        print("Error: None of the target topics found in bag file.")
        return {}

    storage_filter = StorageFilter()
    storage_filter.topics = topics_to_read
    reader.set_filter(storage_filter)
    
    all_data = {topic: [] for topic in topics_to_read}
    
    print(f"Reading topics: {topics_to_read} from bag...")
    while reader.has_next():
        try:
            (topic, data, t) = reader.read_next()
            if topic in topic_types:
                msg_type = get_message_class(topic_types[topic])
                if msg_type:
                    deserialized_msg = deserialize_message(data, msg_type)
                    all_data[topic].append((t, deserialized_msg))
        except Exception:
            pass
            
    del reader
    return all_data

def get_db_messages(db_file, topic_list):
    """DB3ファイルから指定されたトピックの全メッセージを読み込む"""
    conn = sqlite3.connect(db_file)
    c = conn.cursor()
    
    all_data = {topic: [] for topic in topic_list}
    topic_map = {} # topic_name: (topic_id, msg_type)

    c.execute('SELECT id, name, type FROM topics')
    for row in c.fetchall():
        if row[1] in topic_list:
            topic_map[row[1]] = (row[0], row[2])

    if not topic_map:
        print("Error: None of the target topics found in DB file.")
        conn.close()
        return {}
        
    for topic_name, (topic_id, msg_type_str) in topic_map.items():
        msg_type = get_message_class(msg_type_str)
        if not msg_type:
            print(f"Warning: Could not resolve message type for {topic_name}. Skipping.")
            continue
            
        c.execute('SELECT timestamp, data FROM messages WHERE topic_id = ?', (topic_id,))
        print(f"Reading topic: {topic_name} from DB...")
        
        for t, data in c.fetchall():
            try:
                deserialized_msg = deserialize_message(bytes(data), msg_type)
                all_data[topic_name].append((t, deserialized_msg))
            except Exception:
                pass
                
    conn.close()
    return all_data

def find_closest_data(time_ns, data_list):
    """ナノ秒単位の時刻に最も近いデータをリストから検索して返す"""
    if not data_list:
        return None
        
    timestamps = np.array([t for t, msg in data_list])
    if timestamps.size == 0:
        return None

    idx = np.argmin(np.abs(timestamps - time_ns))
    return data_list[idx][1]

def get_transform_matrix(tf_msg):
    """TransformStampedメッセージから4x4の同次変換行列を生成する"""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    
    R = quat2mat([q.w, q.x, q.y, q.z])
    M = compose([t.x, t.y, t.z], R, [1, 1, 1])
    return M

def transform_point_cloud(points_xyz, matrix):
    """点群データに4x4変換行列を適用する"""
    if points_xyz.size == 0:
        return points_xyz

    points_homogeneous = np.hstack((points_xyz, np.ones((points_xyz.shape[0], 1))))
    transformed_points_homogeneous = points_homogeneous @ matrix.T
    transformed_points_xyz = transformed_points_homogeneous[:, :3] / transformed_points_homogeneous[:, 3].reshape(-1, 1)
    
    return transformed_points_xyz

def save_waypoints_to_json(waypoints, wp_dir, file_name):
    """抽出されたウェイポイントデータをJSONファイルに保存する"""
    save_path = os.path.join(wp_dir, file_name)
    os.makedirs(wp_dir, exist_ok=True)
    
    try:
        with open(save_path, 'w', encoding='utf-8') as f:
            json.dump(waypoints, f, indent=4)
        print(f"\n✅ Waypoints saved successfully to {save_path}")
        print(f"   Total waypoints: {len(waypoints)}")
    except Exception as e:
        print(f"\n❌ Error saving waypoints to JSON: {e}")


# --- メイン処理 ---

def process_bag_data(all_data):
    """bagから抽出したデータを使ってC++ノードのロジックを再現し、ウェイポイントを生成する"""
    
    pcd_list = all_data.get(TOPICS["PCD"], [])
    odom_list = all_data.get(TOPICS["ODOM"], [])
    
    # ウェイポイント抽出のための変数
    waypoints_data = []
    last_wp_position = {'x': 1e6, 'y': 1e6, 'z': 1e6} 
    
    # 点群累積のための変数 
    last_pcd_position = {'x': 1e6, 'y': 1e6, 'z': 1e6}
    current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
    combined_pcd = []
    
    if not odom_list:
        print("Error: Odometry data not found. Cannot generate waypoints.")
        return

    print(f"\nStarting data processing...")
    print(f"  PointCloud Distance Filter: {PC_SAVE_DISTANCE} m")
    print(f"  Waypoint Distance Filter: {WP_SAVE_DISTANCE} m")

    for count, (odom_time_ns, odom_msg) in enumerate(odom_list):
        
        # 1. オドメトリの位置を取得 (TARGET_FRAMEでの位置)
        current_position['x'] = odom_msg.pose.pose.position.x
        current_position['y'] = odom_msg.pose.pose.position.y
        current_position['z'] = odom_msg.pose.pose.position.z
        
        q = odom_msg.pose.pose.orientation # 姿勢 (クォータニオン)

        # 2. ウェイポイントの距離フィルタリング
        dx_wp = current_position['x'] - last_wp_position['x']
        dy_wp = current_position['y'] - last_wp_position['y']
        dz_wp = current_position['z'] - last_wp_position['z']
        distance_wp = math.sqrt(dx_wp*dx_wp + dy_wp*dy_wp + dz_wp*dz_wp)

        if distance_wp >= WP_SAVE_DISTANCE:
            # フィルタリング通過 -> ウェイポイントとして保存
            last_wp_position = current_position.copy()
            
            # ウェイポイントのJSON形式を生成
            waypoint = [
                # [x, y, z] - zは0.0を強制
                [
                    current_position['x'],
                    current_position['y'],
                    0.0
                ],
                # [0.0, 0.0, z, w] - クォータニオンの姿勢 (yawのみ)
                [
                    0.0,
                    0.0,
                    q.z, 
                    q.w
                ],
                # 付帯情報
                {
                    "type": "normal",
                    "value": 0,
                    "xy_tolerance": 1.0,
                    "yaw_tolerance": 3.14
                }
            ]
            waypoints_data.append(waypoint)
            print(f"  [Index {count:05}] Waypoint added at ({current_position['x']:.2f}, {current_position['y']:.2f}). Total: {len(waypoints_data)}")

            
        # 3. 点群の距離フィルタリング (既存ロジックの再現)
        dx_pcd = current_position['x'] - last_pcd_position['x']
        dy_pcd = current_position['y'] - last_pcd_position['y']
        dz_pcd = current_position['z'] - last_pcd_position['z']
        distance_pcd = math.sqrt(dx_pcd*dx_pcd + dy_pcd*dy_pcd + dz_pcd*dz_pcd)

        if distance_pcd >= PC_SAVE_DISTANCE:
            # フィルタリング通過 -> PCL処理を行う
            last_pcd_position = current_position.copy()
            
            # 4. 点群メッセージを取得 (オドメトリ時刻に最も近いもの)
            pcd_msg = find_closest_data(odom_time_ns, pcd_list)
            if not pcd_msg:
                continue

            # 5. TF変換を取得 (OdometryをTF代替として使用)
            
            # Odometryは TARGET_FRAME -> ORIG_FRAME の変換を保持していると仮定
            if odom_msg.header.frame_id == TARGET_FRAME and ORIG_FRAME == odom_msg.child_frame_id:
                p = odom_msg.pose.pose.position
                q_odom = odom_msg.pose.pose.orientation
                
                R = quat2mat([q_odom.w, q_odom.x, q_odom.y, q_odom.z])
                transform_matrix = compose([p.x, p.y, p.z], R, [1, 1, 1])
                
                # 6. 点群データ変換と累積
                try:
                    points_structured = np.array(list(point_cloud2.read_points(
                        pcd_msg, 
                        field_names=("x", "y", "z"), 
                        skip_nans=True
                    )))
                except Exception:
                    points_structured = np.array([])
                
                
                if points_structured.size == 0:
                    points_xyz = np.array([])
                else:
                    points_xyz = np.column_stack([
                        points_structured['x'],
                        points_structured['y'],
                        points_structured['z']
                    ]).astype(np.float32)

                if points_xyz.size == 0:
                    print(f"  [Index {count:05}] Warning: Empty or invalid PointCloud. Skipping.")
                    continue

                transformed_points = transform_point_cloud(points_xyz, transform_matrix)
                combined_pcd.append(transformed_points)
            
            else:
                 pass

    # 7. 地図の保存
    if combined_pcd:
        final_map_points = np.vstack(combined_pcd)
        save_path = os.path.join(MAP_DIR, MAP_NAME)
        os.makedirs(MAP_DIR, exist_ok=True)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(final_map_points)
        
        o3d.io.write_point_cloud(save_path, pcd, write_ascii=True)
        print(f"\n--- Processing Finished ---")
        print(f"Total points saved: {final_map_points.shape[0]} points.")
        print(f"Saved map to {save_path}")
    else:
        print("\n--- Processing Finished ---")
        print("No point clouds were saved due to filtering or empty data.")

    # 8. Waypointの保存
    if waypoints_data:
        save_waypoints_to_json(waypoints_data, WP_DIR, WP_FILE_NAME)


if __name__ == "__main__":
    
    if len(sys.argv) < 12: 
        print("Usage: python pcd_tf_extractor.py <bag_folder> <sub_pcd_topic> <sub_odom_topic> <pub_topic(ignored)> <orig_frame> <target_frame> <map_dir> <map_name> <wp_dir> <pc_save_distance> <wp_save_distance> [<tf_topic(optional, default:/tf)>]")
        print("\nExample: python pcd_tf_extractor.py /path/to/bag /velodyne_points /lio_odom base_link odom ./output_map map.pcd ./output_waypoints 1.0 2.0")
        sys.exit(1)

    # コマンドライン引数から設定を読み込み
    bag_folder = os.path.normpath(os.path.join(os.getcwd(), sys.argv[1]))
    TOPICS["PCD"] = sys.argv[2]
    TOPICS["ODOM"] = sys.argv[3]
    # sys.argv[4] は pub_topic で、オフライン処理では無視
    ORIG_FRAME = sys.argv[5]
    TARGET_FRAME = sys.argv[6]
    
    # 7番目の引数: MAP_DIR (点群マップのディレクトリ)
    MAP_DIR = sys.argv[7] 
    
    # 📌 8番目の引数: MAP_NAME (地図ファイル名)
    MAP_NAME = sys.argv[8]
    
    # 📌 9番目の引数: WP_DIR (ウェイポイントディレクトリ)
    WP_DIR = sys.argv[9] 
    WP_FILE_NAME = os.path.splitext(MAP_NAME)[0] + ".json" 
    # 10番目の引数: PC_SAVE_DISTANCE
    try:
        PC_SAVE_DISTANCE = float(sys.argv[10])
    except ValueError:
        print("Error: pc_save_distance must be a number.")
        sys.exit(1)
        
    # 11番目の引数: WP_SAVE_DISTANCE
    try:
        WP_SAVE_DISTANCE = float(sys.argv[11])
    except ValueError:
        print("Error: wp_save_distance must be a number.")
        sys.exit(1)
    
    if len(sys.argv) == 13: 
        TOPICS["TF"] = sys.argv[12]
    
    print(f"Config: PCD={TOPICS['PCD']}, ODOM={TOPICS['ODOM']}, TF={TOPICS['TF']}")
    print(f"Config: Frames={ORIG_FRAME} -> {TARGET_FRAME}")
    print(f"Config: MAP_DIR={MAP_DIR}, MAP_NAME={MAP_NAME}, WP_DIR={WP_DIR}")
    print(f"Config: Filters: PCD={PC_SAVE_DISTANCE}m, WP={WP_SAVE_DISTANCE}m")
    print(f"NOTE: Assuming PointCloud frame (header.frame_id) is effectively equivalent to Odometry child frame ('{ORIG_FRAME}').") 

    mcap_files = glob.glob(os.path.join(bag_folder, '*.mcap'))
    db_file = find_db_file(bag_folder)
    
    topics_to_read = list(TOPICS.values())
    
    all_data = {}

    if mcap_files:
        print(f"\nFound MCAP file: {mcap_files[0]}")
        all_data = read_all_messages_from_bag(mcap_files[0], 'mcap', topics_to_read)
    elif db_file:
        print(f"\nFound DB file: {db_file}")
        all_data = get_db_messages(db_file, topics_to_read)
    else:
        print(f"Error: No .mcap or .db3 files found in '{bag_folder}'.")
        sys.exit(1)
        
    process_bag_data(all_data)