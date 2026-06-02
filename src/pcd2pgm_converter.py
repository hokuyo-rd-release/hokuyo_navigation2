import argparse
import os
import yaml
import numpy as np
from PIL import Image

# Open3DはPCLの機能をPythonで代替するのに便利
try:
    import open3d as o3d
    from scipy.spatial import KDTree
except ImportError:
    print("Error: 'open3d' and 'scipy' are required. Please install them:")
    print("pip install open3d numpy scipy pyyaml Pillow")
    exit(1)

# --- 1. PCD処理・変換クラス ---

class PcdToPgmConverter:
    """
    ROS2のpcd2pgmノードの機能をオフラインで実装するクラス。
    PCDファイルの読み込み、変換、フィルタリング、OccupancyGrid生成、PGM/YAML保存を行う。
    Waypointデータに基づいて通行可能領域を上書きする機能を含む。
    """
    def __init__(self, pcd_file, output_base_path, params):
        self.pcd_file = pcd_file
        self.output_base_path = output_base_path
        self.params = params
        self.cloud = None
        self.cloud_filtered = None
        self.occupancy_grid_data = None
        self.map_info = None
        # ウェイポイントデータを保持する新しいメンバー
        self.waypoints = None 

    def load_pcd(self):
        """PCDファイルを読み込む"""
        if not os.path.exists(self.pcd_file):
            print(f"Error: PCD file not found: {self.pcd_file}")
            return False
        
        # Open3DでPCDを読み込む
        self.cloud = o3d.io.read_point_cloud(self.pcd_file)
        if not self.cloud.has_points():
            print(f"Error: Loaded point cloud is empty: {self.pcd_file}")
            return False

        print(f"Initial point cloud size: {len(self.cloud.points)}")
        return True

    def load_waypoints(self):
        """ウェイポイントファイル (JSON/YAML) を読み込む"""
        wp_file = self.params.get('waypoints_file')
        if not wp_file:
            print("Waypoint file not provided. Skipping waypoint processing.")
            return True

        if not os.path.exists(wp_file):
            print(f"Warning: Waypoint file not found: {wp_file}. Skipping waypoint processing.")
            return True

        try:
            with open(wp_file, 'r') as f:
                # PyYAMLはJSON形式のウェイポイントも処理できる (yaml.safe_loadを使用)
                self.waypoints = yaml.safe_load(f) 
            
            if not isinstance(self.waypoints, list) or not self.waypoints:
                 print("Warning: Waypoint file is empty or invalid format. Skipping waypoint processing.")
                 self.waypoints = None
                 return True

            print(f"Loaded {len(self.waypoints)} waypoints.")
            return True
        except Exception as e:
            print(f"Error loading waypoint file {wp_file}: {e}. Skipping waypoint processing.")
            self.waypoints = None
            return True


    def apply_transform(self):
        """odom_to_lidar_odomパラメータに基づき、逆変換を適用する"""
        if self.cloud is None:
            return

        # パラメータ (x, y, z, roll, pitch, yaw)
        x, y, z, roll, pitch, yaw = self.params['odom_to_lidar_odom']
        
        # 変換行列を作成 (roll, pitch, yaw -> XYZ)
        R = self.cloud.get_rotation_matrix_from_xyz((roll, pitch, yaw)) 
        T = np.array([[x], [y], [z]])
        
        # 変換行列
        transform_matrix = np.identity(4)
        transform_matrix[:3, :3] = R
        transform_matrix[:3, 3] = T.flatten()
        
        # C++コードでは transform.inverse() を適用しているため、逆行列を使用
        inverse_transform = np.linalg.inv(transform_matrix)
        
        self.cloud.transform(inverse_transform)
        print("Applied inverse transform to the point cloud.")

    def pass_through_filter(self):
        """Z軸方向のPassThroughフィルタを適用する"""
        if self.cloud is None:
            return
        
        # flag_pass_through が False (または文字列の 'False') の場合はフィルタリングをスキップ
        flag = self.params.get('flag_pass_through', False)
        if str(flag).lower() == 'false':
            print("PassThrough filter is disabled. Using all points for height.")
            self.cloud_filtered = self.cloud
            return

        points = np.asarray(self.cloud.points)
        
        thre_z_min = self.params['thre_z_min']
        thre_z_max = self.params['thre_z_max']
        
        print(f"Applying PassThrough filter (Z: {thre_z_min} to {thre_z_max})")

        # Z軸でフィルタリング
        z_filter = (points[:, 2] >= thre_z_min) & (points[:, 2] <= thre_z_max)
        
        filtered_points = points[z_filter]

        self.cloud_filtered = o3d.geometry.PointCloud()
        self.cloud_filtered.points = o3d.utility.Vector3dVector(filtered_points)

        print(f"After PassThrough filtering: {len(self.cloud_filtered.points)} points")

    def radius_outlier_filter(self):
        """RadiusOutlierフィルタを適用する"""
        if self.cloud_filtered is None:
            return
        
        # KDTreeのセットアップ
        points = np.asarray(self.cloud_filtered.points)
        if points.size == 0:
            print("Warning: Point cloud is empty, skipping RadiusOutlier filtering.")
            return
            
        # KDTreeはx, y, zの3次元で構築
        tree = KDTree(points)
        
        thre_radius = self.params['thre_radius']
        thres_point_count = self.params['thres_point_count']
        
        filtered_points_indices = []
        
        # 各点について、指定半径内の隣接点数をカウント
        for i, point in enumerate(points):
            # query_ball_point で指定半径内の点のインデックスを取得
            indices = tree.query_ball_point(point, thre_radius)
            if len(indices) >= thres_point_count:
                filtered_points_indices.append(i)
        
        self.cloud_filtered.points = o3d.utility.Vector3dVector(points[filtered_points_indices])
        
        print(f"After RadiusOutlier filtering: {len(self.cloud_filtered.points)} points")

    def create_occupancy_grid(self):
        """2D OccupancyGridマップを生成し、PGMデータとYAML情報を作成する"""
        if self.cloud_filtered is None or not self.cloud_filtered.has_points():
            print("Error: Filtered point cloud is empty, cannot create map.")
            return

        # C++の浮動小数点再現のため float32 を経由
        points_32 = np.asarray(self.cloud_filtered.points, dtype=np.float32)
        map_resolution = self.params['map_resolution']

        # 座標の最小・最大値を計算 (X, Yのみ)
        x_min, y_min = np.min(points_32[:, :2], axis=0)
        x_max, y_max = np.max(points_32[:, :2], axis=0)
        
        # グリッドの幅と高さを計算 (C++の std::ceil に対応)
        width = int(np.ceil((x_max - x_min) / map_resolution))
        height = int(np.ceil((y_max - y_min) / map_resolution))
        
        # グリッドの点数カウント配列を初期化
        occupancy_count = np.zeros((height, width), dtype=np.int32)
        
        # 各点についてグリッド座標を計算し、カウント
        for x_point, y_point, _ in points_32:
            # i = X座標インデックス, j = Y座標インデックス
            # C++の std::floor に対応
            i = int(np.floor((x_point - x_min) / map_resolution))
            j = int(np.floor((y_point - y_min) / map_resolution))
            
            # グリッド範囲内かチェック (i: 0 to width-1, j: 0 to height-1)
            if 0 <= i < width and 0 <= j < height:
                occupancy_count[j, i] += 1

        # 占有判定 (ROS OccupancyGrid: 100(Occupied), 0(Free), -1(Unknown))
        ros_grid_data = np.full((height, width), -1, dtype=np.int8) # -1: Unknownで初期化
        
        # Free (カウントゼロ)
        ros_grid_data[occupancy_count == 0] = 0
        
        # Occupied (しきい値以上)
        thres_point_count = self.params['thres_point_count']
        ros_grid_data[occupancy_count >= thres_point_count] = 100
        
        print("Initial ROS grid data generated from PCD count.")

        # =========================================================
        # 🎯 ウェイポイントに基づいて Free 領域を上書き
        # =========================================================
        if self.waypoints:
            print(f"Applying influence of {len(self.waypoints)} waypoints to mark Free space.")
            x_origin = x_min # マップ原点のX座標 (左下)
            y_origin = y_min # マップ原点のY座標 (左下)
            map_res = map_resolution # 変数名を短縮

            # コマンドライン引数から一律の許容誤差を取得
            tolerance = self.params['waypoint_tolerance']
            radius_px = int(np.ceil(tolerance / map_res))
            
            # WaypointのX, Y座標リスト
            wp_coords = []

            # --- 1. Waypoint の中心とその周囲を Free にする ---
            for wp in self.waypoints:
                # ウェイポイントの構造チェック: [position], [orientation], {metadata}
                if len(wp) < 3 or not isinstance(wp[0], list) or len(wp[0]) < 2 or not isinstance(wp[2], dict): 
                    print(f"Warning: Invalid waypoint format found: {wp}. Skipping.")
                    continue 
                
                x_wp, y_wp = wp[0][0], wp[0][1] # 位置 (X, Y) を取得
                wp_coords.append((x_wp, y_wp))
                
                # ウェイポイントのグリッド座標 (i: X, j: Y)
                i_wp = int(np.floor((x_wp - x_origin) / map_res))
                j_wp = int(np.floor((y_wp - y_origin) / map_res))
                
                # ウェイポイントの周囲を Free に上書き (既存ロジック)
                for j in range(max(0, j_wp - radius_px), min(height, j_wp + radius_px + 1)):
                    for i in range(max(0, i_wp - radius_px), min(width, i_wp + radius_px + 1)):
                        ros_grid_data[j, i] = 0 

            # --- 2. Waypoint 同士の間隔を Free にする (線形補間) ---
            if len(wp_coords) >= 2:
                num_waypoints = len(wp_coords)
                
                # 【修正点】ループフラグに基づいて処理回数を決定
                if self.params['loop_waypoints']:
                    # ループの場合: WP0 -> WP1 ... -> WPn-1 -> WP0 (計 N 回)
                    end_idx = num_waypoints
                    print(f"Path treated as a closed loop.")
                else:
                    # ループではない場合: WP0 -> WP1 ... -> WPn-1 (計 N-1 回)
                    end_idx = num_waypoints - 1
                
                print(f"Applying Free space along the paths between waypoints ({'Loop' if self.params['loop_waypoints'] else 'Open'}) with tolerance {tolerance}m.")
                
                # 連続するウェイポイントのペアに対して処理
                for idx in range(end_idx):
                    # 現在のウェイポイント
                    x1, y1 = wp_coords[idx]
                    
                    # 次のウェイポイント (ループ処理のための剰余演算)
                    x2, y2 = wp_coords[(idx + 1) % num_waypoints]

                    # 距離を計算し、補間するステップ数を決定
                    distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                    # 1ピクセルあたり1ステップの精度で補間
                    num_steps = max(2, int(distance / map_res)) 

                    # 線形補間によりパス上の点を生成
                    x_interp = np.linspace(x1, x2, num_steps)
                    y_interp = np.linspace(y1, y2, num_steps)

                    # パス上の各点をグリッド座標に変換
                    x_floor = np.floor((x_interp - x_origin) / map_res)
                    y_floor = np.floor((y_interp - y_origin) / map_res)
                    i_interp = x_floor.astype(int)
                    j_interp = y_floor.astype(int)

                    # グリッド範囲内の有効なインデックスを取得
                    valid_mask = (i_interp >= 0) & (i_interp < width) & \
                                 (j_interp >= 0) & (j_interp < height)
                    
                    valid_i = i_interp[valid_mask]
                    valid_j = j_interp[valid_mask]

                    # パス上のセルとその周囲を Free に上書き
                    for i_path, j_path in zip(valid_i, valid_j):
                        # パス上の点とその周囲 (radius_px) を Free にする
                        for j in range(max(0, j_path - radius_px), min(height, j_path + radius_px + 1)):
                            for i in range(max(0, i_path - radius_px), min(width, i_path + radius_px + 1)):
                                ros_grid_data[j, i] = 0
            
            # Waypointの周囲のFreeマーク処理とパスのFreeマーク処理が統合されます。


        # PGMデータへ変換 (PGM: 0(Occupied/Black), 254(Free/White), 205(Unknown/Gray))
        pgm_data = np.full((height, width), 205, dtype=np.uint8) # 205: Unknown
        pgm_data[ros_grid_data == 0] = 254  # Free: 254 (White)
        
        # 占有セルを PGM値 0 (完全な黒) に設定
        pgm_data[ros_grid_data == 100] = 0  # Occupied: 0 (Black)
        
        # ROS Map Server/PGM は左上を原点とし、Y軸を反転させる必要がある
        pgm_data_final = np.flipud(pgm_data)

        self.occupancy_grid_data = pgm_data_final
        print(f"Generated PGM map: {width}x{height} pixels")

        # 座標を小数点以下2桁に丸めて、リスト形式で設定
        x_min_rounded = float(np.round(x_min, 2))
        y_min_rounded = float(np.round(y_min, 2))

        # YAMLマップ情報
        self.map_info = {
            'image': os.path.basename(self.output_base_path) + '.pgm',
            'resolution': map_resolution,
            # リスト形式 [x, y, z] で代入
            'origin': [x_min_rounded, y_min_rounded, 0.0], 
            'negate': 0, 
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
    def save_files(self):
        """PGM画像とYAML設定ファイルを保存する"""
        if self.occupancy_grid_data is None or self.map_info is None:
            print("Error: Map data not generated. Cannot save files.")
            return False

        pgm_path = self.output_base_path + '.pgm'
        yaml_path = self.output_base_path + '.yaml'

        # PGM画像保存 (L: 8-bit grayscale)
        img = Image.fromarray(self.occupancy_grid_data, mode='L')
        img.save(pgm_path)

        # YAMLファイル保存
        with open(yaml_path, 'w') as f:
            # default_flow_style=True で [x, y, z] のコンパクトなリスト形式で出力
            yaml.dump(self.map_info, f, default_flow_style=True)
        
        # 最終確認メッセージ
        x_out, y_out, _ = self.map_info['origin']
        print(f"\nSuccessfully converted and saved map files.")
        print(f"- PGM Map: {pgm_path}")
        print(f"- YAML Info: {yaml_path}")
        print(f"  (YAML Origin Output: [{x_out}, {y_out}, 0.0])")
        
        return True

    def run(self):
        """全処理を実行する"""
        if not self.load_pcd():
            return False
        
        # ウェイポイントの読み込み
        # ウェイポイントファイルがない場合でも処理は続行するため、return False は使わない
        self.load_waypoints()

        self.apply_transform()
        self.pass_through_filter() 
        self.radius_outlier_filter() 
        self.create_occupancy_grid()
        return self.save_files()


# --- 2. コマンドライン実行部分 ---

def main():
    parser = argparse.ArgumentParser(
        description="Convert PCD map to PGM/YAML OccupancyGrid map offline, mimicking ROS2 pcd2pgm node."
    )
    
    # 必須引数
    parser.add_argument("pcd_file", type=str, help="Path to the input PCD file.")
    parser.add_argument("output_path", type=str, help="Base path for output files (e.g., /path/to/map_base). Output will be <base_path>.pgm and <base_path>.yaml.")

    # オプション引数 (C++ノードのパラメータに対応)
    parser.add_argument("--thre_z_min", type=float, default=0.5, help="Minimum Z threshold for PassThrough filter.")
    parser.add_argument("--thre_z_max", type=float, default=7.0, help="Maximum Z threshold for PassThrough filter.") 
    parser.add_argument("--flag_pass_through", type=bool, default=False, help="Not used for Z-filter ON/OFF in this Python impl, but kept for parameter consistency.")
    parser.add_argument("--thre_radius", type=float, default=0.1, help="Radius for RadiusOutlier filter search.")
    parser.add_argument("--map_resolution", type=float, default=0.05, help="Resolution of the output map (meters/pixel).")
    parser.add_argument("--thres_point_count", type=int, default=1, help="Minimum number of neighbors in radius for RadiusOutlier filter.")
    
    # 変換行列 (6DOF: x, y, z, roll, pitch, yaw)
    parser.add_argument(
        "--odom_to_lidar_odom", 
        type=float, 
        nargs=6, 
        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
        help="6DOF transform parameters (x y z roll pitch yaw) to apply the inverse transform."
    )
    
    # Waypointに対する自由領域の半径
    parser.add_argument(
        "--waypoint_tolerance", 
        type=float, 
        default=1.0, 
        help="Tolerance radius (meters) around waypoints and the path between them to mark as Free space. (Default: 1.0m)"
    )
    
    # ウェイポイントファイル引数
    parser.add_argument(
        "--waypoints_file", 
        type=str, 
        default=None, 
        help="Path to a YAML/JSON file containing waypoint list data to mark 'Free' areas."
    )

    # 【新規追加】ループ処理を有効にするフラグ
    parser.add_argument(
        "--loop_waypoints", 
        action="store_true",  
        help="Treat the waypoints as a closed loop, connecting the last waypoint back to the first one."
    )


    args = parser.parse_args()

    # パラメータ辞書を作成
    params = {
        'thre_z_min': args.thre_z_min,
        'thre_z_max': args.thre_z_max,
        'flag_pass_through': args.flag_pass_through,
        'thre_radius': args.thre_radius,
        'map_resolution': args.map_resolution,
        'thres_point_count': args.thres_point_count,
        'odom_to_lidar_odom': args.odom_to_lidar_odom,
        'waypoints_file': args.waypoints_file, 
        'waypoint_tolerance': args.waypoint_tolerance, 
        # 新しいループフラグ
        'loop_waypoints': args.loop_waypoints, 
    }

    # 変換処理を実行
    converter = PcdToPgmConverter(args.pcd_file, args.output_path, params)
    if converter.run():
        return 0
    else:
        return 1

if __name__ == "__main__":
    try:
        exit(main())
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        exit(1)