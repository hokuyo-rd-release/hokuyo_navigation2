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
    """
    def __init__(self, pcd_file, output_base_path, params):
        self.pcd_file = pcd_file
        self.output_base_path = output_base_path
        self.params = params
        self.cloud = None
        self.cloud_filtered = None
        self.occupancy_grid_data = None
        self.map_info = None

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

    def apply_transform(self):
        """odom_to_lidar_odomパラメータに基づき、逆変換を適用する"""
        if self.cloud is None:
            return

        x, y, z, roll, pitch, yaw = self.params['odom_to_lidar_odom']
        
        # 変換行列を作成 (roll, pitch, yaw -> XYZ)
        R = self.cloud.get_rotation_matrix_from_xyz((roll, pitch, yaw)) 
        T = np.array([[x], [y], [z]])
        
        # Eigen::Affine3f::Identity()からの変換行列
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
        
        points = np.asarray(self.cloud.points)
        
        thre_z_min = self.params['thre_z_min']
        thre_z_max = self.params['thre_z_max']
        
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
        print(f"  (YAML Origin Output: [{x_out}, {y_out}, 0.0])")
        
        return True

    def run(self):
        """全処理を実行する"""
        if not self.load_pcd():
            return False

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
    parser.add_argument("--thre_z_max", type=float, default=10.0, help="Maximum Z threshold for PassThrough filter.") # 🎯 修正済み: 10.0
    parser.add_argument("--flag_pass_through", type=bool, default=False, help="Not used for Z-filter ON/OFF in this Python impl, but kept for parameter consistency.")
    parser.add_argument("--thre_radius", type=float, default=0.5, help="Radius for RadiusOutlier filter search.")
    parser.add_argument("--map_resolution", type=float, default=0.05, help="Resolution of the output map (meters/pixel).")
    parser.add_argument("--thres_point_count", type=int, default=10, help="Minimum number of neighbors in radius for RadiusOutlier filter.")
    
    # 変換行列 (6DOF: x, y, z, roll, pitch, yaw)
    parser.add_argument(
        "--odom_to_lidar_odom", 
        type=float, 
        nargs=6, 
        default=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
        help="6DOF transform parameters (x y z roll pitch yaw) to apply the inverse transform."
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
    }

    # 変換処理を実行
    converter = PcdToPgmConverter(args.pcd_file, args.output_path, params)
    if converter.run():
        return 0
    else:
        return 1

if __name__ == "__main__":
    exit(main())