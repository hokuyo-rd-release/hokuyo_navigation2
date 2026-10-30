#!/usr/bin/env python3
"""loc モード (simple_fastlio_localization) の自己位置推定の良し悪しを照合ごとに記録するノード.

localization_node は照合の成否 (収束したか) しか持たず、一致度や見失いを出力しない.
このノードは照合のたびに出る /loc_registered_cloud (地図座標に変換したスキャン) を
地図 PCD と突き合わせ、次の値を CSV に残す.

  registration.csv  照合 1 回ごと
    - converged      GICP が収束したか (失敗時は localization_node が intensity を 0 にする)
    - inlier_XX      スキャン点のうち地図点まで XX m 以内にある割合 (一致率)
    - rmse_in        地図点まで 0.5 m 以内の点の RMSE [m]
    - jump_xy/yaw    前回の照合からの map->odom の変化 (= その照合での補正量)
    - drift_xy/yaw   開始時からの map->odom の変化 (= LIO の累積誤差を地図照合で打ち消した量)
    - est_*          /estimated_pose (地図上の推定位置姿勢)
    - lap, wp        waypoint_manager のログから求めた周回番号と直近の waypoint
  events.csv        waypoint 到達・停止・周回・見失いの疑い/復帰

見失いの疑い: inlier_XX < lost_inlier_ratio、または補正量が lost_jump_xy / lost_jump_yaw を
超えた照合が lost_count 回続いたとき (WARN). 一致率が戻ったら復帰として記録する.

実行例:
  ros2 run hokuyo_navigation2 loc_eval.py --ros-args \\
    -p map_pcd:=$HOME/colcon_ws/src/hokuyo_navigation2/hokuyo_navigation2/map/test.pcd \\
    -p csv_dir:=$HOME/colcon_ws/src/hokuyo_navigation2/hokuyo_navigation2/rosbag/loc_eval
"""
import csv
import math
import os
import re
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

# 照合 1 回に使う点数の上限 (KD 木の検索時間を抑える)
MAX_EVAL_POINTS = 20000
RMSE_RANGE = 0.5

REG_FIELDS = [
    'stamp', 'wall_time', 'lap', 'wp', 'dist', 'converged', 'n_points',
    'inlier_0.1', 'inlier_0.2', 'inlier_0.3', 'rmse_in', 'median_dist',
    'jump_xy', 'jump_yaw_deg', 'drift_xy', 'drift_yaw_deg',
    'est_x', 'est_y', 'est_z', 'est_roll_deg', 'est_pitch_deg', 'est_yaw_deg',
    'lio_x', 'lio_y', 'lio_z', 'lio_yaw_deg', 'suspect',
]
EVENT_FIELDS = [
    'stamp', 'wall_time', 'event', 'lap', 'wp', 'dist',
    'est_x', 'est_y', 'est_z', 'est_yaw_deg', 'inlier', 'detail',
]

RE_REACHED = re.compile(r'Reached waypoint (\d+)')
RE_PASSED = re.compile(r'Waypoint (\d+) has been passed')
RE_SKIP = re.compile(r'[Ss]kipping (?:current )?waypoint ?(\d+)?')
RE_STOP = re.compile(r'Stopping for ([\d.]+) seconds')


def to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def quat_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_pcd_xyz(path):
    """PCD (ascii / binary) から xyz を読む. binary_compressed は未対応."""
    header = {}
    with open(path, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError(f'PCD のヘッダが不正です: {path}')
            text = line.decode('ascii', 'ignore').strip()
            if not text or text.startswith('#'):
                continue
            key, _, value = text.partition(' ')
            header[key.upper()] = value.split()
            if key.upper() == 'DATA':
                break
        fields = header['FIELDS']
        sizes = [int(s) for s in header['SIZE']]
        types = header['TYPE']
        counts = [int(c) for c in header.get('COUNT', ['1'] * len(fields))]
        n = int(header['POINTS'][0])
        data_type = header['DATA'][0]
        if data_type == 'ascii':
            arr = np.loadtxt(f, dtype=np.float64, ndmin=2)
            cols = np.cumsum([0] + counts)
            return np.stack([arr[:, cols[fields.index(c)]] for c in 'xyz'], axis=1)
        if data_type != 'binary':
            raise ValueError(f'PCD の DATA {data_type} には対応していません: {path}')
        kind = {'F': 'f', 'I': 'i', 'U': 'u'}
        dtype = np.dtype([(name, f'<{kind[t]}{s}', (c,)) if c > 1 else (name, f'<{kind[t]}{s}')
                          for name, s, t, c in zip(fields, sizes, types, counts)])
        arr = np.frombuffer(f.read(dtype.itemsize * n), dtype=dtype, count=n)
        return np.stack([arr[c].astype(np.float64) for c in 'xyz'], axis=1)


def voxel_downsample(points, size):
    if size <= 0:
        return points
    keys = np.floor(points / size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(idx)]


class LocEval(Node):

    def __init__(self):
        super().__init__('loc_eval')
        self.declare_parameter('map_pcd', '')
        self.declare_parameter('csv_dir', '')
        self.declare_parameter('map_voxel', 0.1)
        self.declare_parameter('report_period', 5.0)
        self.declare_parameter('inlier_dist', 0.2)
        self.declare_parameter('lost_inlier_ratio', 0.5)
        self.declare_parameter('lost_jump_xy', 0.3)
        self.declare_parameter('lost_jump_yaw_deg', 5.0)
        self.declare_parameter('lost_count', 3)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('waypoint_node', 'waypoint_manager')
        self.report_period = self.get_parameter('report_period').value
        self.inlier_dist = self.get_parameter('inlier_dist').value
        self.lost_inlier_ratio = self.get_parameter('lost_inlier_ratio').value
        self.lost_jump_xy = self.get_parameter('lost_jump_xy').value
        self.lost_jump_yaw = math.radians(self.get_parameter('lost_jump_yaw_deg').value)
        self.lost_count = self.get_parameter('lost_count').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.waypoint_node = self.get_parameter('waypoint_node').value

        map_pcd = os.path.expanduser(self.get_parameter('map_pcd').value)
        if not map_pcd or not os.path.exists(map_pcd):
            raise RuntimeError(f'map_pcd が見つかりません: "{map_pcd}"')
        t0 = time.monotonic()
        map_points = voxel_downsample(load_pcd_xyz(map_pcd), self.get_parameter('map_voxel').value)
        self.map_tree = cKDTree(map_points)
        self.get_logger().info(
            f'地図 {map_pcd} を読み込みました ({len(map_points)} 点, {time.monotonic() - t0:.1f}s)')

        self.est = None            # (stamp, x, y, z, roll, pitch, yaw)
        self.lio = None            # (x, y, z, yaw)
        self.map_odom = None       # (x, y, yaw) 最新の map->odom
        self.map_odom_first = None
        self.map_odom_last_reg = None
        self.dist = 0.0
        self.last_xy = None
        self.lap = 1
        self.wp = -1
        self.suspect_run = 0
        self.lost = False
        self.n_reg = 0
        self.window = []           # report_period 内の照合結果 (inlier, converged)
        self.lost_events = 0

        self.reg_file = self.reg_writer = self.event_file = self.event_writer = None
        csv_dir = os.path.expanduser(self.get_parameter('csv_dir').value)
        if csv_dir:
            run_dir = os.path.join(csv_dir, datetime.now().strftime('loc_eval_%Y%m%d_%H%M%S'))
            os.makedirs(run_dir, exist_ok=True)
            self.reg_file = open(os.path.join(run_dir, 'registration.csv'), 'w', newline='')
            self.reg_writer = csv.DictWriter(self.reg_file, fieldnames=REG_FIELDS)
            self.reg_writer.writeheader()
            self.event_file = open(os.path.join(run_dir, 'events.csv'), 'w', newline='')
            self.event_writer = csv.DictWriter(self.event_file, fieldnames=EVENT_FIELDS)
            self.event_writer.writeheader()
            with open(os.path.join(run_dir, 'params.txt'), 'w') as f:
                for name in sorted(self._parameters):
                    f.write(f'{name}: {self.get_parameter(name).value}\n')
            self.get_logger().info(f'CSV を {run_dir} に書き出します')

        self.create_subscription(PoseStamped, '/estimated_pose', self.pose_cb, 10)
        self.create_subscription(Odometry, '/rsf/lio_imu_rate_odom', self.lio_cb, qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, QoSProfile(depth=100))
        self.create_subscription(PointCloud2, '/loc_registered_cloud', self.reg_cb, 10)
        self.create_subscription(Log, '/rosout', self.rosout_cb, QoSProfile(depth=100))
        self.create_timer(self.report_period, self.report)

    # --- 入力 ---

    def pose_cb(self, msg):
        p = msg.pose.position
        roll, pitch, yaw = quat_to_rpy(msg.pose.orientation)
        self.est = (to_sec(msg.header.stamp), p.x, p.y, p.z, roll, pitch, yaw)
        if self.last_xy is not None:
            self.dist += math.hypot(p.x - self.last_xy[0], p.y - self.last_xy[1])
        self.last_xy = (p.x, p.y)

    def lio_cb(self, msg):
        p = msg.pose.pose.position
        self.lio = (p.x, p.y, p.z, quat_to_rpy(msg.pose.pose.orientation)[2])

    def tf_cb(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id == self.map_frame and tf.child_frame_id == self.odom_frame:
                t = tf.transform.translation
                self.map_odom = (t.x, t.y, quat_to_rpy(tf.transform.rotation)[2])
                if self.map_odom_first is None:
                    self.map_odom_first = self.map_odom

    def rosout_cb(self, msg):
        if self.waypoint_node not in msg.name:
            return
        text = msg.msg
        if 'Looping waypoints' in text:
            self.lap += 1
            self.event('lap_start', detail=f'lap={self.lap}')
            return
        for regex, name in ((RE_REACHED, 'reached'), (RE_PASSED, 'passed')):
            m = regex.search(text)
            if m:
                self.wp = int(m.group(1))
                self.event(name)
                return
        m = RE_STOP.search(text)
        if m:
            self.event('stop', detail=f'{m.group(1)}s')
            return
        if 'Stop duration completed' in text:
            self.event('resume')
            return
        m = RE_SKIP.search(text)
        if m:
            self.event('skip', detail=text)

    def reg_cb(self, msg):
        names = [f.name for f in msg.fields]
        fields = ['x', 'y', 'z'] + (['intensity'] if 'intensity' in names else [])
        pts = point_cloud2.read_points_numpy(msg, field_names=fields, skip_nans=True)
        if len(pts) == 0:
            return
        converged = bool(np.any(pts[:, 3] != 0.0)) if pts.shape[1] > 3 else True
        xyz = pts[:, :3].astype(np.float64)
        if len(xyz) > MAX_EVAL_POINTS:
            xyz = xyz[np.random.default_rng(self.n_reg).choice(len(xyz), MAX_EVAL_POINTS, replace=False)]
        d, _ = self.map_tree.query(xyz, distance_upper_bound=5.0, workers=-1)
        d = np.where(np.isfinite(d), d, 5.0)
        near = d[d < RMSE_RANGE]
        inl = {r: float(np.mean(d < r)) for r in (0.1, 0.2, 0.3)}
        inlier = float(np.mean(d < self.inlier_dist))

        jump_xy = jump_yaw = drift_xy = drift_yaw = float('nan')
        if self.map_odom is not None:
            if self.map_odom_last_reg is not None:
                jump_xy = math.hypot(self.map_odom[0] - self.map_odom_last_reg[0],
                                     self.map_odom[1] - self.map_odom_last_reg[1])
                jump_yaw = abs(wrap(self.map_odom[2] - self.map_odom_last_reg[2]))
            drift_xy = math.hypot(self.map_odom[0] - self.map_odom_first[0],
                                  self.map_odom[1] - self.map_odom_first[1])
            drift_yaw = wrap(self.map_odom[2] - self.map_odom_first[2])
            self.map_odom_last_reg = self.map_odom

        suspect = (not converged) or inlier < self.lost_inlier_ratio \
            or (jump_xy > self.lost_jump_xy) or (jump_yaw > self.lost_jump_yaw)
        self.suspect_run = self.suspect_run + 1 if suspect else 0
        if not self.lost and self.suspect_run >= self.lost_count:
            self.lost = True
            self.lost_events += 1
            detail = (f'inlier={inlier:.2f} jump={jump_xy:.2f}m/{math.degrees(jump_yaw):.1f}deg '
                      f'converged={converged}')
            self.get_logger().warn(f'見失いの疑い (lap={self.lap} wp={self.wp}): {detail}')
            self.event('lost_suspect', inlier=inlier, detail=detail)
        elif self.lost and self.suspect_run == 0:
            self.lost = False
            self.get_logger().info(f'一致率が戻りました (lap={self.lap} inlier={inlier:.2f})')
            self.event('recovered', inlier=inlier)

        self.n_reg += 1
        self.window.append((inlier, converged))
        if self.reg_writer is None:
            return
        est = self.est or (float('nan'),) * 7
        lio = self.lio or (float('nan'),) * 4
        self.reg_writer.writerow({
            'stamp': f'{to_sec(msg.header.stamp):.3f}', 'wall_time': f'{time.time():.3f}',
            'lap': self.lap, 'wp': self.wp, 'dist': f'{self.dist:.2f}',
            'converged': int(converged), 'n_points': len(pts),
            'inlier_0.1': f'{inl[0.1]:.4f}', 'inlier_0.2': f'{inl[0.2]:.4f}', 'inlier_0.3': f'{inl[0.3]:.4f}',
            'rmse_in': f'{math.sqrt(np.mean(near ** 2)) if len(near) else float("nan"):.4f}',
            'median_dist': f'{float(np.median(d)):.4f}',
            'jump_xy': f'{jump_xy:.4f}', 'jump_yaw_deg': f'{math.degrees(jump_yaw):.3f}',
            'drift_xy': f'{drift_xy:.4f}', 'drift_yaw_deg': f'{math.degrees(drift_yaw):.3f}',
            'est_x': f'{est[1]:.4f}', 'est_y': f'{est[2]:.4f}', 'est_z': f'{est[3]:.4f}',
            'est_roll_deg': f'{math.degrees(est[4]):.3f}', 'est_pitch_deg': f'{math.degrees(est[5]):.3f}',
            'est_yaw_deg': f'{math.degrees(est[6]):.3f}',
            'lio_x': f'{lio[0]:.4f}', 'lio_y': f'{lio[1]:.4f}', 'lio_z': f'{lio[2]:.4f}',
            'lio_yaw_deg': f'{math.degrees(lio[3]):.3f}', 'suspect': int(suspect),
        })
        self.reg_file.flush()

    # --- 出力 ---

    def event(self, name, inlier=None, detail=''):
        if self.event_writer is None:
            return
        est = self.est or (float('nan'),) * 7
        if inlier is None:
            inlier = self.window[-1][0] if self.window else float('nan')
        self.event_writer.writerow({
            'stamp': f'{est[0]:.3f}', 'wall_time': f'{time.time():.3f}', 'event': name,
            'lap': self.lap, 'wp': self.wp, 'dist': f'{self.dist:.2f}',
            'est_x': f'{est[1]:.4f}', 'est_y': f'{est[2]:.4f}', 'est_z': f'{est[3]:.4f}',
            'est_yaw_deg': f'{math.degrees(est[6]):.3f}', 'inlier': f'{inlier:.4f}', 'detail': detail,
        })
        self.event_file.flush()

    def report(self):
        if not self.window:
            self.get_logger().info(f'照合結果を未受信 (/loc_registered_cloud) lap={self.lap} wp={self.wp}')
            return
        inl = np.array([w[0] for w in self.window])
        fail = sum(1 for w in self.window if not w[1])
        drift = ''
        if self.map_odom is not None and self.map_odom_first is not None:
            drift = (f' drift={math.hypot(self.map_odom[0] - self.map_odom_first[0], self.map_odom[1] - self.map_odom_first[1]):.2f}m/'
                     f'{math.degrees(wrap(self.map_odom[2] - self.map_odom_first[2])):+.1f}deg')
        self.get_logger().info(
            f'lap={self.lap} wp={self.wp} dist={self.dist:.1f}m reg={len(self.window)} '
            f'({len(self.window) / self.report_period:.1f}Hz) not_conv={fail} '
            f'inlier@{self.inlier_dist}m mean={inl.mean():.2f} min={inl.min():.2f}{drift} '
            f'lost_events={self.lost_events}{" **見失い疑い中**" if self.lost else ""}')
        self.window = []

    def destroy_node(self):
        for f in (self.reg_file, self.event_file):
            if f:
                f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = LocEval()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
