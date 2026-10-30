#!/usr/bin/env python3
"""GNSS モードの位置系統 (rsf_fix → TF map→base_link) を監視し、どこで止まったかをログに残すノード.

監視対象:
  点群        /hokuyo3d/hokuyo_cloud2   (RSF との通信が生きているかの目安)
  位置系統    /rsf/rsf_fix → /rsf/rsf_fix_with_orient → /rsf/rsf_odom_on_map → TF map→base_link
  向き        /rsf/utm_coord_odom
  RSF の状態  /rsf/rsf_fix_state, /rsf/rsf_fix_type, /rsf/rsf_odom_state, /rsf/rsf_odom_type, /rsf/diagnostics

出力:
  - report_period ごとに各トピックの受信状況 (INFO)
  - 受信の途絶・復帰、スタンプの巻き戻り、RSF の状態変化 (WARN/INFO, 即時)
  - TF が途絶したときは、上流のどこが先に止まったかの判定 (WARN)
  - csv_dir を指定すると report_period ごとの値を CSV にも書き出す

実行例:
  ros2 run hokuyo_navigation2 rsf_check.py
  ros2 run hokuyo_navigation2 rsf_check.py --ros-args -p report_period:=5.0 -p stall_timeout:=3.0 -p csv_dir:=$HOME/.ros/log/rsf_check
"""
import csv
import os
import struct
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

try:
    from fix_msgs.msg import FixWithOrientation
except ImportError:
    FixWithOrientation = None

CLOUD = 'cloud'
FIX = 'rsf_fix'
UTM = 'utm_odom'
FIX_ORIENT = 'fix_with_orient'
ODOM_ON_MAP = 'odom_on_map'
TF = 'tf map->base_link'


def to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Stream:
    """1 トピック分の受信状況."""

    def __init__(self, name, topic):
        self.name = name
        self.topic = topic
        self.total = 0
        self.window = 0
        self.last_rx = None      # time.monotonic()
        self.last_stamp = None   # [s]
        self.last_latency = None  # PC 時刻 - スタンプ [s]
        self.max_gap = 0.0
        self.backward = 0
        self.stalled = False
        self.stalled_since = None

    def age(self, now):
        return None if self.last_rx is None else now - self.last_rx

    def fmt(self, now, period):
        if self.last_rx is None:
            return f'{self.name:<18} 未受信'
        extra = f' backward={self.backward}' if self.backward else ''
        mark = ' **途絶中**' if self.stalled else ''
        return (f'{self.name:<18} rx={self.window:4d} ({self.window / period:5.1f}Hz) '
                f'age={self.age(now):7.2f}s max_gap={self.max_gap:5.2f}s '
                f'now-stamp={self.last_latency:+8.3f}s{extra}{mark}')


class RsfCheck(Node):

    def __init__(self):
        super().__init__('rsf_check')
        self.declare_parameter('report_period', 1.0)
        # 最も遅い GNSS 系のトピックが 1Hz なので、1Hz で 2 回続けて欠けたら途絶とみなす
        self.declare_parameter('stall_timeout', 3.0)
        self.declare_parameter('csv_dir', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.report_period = self.get_parameter('report_period').value
        self.stall_timeout = self.get_parameter('stall_timeout').value
        csv_dir = self.get_parameter('csv_dir').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.streams = {
            CLOUD: Stream(CLOUD, '/hokuyo3d/hokuyo_cloud2'),
            FIX: Stream(FIX, '/rsf/rsf_fix'),
            UTM: Stream(UTM, '/rsf/utm_coord_odom'),
            FIX_ORIENT: Stream(FIX_ORIENT, '/rsf/rsf_fix_with_orient'),
            ODOM_ON_MAP: Stream(ODOM_ON_MAP, '/rsf/rsf_odom_on_map'),
            TF: Stream(TF, '/tf'),
        }
        self.states = {
            '/rsf/rsf_fix_state': None,
            '/rsf/rsf_fix_type': None,
            '/rsf/rsf_odom_state': None,
            '/rsf/rsf_odom_type': None,
        }
        self.fix_status = None
        self.last_fix = None
        self.last_pose = None
        self.diag = {}
        self.diag_level = None

        # 点群は大きいのでデシリアライズせず、CDR の先頭からスタンプだけ読む
        self.create_subscription(PointCloud2, self.streams[CLOUD].topic, self.cloud_cb,
                                 qos_profile_sensor_data, raw=True)
        self.create_subscription(NavSatFix, self.streams[FIX].topic, self.fix_cb, 10)
        self.create_subscription(Odometry, self.streams[UTM].topic,
                                 lambda m: self.on_receive(UTM, to_sec(m.header.stamp)), 10)
        if FixWithOrientation is not None:
            self.create_subscription(FixWithOrientation, self.streams[FIX_ORIENT].topic,
                                     lambda m: self.on_receive(FIX_ORIENT, to_sec(m.fix.header.stamp)), 10)
        else:
            self.get_logger().warn('fix_msgs が見つからないため /rsf/rsf_fix_with_orient は監視しない')
            del self.streams[FIX_ORIENT]
        self.create_subscription(Odometry, self.streams[ODOM_ON_MAP].topic, self.odom_on_map_cb, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, QoSProfile(depth=100))
        for topic in self.states:
            self.create_subscription(String, topic, lambda m, t=topic: self.state_cb(t, m), 10)
        self.create_subscription(DiagnosticArray, '/rsf/diagnostics', self.diag_cb, 10)

        self.csv_file = None
        self.csv_writer = None
        if csv_dir:
            os.makedirs(os.path.expanduser(csv_dir), exist_ok=True)
            path = os.path.join(os.path.expanduser(csv_dir),
                                datetime.now().strftime('rsf_check_%Y%m%d_%H%M%S.csv'))
            self.csv_file = open(path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            header = ['pc_time']
            for name in self.streams:
                name = name.split()[0]
                header += [f'{name}_hz', f'{name}_age', f'{name}_max_gap', f'{name}_now_minus_stamp']
            header += ['fix_status', 'lat', 'lon', 'x', 'y'] + list(self.states) + [
                'diag_level', 'device_temperature', 'cpu_usage', 'odometry_state']
            self.csv_writer.writerow(header)
            self.get_logger().info(f'CSV 出力: {path}')

        self.window_start = time.monotonic()
        self.create_timer(min(0.2, self.report_period), self.check_stall)
        self.create_timer(self.report_period, self.report)
        self.get_logger().info(
            f'起動 (report_period={self.report_period}s, stall_timeout={self.stall_timeout}s, '
            f'TF {self.map_frame}->{self.base_frame})')

    # ---------------- 受信 ----------------

    def on_receive(self, name, stamp):
        s = self.streams[name]
        now = time.monotonic()
        if s.last_rx is None:
            self.get_logger().info(f'[{name}] 初回受信: stamp={stamp:.3f} now-stamp={time.time() - stamp:+.3f}s')
        else:
            s.max_gap = max(s.max_gap, now - s.last_rx)
            if stamp < s.last_stamp:
                s.backward += 1
                self.get_logger().warn(
                    f'[{name}] スタンプが巻き戻った: {s.last_stamp:.3f} -> {stamp:.3f} ({stamp - s.last_stamp:+.3f}s)')
        if s.stalled:
            self.get_logger().warn(
                f'[{name}] 受信再開: {now - s.last_rx:.2f}s 途絶していた '
                f'(stamp {s.last_stamp:.3f} -> {stamp:.3f})')
            s.stalled = False
        s.last_rx = now
        s.last_stamp = stamp
        s.last_latency = time.time() - stamp
        s.total += 1
        s.window += 1

    def cloud_cb(self, data):
        # CDR: 4 バイトのエンカプセレーションヘッダの後に header.stamp (int32 sec, uint32 nanosec)
        fmt = '<iI' if data[1] == 1 else '>iI'
        sec, nanosec = struct.unpack_from(fmt, data, 4)
        self.on_receive(CLOUD, sec + nanosec * 1e-9)

    def fix_cb(self, msg):
        self.on_receive(FIX, to_sec(msg.header.stamp))
        if msg.status.status != self.fix_status:
            log = self.get_logger().warn if msg.status.status < 0 else self.get_logger().info
            log(f'[{FIX}] NavSatStatus 変化: {self.fix_status} -> {msg.status.status}')
            self.fix_status = msg.status.status
        self.last_fix = msg

    def odom_on_map_cb(self, msg):
        self.on_receive(ODOM_ON_MAP, to_sec(msg.header.stamp))
        self.last_pose = msg.pose.pose.position

    def tf_cb(self, msg):
        for t in msg.transforms:
            if t.header.frame_id.lstrip('/') == self.map_frame and t.child_frame_id.lstrip('/') == self.base_frame:
                self.on_receive(TF, to_sec(t.header.stamp))

    def state_cb(self, topic, msg):
        if msg.data != self.states[topic]:
            self.get_logger().info(f'[{topic}] 変化: {self.states[topic]!r} -> {msg.data!r}')
            self.states[topic] = msg.data

    def diag_cb(self, msg):
        for status in msg.status:
            self.diag = {kv.key: kv.value for kv in status.values}
            level = int.from_bytes(status.level, 'little') if isinstance(status.level, bytes) else int(status.level)
            if level != self.diag_level:
                log = self.get_logger().warn if level != 0 else self.get_logger().info
                log(f'[diagnostics] level 変化: {self.diag_level} -> {level} ({status.message}) {self.diag}')
                self.diag_level = level

    # ---------------- 途絶の検出 ----------------

    def check_stall(self):
        now = time.monotonic()
        for name, s in self.streams.items():
            if s.last_rx is None or s.stalled or now - s.last_rx <= self.stall_timeout:
                continue
            s.stalled = True
            s.stalled_since = s.last_rx
            self.get_logger().warn(
                f'[{name}] 受信が途絶: 最終受信から {now - s.last_rx:.2f}s, 最終stamp={s.last_stamp:.3f}')
            if name == TF:
                self.get_logger().warn(f'TF 途絶時の状況: {self.diagnose(now)}\n{self.snapshot(now)}')

    def diagnose(self, now):
        """上流から順に見て、最初に止まっている段を返す (引き継ぎメモ 6.4 の表に対応)."""
        def stalled(name):
            s = self.streams.get(name)
            return s is not None and (s.last_rx is None or now - s.last_rx > self.stall_timeout)

        states = ', '.join(f'{k.split("/")[-1]}={v!r}' for k, v in self.states.items())
        if stalled(FIX) and stalled(CLOUD):
            return 'rsf_fix も点群も止まっている → RSF との通信停止の疑い (ping 192.168.0.100, ss -tni を確認)'
        if stalled(FIX):
            return f'点群は来ているが rsf_fix が止まっている → RSF が位置出力を止めた疑い ({states})'
        if stalled(FIX_ORIENT):
            if self.streams[UTM].last_rx is None:
                return 'utm_coord_odom を一度も受信していない → fix_odom_to_fix_orient が出力しない'
            return 'rsf_fix は来ているが rsf_fix_with_orient が止まっている → fix_odom_to_fix_orient の停止の疑い'
        if stalled(ODOM_ON_MAP):
            return 'rsf_fix_with_orient は来ているが rsf_odom_on_map が止まっている → fix2xyz_out の停止の疑い'
        return 'rsf_odom_on_map は来ているが TF が止まっている → rsf_odom_tf (odom_to_tf) の停止の疑い'

    def snapshot(self, now):
        period = max(now - self.window_start, 1e-3)
        lines = [s.fmt(now, period) for s in self.streams.values()]
        states = '  '.join(f'{k.split("/")[-1]}={v!r}' for k, v in self.states.items())
        lines.append(f'{"rsf state":<18} {states}')
        if self.last_fix is not None:
            lines.append(f'{"rsf_fix":<18} status={self.fix_status} lat={self.last_fix.latitude:.8f} '
                         f'lon={self.last_fix.longitude:.8f}')
        if self.last_pose is not None:
            lines.append(f'{"odom_on_map":<18} x={self.last_pose.x:.3f} y={self.last_pose.y:.3f}')
        if self.diag:
            lines.append(f'{"diagnostics":<18} level={self.diag_level} '
                         f'temp={self.diag.get("device_temperature")}C cpu={self.diag.get("cpu_usage")}% '
                         f'odometry_state={self.diag.get("odometry_state")}')
        return '\n'.join(lines)

    # ---------------- 定期出力 ----------------

    def report(self):
        now = time.monotonic()
        period = max(now - self.window_start, 1e-3)
        self.get_logger().info('\n' + self.snapshot(now))
        if self.csv_writer is not None:
            self.write_csv(now, period)
        for s in self.streams.values():
            s.window = 0
            s.max_gap = 0.0
            s.backward = 0
        self.window_start = now

    def write_csv(self, now, period):
        def r(v, n=3):
            return '' if v is None else round(v, n)
        row = [f'{time.time():.3f}']
        for s in self.streams.values():
            row += [r(s.window / period, 1), r(s.age(now)), r(s.max_gap), r(s.last_latency)]
        fix = self.last_fix
        row += [self.fix_status,
                '' if fix is None else f'{fix.latitude:.8f}', '' if fix is None else f'{fix.longitude:.8f}',
                '' if self.last_pose is None else r(self.last_pose.x),
                '' if self.last_pose is None else r(self.last_pose.y)]
        row += [self.states[k] for k in self.states]
        row += [self.diag_level, self.diag.get('device_temperature'), self.diag.get('cpu_usage'),
                self.diag.get('odometry_state')]
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = RsfCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
