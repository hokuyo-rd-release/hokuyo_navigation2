#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np

from scipy.spatial.transform import Rotation as R

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from kgeom3d import ominus_se3

from ahrs.filters import Madgwick

def load_pcd_index(index_path: Path):
    """
    index file format per line:
      <timestamp_nsec> <pcd_filename>
    separated by whitespace.
    Returns:
      ts_ns: (M,) int64 sorted ascending
      names: (M,) object (str)
    """
    if not index_path.exists():
        raise FileNotFoundError(f"index file not found: {index_path}")

    ts = []
    names = []
    with index_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                t_ns = int(parts[0])
            except ValueError:
                continue
            name = parts[1]
            ts.append(t_ns)
            names.append(name)

    if len(ts) == 0:
        raise RuntimeError(f"index file has no valid entries: {index_path}")

    ts = np.asarray(ts, dtype=np.int64)
    names = np.asarray(names, dtype=object)

    order = np.argsort(ts)
    return ts[order], names[order]


def nearest_pcd_name(odom_ts_ns: int, pcd_ts_ns: np.ndarray, pcd_names: np.ndarray) -> str:
    """
    Find nearest timestamp in pcd_ts_ns to odom_ts_ns (both in nsec).
    Use binary search.
    """
    idx = int(np.searchsorted(pcd_ts_ns, odom_ts_ns))
    if idx <= 0:
        return str(pcd_names[0])
    if idx >= len(pcd_ts_ns):
        return str(pcd_names[-1])

    t0 = pcd_ts_ns[idx - 1]
    t1 = pcd_ts_ns[idx]
    if abs(odom_ts_ns - t0) <= abs(t1 - odom_ts_ns):
        return str(pcd_names[idx - 1])
    return str(pcd_names[idx])


def stamp_to_nsec(msg, bag_time_ns: int) -> int:
    """Prefer header stamp if valid; else fallback to bag time. Return int nanoseconds."""
    try:
        s = int(msg.header.stamp.sec)
        ns = int(msg.header.stamp.nanosec)
        t = s * 1_000_000_000 + ns
        if t > 0:
            return t
    except Exception:
        pass
    return int(bag_time_ns)


def info_upper_triangular_6x6(diag: float) -> list[float]:
    """Return 21 numbers (upper triangle) of 6x6 information matrix with diag filled."""
    I = np.zeros((6, 6), dtype=float)
    np.fill_diagonal(I, diag)
    out = []
    for r in range(6):
        for c in range(r, 6):
            out.append(float(I[r, c]))
    return out


def info_upper_triangular_2x2(diag: float) -> tuple[float, float, float]:
    """Return (info00, info01, info11) for 2x2 info matrix = diag*I."""
    return float(diag), 0.0, float(diag)


def madgwick_update_xyzw(madgwick: Madgwick, q_xyzw: np.ndarray, gyr: np.ndarray, acc: np.ndarray, dt: float) -> np.ndarray:
    """
    ahrs.Madgwick.updateIMU は q=[w,x,y,z] の実装が多いので、
    内部で wxyz <-> xyzw を変換して入出力は xyzw に統一。
    """
    q_wxyz = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]], dtype=float)
    q_wxyz = madgwick.updateIMU(q_wxyz, gyr=gyr, acc=acc, dt=float(dt))
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=float)


def gravity_local_from_orientation_xyzw(q_xyzw: np.ndarray, g_world: np.ndarray = None) -> np.ndarray:
    """
    Compute gravity direction in the *local (pose) frame* from orientation q_xyzw.
    Assumption: q encodes rotation from local->world (typical pose quaternion).
    Then g_local = R(q)^(-1) * g_world.
    """
    if g_world is None:
        g_world = np.array([0.0, 0.0, -1.0], dtype=float)  # world z-up, gravity down
    rot = R.from_quat(q_xyzw)  # xyzw (compatible with scipy < 1.14.0)
    g_local = rot.inv().apply(g_world)
    return g_local


def main():
    ap = argparse.ArgumentParser(
        description="Export p2o2 (extended g2o) graph from ROS2 bag: odom SE3 edges + IMU gravity edges."
    )
    ap.add_argument("bag", help="Path to rosbag2 directory (contains metadata.yaml)")
    ap.add_argument("--odom-topic", default="/hokuyo_lio/lidar_odom", help="Odometry topic (nav_msgs/msg/Odometry)")
    ap.add_argument("--imu-topic", default="/hokuyo3d/imu", help="IMU topic (sensor_msgs/msg/Imu)")
    ap.add_argument("--stride", type=int, default=10, help="Keep every Nth odom message (default: 10)")
    ap.add_argument("--out", default="graph.p2o", help="Output p2o(g2o-style) file")
    ap.add_argument("--madgwick-gain", type=float, default=0.033, help="ahrs.Madgwick gain")
    ap.add_argument("--odom-info", type=float, default=1.0, help="EDGE_SE3:QUAT information diagonal (default: 1.0)")
    ap.add_argument("--grav-info", type=float, default=0.1, help="EDGE_GRAVITY information diagonal (2x2) (default: 1.0)")
    ap.add_argument("--pcd-dir", required=True, help="Directory containing PCD files and index file")
    args = ap.parse_args()

    if args.stride <= 0:
        raise ValueError("--stride must be >= 1")

    # Load PCD index
    pcd_dir = Path(args.pcd_dir)
    index_path = pcd_dir / "index.txt"
    pcd_ts_ns, pcd_names = load_pcd_index(index_path)

    # Open bag
    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("cdr", "cdr")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # Topic types
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if args.odom_topic not in type_map:
        raise RuntimeError(f"Odom topic '{args.odom_topic}' not found.\nAvailable:\n  " + "\n  ".join(sorted(type_map)))
    if args.imu_topic not in type_map:
        raise RuntimeError(f"IMU topic '{args.imu_topic}' not found.\nAvailable:\n  " + "\n  ".join(sorted(type_map)))

    OdomMsg = get_message(type_map[args.odom_topic])
    ImuMsg = get_message(type_map[args.imu_topic])

    # Filter topics for speed
    try:
        reader.set_filter(rosbag2_py.StorageFilter(topics=[args.odom_topic, args.imu_topic]))
    except Exception:
        pass

    # IMU filter state (xyzw)
    madgwick = Madgwick(gain=args.madgwick_gain)
    q_imu = None
    t_imu_prev = None
    t_imu_latest = None

    poses = []
    grav_meas = []       # gravity vector in local frame for each kept vertex (or None)
    node_times = []
    pcd_files = []

    odom_count = 0

    # Read in time order; maintain latest IMU orientation
    while reader.has_next():
        topic, data, t_ns = reader.read_next()

        if topic == args.imu_topic:
            msg = deserialize_message(data, ImuMsg)
            t = stamp_to_nsec(msg, t_ns)

            if t_imu_prev is None:
                t_imu_prev = t
                q_imu = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)  # xyzw
                t_imu_latest = t
                continue

            dt = (t - t_imu_prev)/1e9
            t_imu_prev = t
            if not np.isfinite(dt) or dt <= 0.0:
                continue

            # for hokuyo3d
            gyr = np.array([msg.angular_velocity.z, msg.angular_velocity.x, msg.angular_velocity.y], dtype=float)  # rad/s
            acc = np.array([msg.linear_acceleration.z, msg.linear_acceleration.x, msg.linear_acceleration.y], dtype=float)

            q_imu = madgwick_update_xyzw(madgwick, q_imu, gyr=gyr, acc=acc, dt=dt)
            t_imu_latest = t

        elif topic == args.odom_topic:
            msg = deserialize_message(data, OdomMsg)
            odom_count += 1

            # decimate
            if (odom_count % args.stride) != 0:
                continue

            t = stamp_to_nsec(msg, t_ns)
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation

            p_xyz = np.array([float(p.x), float(p.y), float(p.z)], dtype=float)
            q_xyzw = np.array([float(o.x), float(o.y), float(o.z), float(o.w)], dtype=float)

            poses.append(np.array(np.concatenate([p_xyz, q_xyzw]), dtype=float))
            node_times.append(t)
            pcdfile = nearest_pcd_name(int(t), pcd_ts_ns, pcd_names)
            pcd_files.append(args.pcd_dir + "/" + pcdfile)

            # Gravity measurement from latest IMU orientation (NOT odom orientation)
            if q_imu is None or t_imu_latest is None:
                grav_meas.append(None)
            else:
                g_local = gravity_local_from_orientation_xyzw(q_imu)  # unit vector
                grav_meas.append(g_local)

    kept = len(poses)
    if kept == 0:
        raise RuntimeError("No odom poses !")

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    info_se3 = info_upper_triangular_6x6(args.odom_info)
    info_g00, info_g01, info_g11 = info_upper_triangular_2x2(args.grav_info)

    with out_path.open("w", encoding="utf-8") as f:
        # node 0 fixed upright origin (recommended usage) :contentReference[oaicite:1]{index=1}
        f.write("VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n")

        # vertices 1..N from odom absolute poses
        for k, p in enumerate(poses, start=1):
            pcdfile = " " + pcd_files[k-1]
            f.write(
                f"VERTEX_SE3:QUAT {k} "
                + " ".join(f"{v:.9f}" for v in p) + pcdfile + "\n"
            )
        # odom edges:
        # 0->1 and consecutive (i->i+1), relative from odom poses
        diff = ominus_se3(poses[0], np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=float))
        f.write(
            "EDGE_SE3:QUAT 0 1 "
            + " ".join(f"{v:.9f}" for v in diff) + " "
            + " ".join(f"{v:.6f}" for v in info_se3) + "\n"
        )
        for i in range(1, kept):
            diff = ominus_se3(poses[i], poses[i-1])
            f.write(
                f"EDGE_SE3:QUAT {i} {i+1} "
                + " ".join(f"{v:.9f}" for v in diff) + " "
                + " ".join(f"{v:.6f}" for v in info_se3) + "\n"
            )

        # gravity edges:
        # EDGE_GRAVITY 0 i gx gy gz info00 info01 info11 :contentReference[oaicite:2]{index=2}
        for i, g in enumerate(grav_meas, start=1):
            if g is None:
                continue
            gx, gy, gz = float(g[0]), float(g[1]), float(g[2])
            f.write(
                f"EDGE_GRAVITY 0 {i} "
                f"{gx:.9f} {gy:.9f} {gz:.9f} "
                f"{info_g00:.6f} {info_g01:.6f} {info_g11:.6f}\n"
            )

    print(f"Wrote p2o(g2o-style): {out_path}")
    print(f"Read odom msgs: {odom_count}")
    print(f"Kept vertices (stride={args.stride}): {kept}")
    print(f"Vertices: 0..{kept}")
    print(f"Edges: SE3={kept} (0->1 + consecutive), GRAVITY={sum(g is not None for g in grav_meas)}")


if __name__ == "__main__":
    main()
