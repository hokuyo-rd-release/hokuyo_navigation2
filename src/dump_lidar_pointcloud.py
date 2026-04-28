#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2 as pc2


def open_bag(bag_dir: str) -> SequentialReader:
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_dir, storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def points_to_xyz_array(points_iter):
    pts = list(points_iter)
    if len(pts) == 0:
        return np.empty((0, 3), dtype=np.float64)
    return np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float64)


def save_pcd_binary_open3d(xyz: np.ndarray, out_path: Path):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    o3d.io.write_point_cloud(
        str(out_path),
        cloud,
        write_ascii=False,
        compressed=False,  # => DATA binary
    )


def main():
    ap = argparse.ArgumentParser(
        description="rosbag2_py + Open3D: PointCloud2(XYZ) -> binary PCD + index.txt"
    )
    ap.add_argument("--bag", required=True, help="rosbag2 folder (metadata.yaml があるディレクトリ)")
    ap.add_argument("--topic", required=True, help="PointCloud2 topic name")
    ap.add_argument("--outdir", required=True, help="Output directory")
    ap.add_argument("--prefix", default="cloud", help="PCD filename prefix")
    ap.add_argument("--skip-nans", action="store_true", help="Skip NaN points")
    ap.add_argument("--max-msgs", type=int, default=0, help="Max messages (0 = all)")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    index_path = outdir / "index.txt"

    reader = open_bag(args.bag)

    # topic type check
    topic_type = None
    for t in reader.get_all_topics_and_types():
        if t.name == args.topic:
            topic_type = t.type
            break
    if topic_type != "sensor_msgs/msg/PointCloud2":
        raise RuntimeError(f"Topic not found or not PointCloud2: {args.topic}")

    msg_cls = get_message(topic_type)

    count = 0
    with open(index_path, "w", encoding="utf-8") as index_file:
        while reader.has_next():
            topic, data, ts_bag = reader.read_next()
            if topic != args.topic:
                continue

            msg = deserialize_message(data, msg_cls)

            timestamp = (
                msg.header.stamp.sec * 1_000_000_000
                + msg.header.stamp.nanosec
            )

            points_iter = pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=args.skip_nans
            )
            xyz = points_to_xyz_array(points_iter)

            filename = f"{args.prefix}_{count:06d}_{timestamp}.pcd"
            out_path = outdir / filename

            save_pcd_binary_open3d(xyz, out_path)

            # ★ インデックス：timestamp(ns) と filename
            index_file.write(f"{int(timestamp)} {filename}\n")

            print(f"[{count:06d}] {xyz.shape[0]} points -> {filename}")
            count += 1
            if args.max_msgs and count >= args.max_msgs:
                break

    print(f"\nIndex written to: {index_path}")
    print("Done.")


if __name__ == "__main__":
    main()
