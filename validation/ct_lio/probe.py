#!/usr/bin/env python3
"""Bounded, low-overhead physical CT-LIO ROS 2 observer."""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def summarize(times):
    return {"count": len(times), "hz": (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=90)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("ct_lio_phase2_probe")
    topics = ("lidar", "imu", "odom", "path", "map", "tf")
    times = {key: [] for key in topics}
    previous = {}
    regressions = {key: 0 for key in topics}
    invalid = {key: 0 for key in topics}
    positions = []
    orientations = []
    scan = {"points_min": None, "points_max": 0, "timespan_s_min": None, "timespan_s_max": 0.0,
            "timestamp_field": False, "point_time_regressions": 0, "positive_timespan_frames": 0,
            "sampled_frames": 0}
    imu_acc = []
    imu_gyro = []
    tf_frames = set()

    def mark(key, header=None):
        times[key].append(time.monotonic())
        if header is not None:
            value = stamp_ns(header.stamp)
            if key in previous and value < previous[key]:
                regressions[key] += 1
            previous[key] = value

    def lidar(msg):
        mark("lidar", msg.header)
        count = msg.width * msg.height
        scan["points_min"] = count if scan["points_min"] is None else min(scan["points_min"], count)
        scan["points_max"] = max(scan["points_max"], count)
        if count == 0:
            invalid["lidar"] += 1
            return
        field = next((f for f in msg.fields if f.name == "timestamp" and f.datatype == 8), None)
        scan["timestamp_field"] |= field is not None
        if field is None:
            invalid["lidar"] += 1
            return
        # Inspect the first ten real frames; leave the rest lightweight for rate measurement.
        if scan["sampled_frames"] >= 10:
            return
        scan["sampled_frames"] += 1
        values = np.ndarray((count,), dtype=np.dtype({"names": ["timestamp"], "formats": ["<f8"],
                            "offsets": [field.offset], "itemsize": msg.point_step}), buffer=msg.data)["timestamp"]
        if not np.isfinite(values).all():
            invalid["lidar"] += 1
            return
        span = float((values[-1] - values[0]) / 1e9)
        scan["timespan_s_min"] = span if scan["timespan_s_min"] is None else min(scan["timespan_s_min"], span)
        scan["timespan_s_max"] = max(scan["timespan_s_max"], span)
        scan["point_time_regressions"] += int(np.count_nonzero(np.diff(values) < 0))
        scan["positive_timespan_frames"] += int(span > 0)

    def imu(msg):
        mark("imu", msg.header)
        a = msg.linear_acceleration
        g = msg.angular_velocity
        av = (a.x, a.y, a.z)
        gv = (g.x, g.y, g.z)
        if not all(map(math.isfinite, av + gv)):
            invalid["imu"] += 1
        else:
            imu_acc.append(math.sqrt(sum(v * v for v in av)))
            imu_gyro.append(math.sqrt(sum(v * v for v in gv)))

    def odom(msg):
        mark("odom", msg.header)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(map(math.isfinite, values)):
            invalid["odom"] += 1
        else:
            positions.append(values[:3])
            orientations.append(values[3:])

    def path(msg):
        mark("path", msg.header)
        if not msg.poses:
            invalid["path"] += 1

    def map_cloud(msg):
        mark("map", msg.header)
        if msg.width * msg.height == 0:
            invalid["map"] += 1

    def tf(msg):
        mark("tf")
        for item in msg.transforms:
            tf_frames.add((item.header.frame_id, item.child_frame_id))

    subscriptions = [
        node.create_subscription(PointCloud2, "/livox/lidar", lidar, qos_profile_sensor_data),
        node.create_subscription(Imu, "/livox/imu", imu, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/ct_lio/odom", odom, qos_profile_sensor_data),
        node.create_subscription(Path, "/ct_lio/odometry_path", path, qos_profile_sensor_data, raw=False),
        node.create_subscription(PointCloud2, "/ct_lio/map_incremental", map_cloud, qos_profile_sensor_data),
        node.create_subscription(TFMessage, "/tf", tf, qos_profile_sensor_data),
    ]
    start = time.monotonic()
    while time.monotonic() - start < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    duration = time.monotonic() - start
    first = positions[0] if positions else None
    last = positions[-1] if positions else None
    displacements = [math.dist(first, p) for p in positions] if first else []
    result = {
        "duration_s": duration,
        "rates": {key: summarize(value) for key, value in times.items()},
        "timestamp_regressions": regressions,
        "invalid_messages": invalid,
        "lidar": scan,
        "imu": {"acc_norm_mean_g": float(np.mean(imu_acc)) if imu_acc else None,
                "acc_norm_min_g": min(imu_acc) if imu_acc else None,
                "acc_norm_max_g": max(imu_acc) if imu_acc else None,
                "gyro_norm_mean_rad_s": float(np.mean(imu_gyro)) if imu_gyro else None},
        "ct_lio": {"initial_xyz_m": first, "final_xyz_m": last,
                   "final_displacement_m": displacements[-1] if displacements else None,
                   "maximum_displacement_m": max(displacements) if displacements else None,
                   "initial_xyzw": orientations[0] if orientations else None,
                   "final_xyzw": orientations[-1] if orientations else None},
        "tf_frames": sorted(tf_frames),
    }
    with open(args.output, "w", encoding="utf-8") as output:
        json.dump(result, output, indent=2)
    print(json.dumps(result, indent=2), flush=True)
    subscriptions.clear()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
