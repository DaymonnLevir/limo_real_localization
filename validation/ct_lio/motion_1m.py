#!/usr/bin/env python3
"""Single bounded forward run; wheel odometry controls stopping, CT-LIO is observed independently."""
import json
import math
import signal
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from limo_msgs.msg import LimoStatus
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


SPEED = 0.045
TARGET_M = 1.0
MAX_MOTION_S = 30.0
OUTPUT = "/validation/motion_1m.json"


def xyz(msg):
    p = msg.pose.pose.position
    return (p.x, p.y, p.z)


def main():
    rclpy.init()
    node = rclpy.create_node("ct_lio_single_forward_guard")
    publisher = node.create_publisher(Twist, "/ct_lio_test/cmd_vel", 10)
    seen = {}
    data = {}
    initial_wheel = None
    initial_ct = None
    first_motion = None
    stop_reason = None
    stop_requested = False
    max_distance = 0.0
    counts = {"wheel": 0, "ct": 0, "lidar": 0, "status": 0}
    invalid = 0

    def interrupt(_signal, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)

    def status(msg):
        seen["status"] = time.monotonic()
        counts["status"] += 1
        data["status"] = {"vehicle_state": msg.vehicle_state, "control_mode": msg.control_mode,
                          "motion_mode": msg.motion_mode, "error_code": msg.error_code}

    def wheel(msg):
        nonlocal initial_wheel, max_distance
        seen["wheel"] = time.monotonic()
        counts["wheel"] += 1
        p = xyz(msg)
        v = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        if not all(map(math.isfinite, p + v)):
            return
        data["wheel_xyz"] = p
        data["wheel_speed"] = v
        if initial_wheel is None and first_motion is not None:
            initial_wheel = p
        if initial_wheel is not None:
            max_distance = max(max_distance, math.dist(initial_wheel[:2], p[:2]))

    def ct(msg):
        nonlocal initial_ct, invalid
        seen["ct"] = time.monotonic()
        counts["ct"] += 1
        p = xyz(msg)
        q = msg.pose.pose.orientation
        orientation = (q.x, q.y, q.z, q.w)
        if not all(map(math.isfinite, p + orientation)):
            invalid += 1
            return
        data["ct_xyz"] = p
        data["ct_xyzw"] = orientation
        if initial_ct is None and first_motion is not None:
            initial_ct = (p, orientation)

    def lidar(msg):
        seen["lidar"] = time.monotonic()
        counts["lidar"] += 1
        if msg.width * msg.height == 0:
            return
        fields = {f.name: f.offset for f in msg.fields}
        if not all(key in fields for key in ("x", "y", "z")):
            return
        count = msg.width * msg.height
        points = np.ndarray((count,), dtype=np.dtype({"names": ["x", "y", "z"],
                          "formats": ["<f4"] * 3,
                          "offsets": [fields["x"], fields["y"], fields["z"]],
                          "itemsize": msg.point_step}), buffer=msg.data)
        x, y, z = points["x"], points["y"], points["z"]
        mask = (x > 0.25) & (x < 1.5) & (np.abs(y) < 0.35) & (z > -0.15) & (z < 0.8)
        if np.count_nonzero(mask) >= 5:
            data["front_clearance_m"] = float(np.min(x[mask]))
        else:
            data["front_clearance_m"] = None

    subscriptions = [
        node.create_subscription(LimoStatus, "/ct_lio_test/status", status, 10),
        node.create_subscription(Odometry, "/ct_lio_test/wheel_odom", wheel, qos_profile_sensor_data),
        node.create_subscription(Odometry, "/ct_lio/odom", ct, qos_profile_sensor_data),
        node.create_subscription(PointCloud2, "/livox/lidar", lidar, qos_profile_sensor_data),
    ]
    start = time.monotonic()
    last_publish = 0.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            if stop_requested:
                stop_reason = "operator_interrupt"
                break
            if first_motion is None and now - start > 10.0:
                stop_reason = "preflight_timeout"
                break
            reason = None
            for key, limit in (("status", 1.0), ("wheel", 0.5), ("ct", 0.5), ("lidar", 0.5)):
                if now - seen.get(key, 0) > limit:
                    reason = "stale_" + key
                    break
            if reason is None:
                state = data["status"]
                if state["vehicle_state"] != 0 or state["control_mode"] != 1 or state["error_code"] != 0 or state["motion_mode"] not in (0, 2):
                    reason = "abnormal_base_state"
                elif "wheel_xyz" not in data or "ct_xyz" not in data:
                    reason = "invalid_pose"
                elif data.get("front_clearance_m") is not None and data["front_clearance_m"] < 0.55:
                    reason = "front_obstacle"
                elif first_motion is not None and (abs(data["wheel_speed"][0]) > 0.12 or abs(data["wheel_speed"][1]) > 0.12):
                    reason = "unexpected_speed_or_turn"
                elif invalid:
                    reason = "invalid_ct_pose"
            if first_motion is None:
                if reason is not None:
                    continue
                first_motion = now
                initial_wheel = data["wheel_xyz"]
                initial_ct = (data["ct_xyz"], data["ct_xyzw"])
                print("Starting one straight forward run", flush=True)
            if reason is not None:
                stop_reason = reason
                break
            if max_distance >= TARGET_M:
                stop_reason = "wheel_distance_reached"
                break
            if now - first_motion >= MAX_MOTION_S:
                stop_reason = "hard_motion_timeout"
                break
            if now - last_publish >= 0.05:
                command = Twist()
                command.linear.x = SPEED
                publisher.publish(command)
                last_publish = now
            if now - start >= 35:
                stop_reason = "outer_timeout"
                break
    finally:
        motion_end = time.monotonic()
        # Keep the zero command live long enough for the base watchdog and serial loop.
        for _ in range(60):
            publisher.publish(Twist())
            rclpy.spin_once(node, timeout_sec=0.03)
        wheel_final = data.get("wheel_xyz")
        ct_final = data.get("ct_xyz")
        ct_delta = [ct_final[i] - initial_ct[0][i] for i in range(3)] if initial_ct and ct_final else None
        result = {
            "commanded_speed_m_s": SPEED, "target_distance_m": TARGET_M,
            "motion_duration_s": motion_end - first_motion if first_motion else 0,
            "stop_reason": stop_reason, "wheel_initial_xyz_m": initial_wheel,
            "wheel_final_xyz_m": wheel_final,
            "wheel_displacement_m": math.dist(initial_wheel[:2], wheel_final[:2]) if initial_wheel and wheel_final else None,
            "wheel_max_displacement_m": max_distance,
            "ct_initial_xyz_m": initial_ct[0] if initial_ct else None,
            "ct_final_xyz_m": ct_final,
            "ct_delta_xyz_m": ct_delta,
            "ct_displacement_m": math.dist((0, 0, 0), ct_delta) if ct_delta else None,
            "ct_initial_xyzw": initial_ct[1] if initial_ct else None,
            "ct_final_xyzw": data.get("ct_xyzw"),
            "invalid_ct_odom_count": invalid, "counts": counts,
            "final_base_state": data.get("status"), "front_clearance_m": data.get("front_clearance_m"),
            "zero_command_sent": True,
        }
        with open(OUTPUT, "w", encoding="utf-8") as output:
            json.dump(result, output, indent=2)
        print(json.dumps(result, indent=2), flush=True)
        subscriptions.clear()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
