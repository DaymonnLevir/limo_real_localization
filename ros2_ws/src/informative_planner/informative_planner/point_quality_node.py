#!/usr/bin/env python3
"""Publica /<robot>/point_quality a partir do tree-mapper v2.

Mesma semântica do artigo (tree-mapper ros2-feat/tunning, tree_detector_node):
point_score = número de troncos cujo ajuste geométrico foi aceito NO FRAME DE
NUVEM ATUAL. Na v2 isso é o array por frame do vegetation_candidate_classifier
(/tree_mapper/internal/detections_full), que só marca 'tree' quando passam os
critérios geométricos (circle_fit_score, fatias, desvio de centro/raio).
Não usa o mapa fundido/confirmado (/tree_map_full).

Payload idêntico ao do artigo:
  {"point_score": N, "position_x": x, "position_y": y}   (x, y no PLANNING_FRAME)
"""

import json

import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
import tf2_ros
from tree_mapper.msg import ObjectDetectionArray

from informative_planner import config


class PointQualityNode(Node):

    def __init__(self):
        super().__init__('point_quality_node')
        self.uav_name = str(self.declare_parameter('uav_name', 'limo').value)
        self.detections_topic = str(self.declare_parameter(
            'detections_topic',
            '/tree_mapper/internal/detections_full',
        ).value)
        self.odom_topic = str(
            self.declare_parameter('odom_topic', '/odom').value
        )
        self.planning_frame = str(self.declare_parameter(
            'planning_frame',
            config.PLANNING_FRAME,
        ).value)
        self.point_quality_topic = '/%s/point_quality' % self.uav_name

        self._robot_xy = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(String, self.point_quality_topic, 10)
        self.create_subscription(
            Odometry, self.odom_topic, self._odom_callback, 10
        )
        self.create_subscription(
            ObjectDetectionArray,
            self.detections_topic,
            self._detections_callback,
            10,
        )
        self.get_logger().info(
            f'point_quality: {self.detections_topic} + {self.odom_topic} '
            f'-> {self.point_quality_topic} (frame={self.planning_frame})'
        )

    def _odom_callback(self, msg):
        """Cache robot XY in the planning frame (igual ao _odom_cb do artigo)."""
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ], dtype=float)
        source = msg.header.frame_id
        if not source or source == self.planning_frame:
            self._robot_xy = (float(pos[0]), float(pos[1]))
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self.planning_frame, source, Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'odom TF {source} -> {self.planning_frame} falhou: {exc}',
                throttle_duration_sec=5.0,
            )
            return
        t = transform.transform.translation
        q = transform.transform.rotation
        rot = _quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        xyz = rot @ pos + np.array([t.x, t.y, t.z])
        self._robot_xy = (float(xyz[0]), float(xyz[1]))

    def _detections_callback(self, msg):
        if self._robot_xy is None:
            return
        trees = [
            d for d in msg.detections
            if not d.class_label or d.class_label == 'tree'
        ]
        out = String()
        out.data = json.dumps({
            'point_score': len(trees),
            'position_x': self._robot_xy[0],
            'position_y': self._robot_xy[1],
        })
        self._pub.publish(out)


def _quaternion_to_rotation_matrix(x, y, z, w):
    norm = x * x + y * y + z * z + w * w
    if norm <= 1e-12:
        return np.eye(3)
    s = 2.0 / norm
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ], dtype=float)


def main():
    rclpy.init()
    node = PointQualityNode()
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
