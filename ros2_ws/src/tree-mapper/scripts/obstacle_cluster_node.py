#!/usr/bin/env python3

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from tree_mapper_ros2 import TreeMapperNode, spin_node

try:
    from sklearn.cluster import DBSCAN

    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False


class ObstacleClusterNode(TreeMapperNode):
    """Cluster the non-ground cloud with DBSCAN and republish it colored by cluster id."""

    def __init__(self):
        super().__init__("obstacle_cluster_node")

        self.input_topic = self.param("input_topic", "/ground_segmentation/obstacle_points")
        self.output_topic = self.param("output_topic", "/obstacle_clusters")
        self.dbscan_eps = float(self.param("dbscan_eps", 0.5))
        self.dbscan_min_samples = int(self.param("dbscan_min_samples", 8))
        self.max_points = int(self.param("max_points", 60000))

        if not SKLEARN_AVAILABLE:
            raise RuntimeError("obstacle_cluster_node requires scikit-learn")

        self.cluster_pub = self.create_publisher(PointCloud2, self.output_topic, 1)
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, self.sensor_qos())

        self.loginfo(
            "obstacle_cluster_node: in=%s out=%s eps=%.2f min_samples=%d",
            self.input_topic,
            self.output_topic,
            self.dbscan_eps,
            self.dbscan_min_samples,
        )

    @staticmethod
    def _read_xyz_points(msg):
        raw = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        array = np.asarray(raw)
        if array.size == 0:
            return np.empty((0, 3), dtype=float)
        if array.dtype.names:
            points = np.column_stack(
                (
                    array["x"].astype(float, copy=False),
                    array["y"].astype(float, copy=False),
                    array["z"].astype(float, copy=False),
                )
            )
        else:
            points = array.astype(float, copy=False)
        return points[np.all(np.isfinite(points), axis=1)]

    def _publish_clusters(self, header, points, labels):
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="cluster", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out_header = Header()
        out_header.stamp = header.stamp
        out_header.frame_id = header.frame_id
        rows = np.column_stack((points, labels.astype(float))).astype(np.float32)
        self.cluster_pub.publish(pc2.create_cloud(out_header, fields, rows.tolist()))

    def cloud_callback(self, msg):
        try:
            points = self._read_xyz_points(msg)
        except Exception as exc:
            self.logwarn_throttle(2.0, "obstacle_cluster_node point read failed: %s", exc)
            return
        if points.shape[0] < self.dbscan_min_samples:
            self._publish_clusters(msg.header, np.empty((0, 3)), np.empty(0))
            return
        if points.shape[0] > self.max_points:
            sample_idx = np.random.choice(points.shape[0], self.max_points, replace=False)
            points = points[sample_idx]

        labels = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points).labels_
        keep = labels >= 0
        self._publish_clusters(msg.header, points[keep], labels[keep])
        self.loginfo_throttle(
            2.0,
            "obstacle_cluster_node: points=%d clustered=%d clusters=%d",
            points.shape[0],
            int(np.count_nonzero(keep)),
            int(labels.max()) + 1 if labels.size and labels.max() >= 0 else 0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleClusterNode()
    spin_node(node)


if __name__ == "__main__":
    main()
