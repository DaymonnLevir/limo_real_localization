#!/usr/bin/env python3

import csv
import os
import threading

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, CheckButtons
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32MultiArray, String

from tree_mapper.msg import ObjectDetectionArray

from tree_mapper_ros2 import TreeMapperNode, as_bool, spin_node_in_background
from tree_mapper_runtime import encode_reset_payload, start_new_run


class TreeClusterXYPlotter(TreeMapperNode):
    """Real-time panel for current clusters and final map overlay."""

    LEGEND_ANCHOR = (0.50, 1.01)
    METRICS_ANCHOR = (0.67, 0.11)
    RIGHT_MARGIN = 0.98
    BOTTOM_MARGIN = 0.34

    def __init__(self):
        super().__init__("tree_cluster_xy_plotter")

        self.cluster_points_topic = self.param("cluster_points_topic", "/tree_mapper/internal/cluster_points")
        self.cluster_labels_topic = self.param("cluster_labels_topic", "/tree_mapper/internal/cluster_labels")
        self.map_array_topic = self.param("map_array_topic", "/tree_map_full")
        self.map_candidate_array_topic = self.param("map_candidate_array_topic", "/tree_map_candidate_full")
        self.shrub_cluster_points_topic = self.param(
            "shrub_cluster_points_topic", "/shrub_mapper/internal/cluster_points"
        )
        self.shrub_cluster_labels_topic = self.param(
            "shrub_cluster_labels_topic", "/shrub_mapper/internal/cluster_labels"
        )
        self.shrub_map_array_topic = self.param("shrub_map_array_topic", "/shrub_map_full")
        self.shrub_map_candidate_array_topic = self.param(
            "shrub_map_candidate_array_topic", "/shrub_map_candidate_full"
        )
        self.route_pose_topic = self.param("route_pose_topic", "/odom")
        self.requested_run_output_dir = self.param("run_output_dir", "")
        self.reset_run_topic = self.param("reset_run_topic", "/tree_mapper/reset_run_token")
        self.log_detections_topic = self.param("log_detections_topic", "/tree_mapper/log_detections_now")

        self.refresh_ms = int(self.param("refresh_ms", 500))
        self.fixed_axes = as_bool(self.param("fixed_axes", False))
        self.x_min = float(self.param("x_min", -10.0))
        self.x_max = float(self.param("x_max", 10.0))
        self.y_min = float(self.param("y_min", -8.0))
        self.y_max = float(self.param("y_max", 7.0))
        self.axis_margin = float(self.param("axis_margin", 5.0))
        self.use_boundary_bounds = as_bool(self.param("use_boundary_bounds", False))
        self.tick_step = float(self.param("tick_step", 5.0))

        self.cluster_point_size = float(self.param("cluster_point_size", 12.0))
        self.cluster_alpha = float(self.param("cluster_alpha", 0.62))
        self.show_map_labels = as_bool(self.param("show_map_labels", True))
        self.show_metrics = as_bool(self.param("show_metrics", True))
        self.max_labels = int(self.param("max_labels", 90))
        self.show_route_trace = as_bool(self.param("show_route_trace", True))
        self.route_min_point_dist = float(self.param("route_min_point_dist", 0.15))
        self.route_max_points = int(self.param("route_max_points", 4000))
        self.route_line_width = float(self.param("route_line_width", 1.8))

        ground_truth_csv_path = str(self.param("ground_truth_csv_path", "") or "").strip()
        self.ground_truth_csv_path = (
            os.path.abspath(os.path.expanduser(ground_truth_csv_path)) if ground_truth_csv_path else ""
        )
        self.ground_truth_path_missing_warned = False

        self.lock = threading.Lock()
        self.cluster_points = []
        self.cluster_labels = []
        self.shrub_cluster_points = []
        self.shrub_cluster_labels = []
        self.map_detections = []
        self.map_candidate_detections = []
        self.shrub_map_detections = []
        self.shrub_map_candidate_detections = []
        self.route_points = []
        self.data_frame_id = ""
        self.route_frame_id = ""
        self.layer_visibility = {
            "tree": True,
            "ground truth": bool(self.ground_truth_csv_path and os.path.exists(self.ground_truth_csv_path)),
            "cluster": True,
            "candidate shrub": True,
            "candidate tree": True,
        }
        self.ground_truth = self._load_ground_truth()

        self.palette = plt.cm.get_cmap("tab20", 20)

        self.cluster_points_sub = self.create_subscription(PoseArray, self.cluster_points_topic, self.cluster_points_cb, 1)
        self.cluster_labels_sub = self.create_subscription(
            Int32MultiArray, self.cluster_labels_topic, self.cluster_labels_cb, 1
        )
        self.map_sub = self.create_subscription(ObjectDetectionArray, self.map_array_topic, self.map_array_cb, 1)
        self.map_candidate_sub = self.create_subscription(
            ObjectDetectionArray, self.map_candidate_array_topic, self.map_candidate_array_cb, 1
        )
        self.shrub_cluster_points_sub = self.create_subscription(
            PoseArray, self.shrub_cluster_points_topic, self.shrub_cluster_points_cb, 1
        )
        self.shrub_cluster_labels_sub = self.create_subscription(
            Int32MultiArray, self.shrub_cluster_labels_topic, self.shrub_cluster_labels_cb, 1
        )
        self.shrub_map_sub = self.create_subscription(
            ObjectDetectionArray, self.shrub_map_array_topic, self.shrub_map_array_cb, 1
        )
        self.shrub_candidate_sub = self.create_subscription(
            ObjectDetectionArray, self.shrub_map_candidate_array_topic, self.shrub_map_candidate_array_cb, 1
        )
        self.reset_sub = self.create_subscription(String, self.reset_run_topic, self.reset_run_cb, 1)
        self.route_pose_sub = None
        if self.show_route_trace and self.route_pose_topic:
            self.route_pose_sub = self.create_subscription(Odometry, self.route_pose_topic, self.route_cb, 1)
        self.reset_run_pub = self.create_publisher(String, self.reset_run_topic, 1)
        self.log_detections_pub = self.create_publisher(Empty, self.log_detections_topic, 1)

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.fig.subplots_adjust(right=self.RIGHT_MARGIN, bottom=self.BOTTOM_MARGIN)
        self.ax.set_title("Tree Mapper - Cluster Panel", pad=52)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, alpha=0.35)
        self.ax.set_aspect("equal", "box")

        self.cluster_scatter = self.ax.scatter([], [], s=self.cluster_point_size, alpha=self.cluster_alpha, linewidths=0, label="cluster")
        self.map_scatter = self.ax.scatter([], [], c="#1f8f3a", s=84, marker="x", linewidths=1.8, label="confirmed")
        self.candidate_scatter = self.ax.scatter([], [], c="#e67e22", s=64, marker="^", alpha=0.90, label="candidate")
        self.shrub_cluster_scatter = self.ax.scatter(
            [], [], s=self.cluster_point_size, alpha=self.cluster_alpha, linewidths=0, marker="s", label="shrub cluster"
        )
        self.shrub_map_scatter = self.ax.scatter([], [], c="#d95f02", s=78, marker="D", alpha=0.95, label="shrub confirmed")
        self.shrub_candidate_scatter = self.ax.scatter(
            [], [], c="#f0ad4e", s=62, marker="P", alpha=0.90, label="shrub candidate"
        )
        self.route_line, = self.ax.plot([], [], color="#444444", linewidth=self.route_line_width, alpha=0.85, label="route")
        self.route_head_scatter = self.ax.scatter([], [], c="#111111", s=24, marker="o", alpha=0.9, label="drone")
        self.gt_tree_scatter = self.ax.scatter(
            [], [], s=44, marker="s", facecolors="none", edgecolors="#2e8b57", linewidths=1.5, alpha=0.90, label="gt tree"
        )
        self.gt_bush_scatter = self.ax.scatter(
            [], [], s=44, marker="D", facecolors="none", edgecolors="#b8860b", linewidths=1.5, alpha=0.90, label="gt bush"
        )
        self.ax.legend(
            loc="lower center",
            bbox_to_anchor=self.LEGEND_ANCHOR,
            borderaxespad=0.0,
            ncol=5,
            fontsize=8,
        )
        self.reset_button_ax = self.fig.add_axes([0.07, 0.10, 0.16, 0.05])
        self.reset_button = Button(self.reset_button_ax, "Reset Run")
        self.reset_button.on_clicked(self.reset_run_clicked)
        self.log_button_ax = self.fig.add_axes([0.07, 0.03, 0.16, 0.05])
        self.log_button = Button(self.log_button_ax, "Log Detections")
        self.log_button.on_clicked(self.log_detections_clicked)
        self.checkbox_ax = self.fig.add_axes([0.30, 0.03, 0.30, 0.18])
        checkbox_labels = ["tree", "ground truth", "cluster", "candidate shrub", "candidate tree"]
        checkbox_states = [self.layer_visibility[label] for label in checkbox_labels]
        self.layer_checkboxes = CheckButtons(self.checkbox_ax, checkbox_labels, checkbox_states)
        self.layer_checkboxes.on_clicked(self.toggle_layer)
        for label in self.layer_checkboxes.labels:
            label.set_fontsize(8)

        self.text_artists = []
        self.metrics_artist = None
        if self.show_metrics:
            self.metrics_artist = self.ax.text(
                self.METRICS_ANCHOR[0],
                self.METRICS_ANCHOR[1],
                "",
                transform=self.fig.transFigure,
                ha="left",
                va="center",
                fontsize=9,
                bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.68),
                clip_on=False,
            )

        self._apply_boundary_bounds_if_available()
        self._configure_axes()

        self.loginfo(
            "tree_cluster_xy_plotter: tree clusters=%s map=%s candidates=%s shrub clusters=%s map=%s candidates=%s",
            self.cluster_points_topic,
            self.map_array_topic,
            self.map_candidate_array_topic,
            self.shrub_cluster_points_topic,
            self.shrub_map_array_topic,
            self.shrub_map_candidate_array_topic,
        )
        if self.show_route_trace and self.route_pose_topic:
            self.loginfo("tree_cluster_xy_plotter: route topic=%s", self.route_pose_topic)

        self.anim = FuncAnimation(self.fig, self.update_plot, interval=self.refresh_ms)

    def _clear_local_state(self):
        with self.lock:
            self.cluster_points = []
            self.cluster_labels = []
            self.shrub_cluster_points = []
            self.shrub_cluster_labels = []
            self.map_detections = []
            self.map_candidate_detections = []
            self.shrub_map_detections = []
            self.shrub_map_candidate_detections = []
            self.route_points = []
            self.data_frame_id = ""
            self.route_frame_id = ""

    def reset_run_cb(self, _msg):
        self._clear_local_state()

    def reset_run_clicked(self, _event):
        runtime = start_new_run("tree_mapper", self.requested_run_output_dir)
        self._clear_local_state()
        self.reset_run_pub.publish(String(data=encode_reset_payload(runtime)))
        self.loginfo(
            "tree_cluster_xy_plotter: reset run requested run_id=%s dir=%s",
            runtime["run_id"],
            runtime["run_dir"],
        )

    def log_detections_clicked(self, _event):
        self.log_detections_pub.publish(Empty())
        self.loginfo(
            "tree_cluster_xy_plotter: requested detection snapshot on %s",
            self.log_detections_topic,
        )

    def toggle_layer(self, label):
        if label in self.layer_visibility:
            self.layer_visibility[label] = not self.layer_visibility[label]

    def _load_ground_truth(self):
        if not self.ground_truth_csv_path:
            return []
        if not os.path.exists(self.ground_truth_csv_path):
            if not self.ground_truth_path_missing_warned:
                self.logwarn("tree_cluster_xy_plotter: ground truth CSV not found: %s", self.ground_truth_csv_path)
                self.ground_truth_path_missing_warned = True
            return []
        self.ground_truth_path_missing_warned = False
        entries = []
        try:
            with open(self.ground_truth_csv_path, "r") as handle:
                reader = csv.DictReader(handle)
                for row in reader:
                    label = str(row.get("type", "tree")).strip().lower()
                    entries.append(
                        {
                            "type": "bush" if "bush" in label else "tree",
                            "x": float(row.get("x", 0.0)),
                            "y": float(row.get("y", 0.0)),
                        }
                    )
        except Exception as exc:
            self.logwarn("tree_cluster_xy_plotter: failed reading ground truth %s: %s", self.ground_truth_csv_path, exc)
            return []
        return entries

    def _apply_boundary_bounds_if_available(self):
        if not self.fixed_axes or not self.use_boundary_bounds:
            return
        try:
            bmin = self.param("boundary.min", [self.x_min, self.y_min])
            bmax = self.param("boundary.max", [self.x_max, self.y_max])
            self.x_min = float(bmin[0]) - self.axis_margin
            self.y_min = float(bmin[1]) - self.axis_margin
            self.x_max = float(bmax[0]) + self.axis_margin
            self.y_max = float(bmax[1]) + self.axis_margin
        except Exception:
            return

    def _configure_axes(self):
        if not self.fixed_axes:
            return
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        if self.tick_step > 0.0:
            self.ax.set_xticks(np.arange(self.x_min, self.x_max + self.tick_step, self.tick_step))
            self.ax.set_yticks(np.arange(self.y_min, self.y_max + self.tick_step, self.tick_step))

    @staticmethod
    def _set_offsets(scatter, pts):
        if pts:
            scatter.set_offsets(np.asarray(pts, dtype=float))
        else:
            scatter.set_offsets(np.empty((0, 2), dtype=float))

    @staticmethod
    def _array_to_detections(msg):
        detections = []
        for det in msg.detections:
            detections.append(
                {
                    "id": int(det.id),
                    "x": float(det.pose.position.x),
                    "y": float(det.pose.position.y),
                    "diameter": float(det.diameter),
                    "circle_fit_score": float(det.circle_fit_score),
                    "geometric_score": float(det.geometric_score),
                    "cluster_label": int(det.cluster_label),
                }
            )
        return detections

    def cluster_points_cb(self, msg):
        with self.lock:
            if msg.header.frame_id:
                self.data_frame_id = msg.header.frame_id
            self.cluster_points = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def cluster_labels_cb(self, msg):
        with self.lock:
            self.cluster_labels = list(msg.data)

    def shrub_cluster_points_cb(self, msg):
        with self.lock:
            if msg.header.frame_id:
                self.data_frame_id = msg.header.frame_id
            self.shrub_cluster_points = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def shrub_cluster_labels_cb(self, msg):
        with self.lock:
            self.shrub_cluster_labels = list(msg.data)

    def map_array_cb(self, msg):
        with self.lock:
            if msg.header.frame_id:
                self.data_frame_id = msg.header.frame_id
            self.map_detections = self._array_to_detections(msg)

    def map_candidate_array_cb(self, msg):
        with self.lock:
            if msg.header.frame_id:
                self.data_frame_id = msg.header.frame_id
            self.map_candidate_detections = self._array_to_detections(msg)

    def shrub_map_array_cb(self, msg):
        with self.lock:
            if msg.header.frame_id:
                self.data_frame_id = msg.header.frame_id
            self.shrub_map_detections = self._array_to_detections(msg)

    def shrub_map_candidate_array_cb(self, msg):
        with self.lock:
            if msg.header.frame_id:
                self.data_frame_id = msg.header.frame_id
            self.shrub_map_candidate_detections = self._array_to_detections(msg)

    def _append_route_point(self, point, route_frame_id):
        with self.lock:
            self.route_frame_id = route_frame_id
            if self.route_points:
                prev = self.route_points[-1]
                if np.hypot(point[0] - prev[0], point[1] - prev[1]) < self.route_min_point_dist:
                    return
            self.route_points.append(point)
            if self.route_max_points > 0 and len(self.route_points) > self.route_max_points:
                self.route_points = self.route_points[-self.route_max_points :]
            data_frame_id = self.data_frame_id
        if route_frame_id and data_frame_id and route_frame_id != data_frame_id:
            self.logwarn_throttle(
                5.0,
                "tree_cluster_xy_plotter: route frame (%s) differs from cluster/map frame (%s); drone overlay can look displaced",
                route_frame_id,
                data_frame_id,
            )

    def route_cb(self, msg):
        point = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        route_frame_id = msg.header.frame_id if msg.header.frame_id else ""
        self._append_route_point(point, route_frame_id)

    def _cluster_colors(self, labels, count):
        aligned = [-1] * count
        for index in range(min(count, len(labels))):
            aligned[index] = int(labels[index])
        colors = []
        for label in aligned:
            if label < 0:
                colors.append([0.25, 0.25, 0.25, self.cluster_alpha * 0.65])
            else:
                rgba = list(self.palette(label % 20))
                rgba[3] = self.cluster_alpha
                colors.append(rgba)
        return np.asarray(colors, dtype=float)

    def _add_labels(self, detections, prefix, color):
        if self.max_labels <= 0:
            return
        for det in detections[: self.max_labels]:
            label = "%s_%d" % (prefix, det["id"])
            text = self.ax.text(det["x"] + 0.10, det["y"] + 0.10, label, fontsize=8.0, color=color)
            self.text_artists.append(text)

    def _update_metrics(
        self,
        cluster_count,
        cluster_points_count,
        confirmed_count,
        candidate_count,
        shrub_cluster_count,
        shrub_cluster_points_count,
        shrub_confirmed_count,
        shrub_candidate_count,
    ):
        if self.metrics_artist is None:
            return
        lines = [
            "tree_cluster_points=%d" % cluster_points_count,
            "tree_clusters=%d" % cluster_count,
            "tree_confirmed=%d" % confirmed_count,
            "tree_candidates=%d" % candidate_count,
            "shrub_cluster_points=%d" % shrub_cluster_points_count,
            "shrub_clusters=%d" % shrub_cluster_count,
            "shrub_confirmed=%d" % shrub_confirmed_count,
            "shrub_candidates=%d" % shrub_candidate_count,
        ]
        self.metrics_artist.set_text("\n".join(lines))

    def update_plot(self, _):
        with self.lock:
            cluster_points = list(self.cluster_points)
            cluster_labels = list(self.cluster_labels)
            shrub_cluster_points = list(self.shrub_cluster_points)
            shrub_cluster_labels = list(self.shrub_cluster_labels)
            confirmed = list(self.map_detections)
            candidates = list(self.map_candidate_detections)
            shrub_confirmed = list(self.shrub_map_detections)
            shrub_candidates = list(self.shrub_map_candidate_detections)
            route = list(self.route_points)
            ground_truth = list(self.ground_truth)

        cluster_xy = list(cluster_points)
        shrub_cluster_xy = list(shrub_cluster_points)
        confirmed_xy = [(det["x"], det["y"]) for det in confirmed]
        candidate_xy = [(det["x"], det["y"]) for det in candidates]
        shrub_confirmed_xy = [(det["x"], det["y"]) for det in shrub_confirmed]
        shrub_candidate_xy = [(det["x"], det["y"]) for det in shrub_candidates]
        ground_truth_available = bool(self.ground_truth_csv_path and os.path.exists(self.ground_truth_csv_path))
        gt_tree_xy = [(entry["x"], entry["y"]) for entry in ground_truth if entry["type"] == "tree"] if ground_truth_available else []
        gt_bush_xy = [(entry["x"], entry["y"]) for entry in ground_truth if entry["type"] == "bush"] if ground_truth_available else []
        tree_visible = self.layer_visibility["tree"]
        ground_truth_visible = self.layer_visibility["ground truth"] and ground_truth_available
        cluster_visible = self.layer_visibility["cluster"]
        candidate_shrub_visible = self.layer_visibility["candidate shrub"]
        candidate_tree_visible = self.layer_visibility["candidate tree"]

        self._set_offsets(self.cluster_scatter, cluster_xy)
        if cluster_xy:
            self.cluster_scatter.set_facecolors(self._cluster_colors(cluster_labels, len(cluster_xy)))
        else:
            self.cluster_scatter.set_facecolors(np.empty((0, 4), dtype=float))
        self._set_offsets(self.shrub_cluster_scatter, shrub_cluster_xy)
        if shrub_cluster_xy:
            shrub_colors = self._cluster_colors(shrub_cluster_labels, len(shrub_cluster_xy))
            if len(shrub_colors) > 0:
                shrub_colors[:, 0] = np.clip(shrub_colors[:, 0] + 0.20, 0.0, 1.0)
                shrub_colors[:, 1] = np.clip(shrub_colors[:, 1] * 0.7, 0.0, 1.0)
            self.shrub_cluster_scatter.set_facecolors(shrub_colors)
        else:
            self.shrub_cluster_scatter.set_facecolors(np.empty((0, 4), dtype=float))

        self._set_offsets(self.map_scatter, confirmed_xy)
        self._set_offsets(self.candidate_scatter, candidate_xy)
        self._set_offsets(self.shrub_map_scatter, shrub_confirmed_xy)
        self._set_offsets(self.shrub_candidate_scatter, shrub_candidate_xy)
        self._set_offsets(self.gt_tree_scatter, gt_tree_xy)
        self._set_offsets(self.gt_bush_scatter, gt_bush_xy)
        self.cluster_scatter.set_visible(cluster_visible)
        self.shrub_cluster_scatter.set_visible(cluster_visible)
        self.map_scatter.set_visible(tree_visible)
        self.candidate_scatter.set_visible(candidate_tree_visible)
        self.shrub_map_scatter.set_visible(candidate_shrub_visible)
        self.shrub_candidate_scatter.set_visible(candidate_shrub_visible)
        self.gt_tree_scatter.set_visible(ground_truth_visible)
        self.gt_bush_scatter.set_visible(ground_truth_visible)

        if self.show_route_trace and route:
            self.route_line.set_data([point[0] for point in route], [point[1] for point in route])
            self._set_offsets(self.route_head_scatter, [route[-1]])
        else:
            self.route_line.set_data([], [])
            self._set_offsets(self.route_head_scatter, [])

        for text in self.text_artists:
            text.remove()
        self.text_artists = []

        if self.show_map_labels:
            if tree_visible:
                self._add_labels(confirmed, "map", "#145a32")
            if candidate_tree_visible:
                self._add_labels(candidates, "cand", "#9a4d00")
            if candidate_shrub_visible:
                self._add_labels(shrub_confirmed, "shmap", "#a34d00")
                self._add_labels(shrub_candidates, "shcand", "#b87d12")
        if ground_truth_visible:
            for point in gt_tree_xy[: self.max_labels]:
                self.text_artists.append(self.ax.text(point[0] + 0.10, point[1] + 0.10, "tree", fontsize=7.8, color="#1f5d3a"))
            for point in gt_bush_xy[: self.max_labels]:
                self.text_artists.append(self.ax.text(point[0] + 0.10, point[1] + 0.10, "bush", fontsize=7.8, color="#8a6a00"))

        cluster_count = len(set([label for label in cluster_labels if label >= 0]))
        shrub_cluster_count = len(set([label for label in shrub_cluster_labels if label >= 0]))
        self._update_metrics(
            cluster_count,
            len(cluster_xy),
            len(confirmed_xy),
            len(candidate_xy),
            shrub_cluster_count,
            len(shrub_cluster_xy),
            len(shrub_confirmed_xy),
            len(shrub_candidate_xy),
        )

        if not self.fixed_axes:
            all_pts = (
                cluster_xy
                + shrub_cluster_xy
                + confirmed_xy
                + candidate_xy
                + shrub_confirmed_xy
                + shrub_candidate_xy
                + route
                + gt_tree_xy
                + gt_bush_xy
            )
            if all_pts:
                xs = [point[0] for point in all_pts]
                ys = [point[1] for point in all_pts]
                self.ax.set_xlim(min(xs) - 2.0, max(xs) + 2.0)
                self.ax.set_ylim(min(ys) - 2.0, max(ys) + 2.0)
        else:
            self._configure_axes()


def main(args=None):
    rclpy.init(args=args)
    node = TreeClusterXYPlotter()
    executor, thread = spin_node_in_background(node)
    try:
        plt.show()
    finally:
        executor.shutdown()
        thread.join(timeout=1.0)
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
