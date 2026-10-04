#!/usr/bin/env python3

import threading

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, CheckButtons
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String

from tree_mapper.msg import ObjectDetectionArray

from tree_mapper_ros2 import TreeMapperNode, as_bool, spin_node_in_background
from tree_mapper_runtime import encode_reset_payload, start_new_run


class TreeXYPlotter(TreeMapperNode):
    """Real-time panel for live detections and the deduplicated final map."""

    LEGEND_ANCHOR = (0.50, 1.01)
    METRICS_ANCHOR = (0.67, 0.11)
    RIGHT_MARGIN = 0.98
    BOTTOM_MARGIN = 0.28

    def __init__(self):
        super().__init__("tree_xy_plotter")

        self.live_array_topic = self.param("live_array_topic", "/tree_mapper/internal/tracked_full")
        self.map_array_topic = self.param("map_array_topic", "/tree_map_full")
        self.map_candidate_array_topic = self.param("map_candidate_array_topic", "/tree_map_candidate_full")
        self.shrub_live_array_topic = self.param("shrub_live_array_topic", "/shrub_mapper/internal/tracked_full")
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

        self.show_live_labels = as_bool(self.param("show_live_labels", True))
        self.show_map_labels = as_bool(self.param("show_map_labels", True))
        self.show_metrics = as_bool(self.param("show_metrics", True))
        self.max_labels = int(self.param("max_labels", 80))
        self.show_route_trace = as_bool(self.param("show_route_trace", True))
        self.route_min_point_dist = float(self.param("route_min_point_dist", 0.15))
        self.route_max_points = int(self.param("route_max_points", 4000))
        self.route_line_width = float(self.param("route_line_width", 1.8))

        self.lock = threading.Lock()
        self.live_detections = []
        self.map_detections = []
        self.map_candidate_detections = []
        self.shrub_live_detections = []
        self.shrub_map_detections = []
        self.shrub_map_candidate_detections = []
        self.route_points = []
        self.live_frame_id = ""
        self.route_frame_id = ""
        self.layer_visibility = {
            "tree": True,
            "candidate tree": True,
            "candidate shrub": True,
        }

        self.live_sub = self.create_subscription(ObjectDetectionArray, self.live_array_topic, self.live_cb, 1)
        self.map_sub = self.create_subscription(ObjectDetectionArray, self.map_array_topic, self.map_cb, 1)
        self.map_candidate_sub = self.create_subscription(
            ObjectDetectionArray, self.map_candidate_array_topic, self.map_candidate_cb, 1
        )
        self.shrub_live_sub = self.create_subscription(
            ObjectDetectionArray, self.shrub_live_array_topic, self.shrub_live_cb, 1
        )
        self.shrub_map_sub = self.create_subscription(ObjectDetectionArray, self.shrub_map_array_topic, self.shrub_map_cb, 1)
        self.shrub_candidate_sub = self.create_subscription(
            ObjectDetectionArray, self.shrub_map_candidate_array_topic, self.shrub_map_candidate_cb, 1
        )
        self.reset_sub = self.create_subscription(String, self.reset_run_topic, self.reset_run_cb, 1)
        self.route_pose_sub = None
        if self.show_route_trace and self.route_pose_topic:
            self.route_pose_sub = self.create_subscription(Odometry, self.route_pose_topic, self.route_cb, 1)
        self.reset_run_pub = self.create_publisher(String, self.reset_run_topic, 1)
        self.log_detections_pub = self.create_publisher(Empty, self.log_detections_topic, 1)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.subplots_adjust(right=self.RIGHT_MARGIN, bottom=self.BOTTOM_MARGIN)
        self.ax.set_title("Tree Mapper - Live / Final Map", pad=52)
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, alpha=0.35)
        self.ax.set_aspect("equal", "box")

        self.live_scatter = self.ax.scatter([], [], c="#2c7fb8", s=54, alpha=0.90, label="live")
        self.map_scatter = self.ax.scatter([], [], c="#1f8f3a", s=88, marker="x", linewidths=1.8, label="confirmed")
        self.candidate_scatter = self.ax.scatter([], [], c="#e67e22", s=64, marker="^", alpha=0.90, label="candidate")
        self.shrub_live_scatter = self.ax.scatter([], [], c="#8c4f16", s=42, alpha=0.78, marker="o", label="shrub live")
        self.shrub_map_scatter = self.ax.scatter([], [], c="#d95f02", s=88, marker="D", alpha=0.92, label="shrub confirmed")
        self.shrub_candidate_scatter = self.ax.scatter(
            [], [], c="#f4a641", s=62, marker="s", alpha=0.90, label="shrub candidate"
        )
        self.route_line, = self.ax.plot([], [], color="#444444", linewidth=self.route_line_width, alpha=0.85, label="route")
        self.route_head_scatter = self.ax.scatter([], [], c="#111111", s=24, marker="o", alpha=0.9, label="drone")
        self.ax.legend(
            loc="lower center",
            bbox_to_anchor=self.LEGEND_ANCHOR,
            borderaxespad=0.0,
            ncol=4,
            fontsize=8,
        )
        self.reset_button_ax = self.fig.add_axes([0.08, 0.10, 0.16, 0.05])
        self.reset_button = Button(self.reset_button_ax, "Reset Run")
        self.reset_button.on_clicked(self.reset_run_clicked)
        self.log_button_ax = self.fig.add_axes([0.08, 0.03, 0.16, 0.05])
        self.log_button = Button(self.log_button_ax, "Log Detections")
        self.log_button.on_clicked(self.log_detections_clicked)
        self.checkbox_ax = self.fig.add_axes([0.30, 0.04, 0.24, 0.14])
        checkbox_labels = ["tree", "candidate tree", "candidate shrub"]
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
            "tree_xy_plotter: trees live=%s confirmed=%s candidates=%s shrubs live=%s confirmed=%s candidates=%s",
            self.live_array_topic,
            self.map_array_topic,
            self.map_candidate_array_topic,
            self.shrub_live_array_topic,
            self.shrub_map_array_topic,
            self.shrub_map_candidate_array_topic,
        )
        if self.show_route_trace and self.route_pose_topic:
            self.loginfo("tree_xy_plotter: route topic=%s", self.route_pose_topic)

        self.anim = FuncAnimation(self.fig, self.update_plot, interval=self.refresh_ms)

    def _clear_local_state(self):
        with self.lock:
            self.live_detections = []
            self.map_detections = []
            self.map_candidate_detections = []
            self.shrub_live_detections = []
            self.shrub_map_detections = []
            self.shrub_map_candidate_detections = []
            self.route_points = []
            self.live_frame_id = ""
            self.route_frame_id = ""

    def reset_run_cb(self, _msg):
        self._clear_local_state()

    def reset_run_clicked(self, _event):
        runtime = start_new_run("tree_mapper", self.requested_run_output_dir)
        self._clear_local_state()
        self.reset_run_pub.publish(String(data=encode_reset_payload(runtime)))
        self.loginfo("tree_xy_plotter: reset run requested run_id=%s dir=%s", runtime["run_id"], runtime["run_dir"])

    def log_detections_clicked(self, _event):
        self.log_detections_pub.publish(Empty())
        self.loginfo("tree_xy_plotter: requested detection snapshot on %s", self.log_detections_topic)

    def toggle_layer(self, label):
        if label in self.layer_visibility:
            self.layer_visibility[label] = not self.layer_visibility[label]

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
    def _to_xy_list(msg):
        points = []
        for det in msg.detections:
            points.append(
                {
                    "id": int(det.id),
                    "x": float(det.pose.position.x),
                    "y": float(det.pose.position.y),
                    "diameter": float(det.diameter),
                    "circle_fit_score": float(det.circle_fit_score),
                    "geometric_score": float(det.geometric_score),
                }
            )
        return points

    @staticmethod
    def _set_offsets(scatter, pts):
        if pts:
            scatter.set_offsets(np.asarray(pts, dtype=float))
        else:
            scatter.set_offsets(np.empty((0, 2), dtype=float))

    def live_cb(self, msg):
        with self.lock:
            self.live_frame_id = msg.header.frame_id if msg.header.frame_id else ""
            self.live_detections = self._to_xy_list(msg)

    def map_cb(self, msg):
        with self.lock:
            self.map_detections = self._to_xy_list(msg)

    def map_candidate_cb(self, msg):
        with self.lock:
            self.map_candidate_detections = self._to_xy_list(msg)

    def shrub_live_cb(self, msg):
        with self.lock:
            self.shrub_live_detections = self._to_xy_list(msg)

    def shrub_map_cb(self, msg):
        with self.lock:
            self.shrub_map_detections = self._to_xy_list(msg)

    def shrub_map_candidate_cb(self, msg):
        with self.lock:
            self.shrub_map_candidate_detections = self._to_xy_list(msg)

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
            live_frame_id = self.live_frame_id
        if route_frame_id and live_frame_id and route_frame_id != live_frame_id:
            self.logwarn_throttle(
                5.0,
                "tree_xy_plotter: route frame (%s) differs from map frame (%s); drone overlay can look displaced",
                route_frame_id,
                live_frame_id,
            )

    def route_cb(self, msg):
        point = (float(msg.pose.pose.position.x), float(msg.pose.pose.position.y))
        route_frame_id = msg.header.frame_id if msg.header.frame_id else ""
        self._append_route_point(point, route_frame_id)

    def _add_labels(self, detections, prefix, color):
        if self.max_labels <= 0:
            return
        for det in detections[: self.max_labels]:
            label = "%s_%d" % (prefix, det["id"])
            text = self.ax.text(det["x"] + 0.10, det["y"] + 0.10, label, fontsize=8.0, color=color)
            self.text_artists.append(text)

    def _update_metrics(
        self,
        live_count,
        confirmed_count,
        candidate_count,
        shrub_live_count,
        shrub_confirmed_count,
        shrub_candidate_count,
    ):
        if self.metrics_artist is None:
            return
        route_length = 0.0
        if len(self.route_points) > 1:
            for first, second in zip(self.route_points[:-1], self.route_points[1:]):
                route_length += float(np.hypot(second[0] - first[0], second[1] - first[1]))
        lines = [
            "tree_live=%d" % live_count,
            "tree_confirmed=%d" % confirmed_count,
            "tree_candidates=%d" % candidate_count,
            "shrub_live=%d" % shrub_live_count,
            "shrub_confirmed=%d" % shrub_confirmed_count,
            "shrub_candidates=%d" % shrub_candidate_count,
        ]
        if self.show_route_trace:
            lines.append("route_points=%d" % len(self.route_points))
            lines.append("route_length=%.1f m" % route_length)
        self.metrics_artist.set_text("\n".join(lines))

    def update_plot(self, _):
        with self.lock:
            live = list(self.live_detections)
            confirmed = list(self.map_detections)
            candidates = list(self.map_candidate_detections)
            shrub_live = list(self.shrub_live_detections)
            shrub_confirmed = list(self.shrub_map_detections)
            shrub_candidates = list(self.shrub_map_candidate_detections)
            route = list(self.route_points)

        live_xy = [(det["x"], det["y"]) for det in live]
        confirmed_xy = [(det["x"], det["y"]) for det in confirmed]
        candidate_xy = [(det["x"], det["y"]) for det in candidates]
        shrub_live_xy = [(det["x"], det["y"]) for det in shrub_live]
        shrub_confirmed_xy = [(det["x"], det["y"]) for det in shrub_confirmed]
        shrub_candidate_xy = [(det["x"], det["y"]) for det in shrub_candidates]

        self._set_offsets(self.live_scatter, live_xy)
        self._set_offsets(self.map_scatter, confirmed_xy)
        self._set_offsets(self.candidate_scatter, candidate_xy)
        self._set_offsets(self.shrub_live_scatter, shrub_live_xy)
        self._set_offsets(self.shrub_map_scatter, shrub_confirmed_xy)
        self._set_offsets(self.shrub_candidate_scatter, shrub_candidate_xy)
        tree_visible = self.layer_visibility["tree"]
        candidate_tree_visible = self.layer_visibility["candidate tree"]
        candidate_shrub_visible = self.layer_visibility["candidate shrub"]
        self.live_scatter.set_visible(tree_visible)
        self.map_scatter.set_visible(tree_visible)
        self.candidate_scatter.set_visible(candidate_tree_visible)
        self.shrub_live_scatter.set_visible(candidate_shrub_visible)
        self.shrub_map_scatter.set_visible(candidate_shrub_visible)
        self.shrub_candidate_scatter.set_visible(candidate_shrub_visible)

        if self.show_route_trace and route:
            self.route_line.set_data([p[0] for p in route], [p[1] for p in route])
            self._set_offsets(self.route_head_scatter, [route[-1]])
        else:
            self.route_line.set_data([], [])
            self._set_offsets(self.route_head_scatter, [])

        for text in self.text_artists:
            text.remove()
        self.text_artists = []

        if self.show_live_labels and tree_visible:
            self._add_labels(live, "live", "#0b3c5d")
        if self.show_live_labels and candidate_shrub_visible:
            self._add_labels(shrub_live, "shrub", "#8c4f16")
        if self.show_map_labels and tree_visible:
            self._add_labels(confirmed, "map", "#145a32")
        if self.show_map_labels and candidate_tree_visible:
            self._add_labels(candidates, "cand", "#9a4d00")
        if self.show_map_labels and candidate_shrub_visible:
            self._add_labels(shrub_confirmed, "shmap", "#9c3f00")
            self._add_labels(shrub_candidates, "shcand", "#b36900")

        self._update_metrics(
            len(live),
            len(confirmed),
            len(candidates),
            len(shrub_live),
            len(shrub_confirmed),
            len(shrub_candidates),
        )

        if not self.fixed_axes:
            all_pts = live_xy + confirmed_xy + candidate_xy + shrub_live_xy + shrub_confirmed_xy + shrub_candidate_xy + route
            if all_pts:
                xs = [point[0] for point in all_pts]
                ys = [point[1] for point in all_pts]
                self.ax.set_xlim(min(xs) - 2.0, max(xs) + 2.0)
                self.ax.set_ylim(min(ys) - 2.0, max(ys) + 2.0)
        else:
            self._configure_axes()


def main(args=None):
    rclpy.init(args=args)
    node = TreeXYPlotter()
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
