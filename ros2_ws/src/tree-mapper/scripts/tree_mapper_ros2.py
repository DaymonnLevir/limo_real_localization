#!/usr/bin/env python3

import math
import os
import time

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from builtin_interfaces.msg import Duration as DurationMsg
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data


def normalize_param_name(name):
    raw = str(name).strip()
    if raw.startswith("~/"):
        raw = raw[2:]
    elif raw.startswith("~"):
        raw = raw[1:]
    return raw.lstrip("/")


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if text in ("1", "true", "yes", "on"):
        return True
    if text in ("0", "false", "no", "off", ""):
        return False
    return bool(value)


def duration_msg(seconds):
    total = max(float(seconds), 0.0)
    sec = int(math.floor(total))
    nanosec = int(round((total - float(sec)) * 1e9))
    if nanosec >= 1000000000:
        sec += 1
        nanosec -= 1000000000
    return DurationMsg(sec=sec, nanosec=nanosec)


def stamp_to_sec(stamp):
    if stamp is None:
        return 0.0
    return float(getattr(stamp, "sec", 0)) + (float(getattr(stamp, "nanosec", 0)) / 1e9)


def latched_qos(depth=1):
    return QoSProfile(depth=max(int(depth), 1), durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def package_share_dir(package_name):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        return os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


class TreeMapperNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self._throttle_state = {}
        self._shutdown_callbacks = []
        self._shutdown_ran = False

    def param(self, name, default):
        param_name = normalize_param_name(name)
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, default)
        return self.get_parameter(param_name).value

    def has_param(self, name):
        param_name = normalize_param_name(name)
        if self.has_parameter(param_name):
            return True
        overrides = getattr(self, "_parameter_overrides", {})
        return param_name in overrides

    def set_param(self, name, value):
        param_name = normalize_param_name(name)
        if not self.has_parameter(param_name):
            self.declare_parameter(param_name, value)
            return
        self.set_parameters([Parameter(param_name, value=value)])

    def now_msg(self):
        return self.get_clock().now().to_msg()

    def now_sec(self):
        return stamp_to_sec(self.now_msg())

    def sleep(self, seconds):
        time.sleep(max(float(seconds), 0.0))

    def on_shutdown(self, callback):
        self._shutdown_callbacks.append(callback)

    def shutdown(self):
        if self._shutdown_ran:
            return
        self._shutdown_ran = True
        for callback in list(self._shutdown_callbacks):
            try:
                callback()
            except Exception as exc:
                self.logwarn("shutdown callback failed: %s", exc)
        self.destroy_node()

    def resolve_name(self, name):
        text = str(name).strip()
        if text.startswith("~/"):
            base = self.get_fully_qualified_name().rstrip("/")
            return "%s/%s" % (base, text[2:])
        return text

    def sensor_qos(self):
        return qos_profile_sensor_data

    def latched_qos(self, depth=1):
        return latched_qos(depth)

    def _format_log(self, message, args):
        if not args:
            return str(message)
        try:
            return str(message) % args
        except Exception:
            parts = [str(message)] + [str(item) for item in args]
            return " ".join(parts)

    def _log_throttle(self, level, period_sec, message, *args):
        key = (level, str(message))
        now = time.monotonic()
        last = self._throttle_state.get(key, None)
        if last is not None and (now - last) < float(period_sec):
            return
        self._throttle_state[key] = now
        text = self._format_log(message, args)
        if level == "info":
            self.get_logger().info(text)
        elif level == "warning":
            self.get_logger().warning(text)
        elif level == "error":
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def loginfo(self, message, *args):
        self.get_logger().info(self._format_log(message, args))

    def logwarn(self, message, *args):
        self.get_logger().warning(self._format_log(message, args))

    def logerror(self, message, *args):
        self.get_logger().error(self._format_log(message, args))

    def loginfo_throttle(self, period_sec, message, *args):
        self._log_throttle("info", period_sec, message, *args)

    def logwarn_throttle(self, period_sec, message, *args):
        self._log_throttle("warning", period_sec, message, *args)


def spin_node(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def spin_node_in_background(node):
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    import threading

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread
