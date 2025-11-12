#!/usr/bin/env python3
"""ROS 2 to Rerun bridge implemented in Python."""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence

import rerun as rr
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rosidl_runtime_py.utilities import get_message

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray, String


@dataclass(frozen=True)
class TopicConfig:
    topic: str
    type_string: str
    qos: Optional[str] = None


@dataclass
class FlattenResult:
    numeric: Dict[str, float]
    texts: List[str]


def normalize_unit(value: Optional[str]) -> str:
    if value is None:
        return "P13"
    cleaned = value.strip()
    if not cleaned:
        return "P13"
    while cleaned.startswith("/"):
        cleaned = cleaned[1:]
    return cleaned or "P13"


def entity_path(unit: str, topic: str, category: str) -> str:
    prefix = f"/{unit}/"
    if topic.startswith(prefix):
        relative = topic[len(prefix) :].lstrip("/")
        return f"{unit}/{category}/{relative}"
    stripped = topic.lstrip("/")
    return f"global/{category}/{stripped}"


def time_to_seconds(time_msg: Time) -> float:
    return float(time_msg.sec) + float(time_msg.nanosec) * 1e-9


class RerunBridge(Node):
    def __init__(self, unit_name: str) -> None:
        super().__init__("p13_rerun_bridge")
        self._unit_name = unit_name
        self._handlers = self._initialize_handlers()
        self._subscriptions: List[Any] = []

        for config in build_topic_list(unit_name):
            try:
                self._create_subscription(config)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(
                    "Failed to subscribe %s (%s): %s",
                    config.topic,
                    config.type_string,
                    exc,
                )

    # ------------------------------------------------------------------
    # Subscription management
    # ------------------------------------------------------------------
    def _create_subscription(self, config: TopicConfig) -> None:
        msg_type = get_message(config.type_string)
        qos_profile = (
            QoSPresetProfiles.SENSOR_DATA.value
            if config.qos == "sensor"
            else QoSProfile(depth=10)
        )

        def callback(msg: Any, *, cfg: TopicConfig = config) -> None:
            self._handle_message(cfg, msg)

        subscription = self.create_subscription(msg_type, config.topic, callback, qos_profile)
        self._subscriptions.append(subscription)
        self.get_logger().info("Subscribed to %s (%s)", config.topic, config.type_string)

    # ------------------------------------------------------------------
    # Message handling
    # ------------------------------------------------------------------
    def _handle_message(self, config: TopicConfig, msg: Any) -> None:
        stamp = self._extract_header_stamp(msg)
        if stamp is not None:
            rr.set_time_seconds("ros_time", stamp)

        handler = self._handlers.get(config.type_string)
        if handler is not None:
            handler(config.topic, msg)
        else:
            self._handle_as_scalars(config.topic, msg)

    def _extract_header_stamp(self, msg: Any) -> Optional[float]:
        header = getattr(msg, "header", None)
        if header is None:
            return None
        stamp = getattr(header, "stamp", None)
        if isinstance(stamp, Time):
            return time_to_seconds(stamp)
        sec = getattr(stamp, "sec", None)
        nanosec = getattr(stamp, "nanosec", None)
        if isinstance(sec, (int, float)) and isinstance(nanosec, (int, float)):
            return float(sec) + float(nanosec) * 1e-9
        return None

    def _initialize_handlers(self) -> Dict[str, Any]:
        return {
            "geometry_msgs/msg/Twist": self._handle_twist,
            "geometry_msgs/msg/Pose": self._handle_pose_message,
            "geometry_msgs/msg/PoseStamped": self._handle_pose_stamped,
            "geometry_msgs/msg/PointStamped": self._handle_point_stamped,
            "nav_msgs/msg/Odometry": self._handle_odometry,
            "nav_msgs/msg/Path": self._handle_path,
            "std_msgs/msg/String": self._handle_string,
            "std_msgs/msg/Float64MultiArray": self._handle_float64_multi_array,
            "sensor_msgs/msg/Imu": self._handle_imu,
            "sensor_msgs/msg/PointCloud2": self._handle_point_cloud,
        }

    # ------------------------------------------------------------------
    # Specialized handlers
    # ------------------------------------------------------------------
    def _handle_twist(self, topic: str, msg: Twist) -> None:
        base_path = entity_path(self._unit_name, topic, "graphs")
        rr.log(f"{base_path}/linear/x", rr.Scalar(float(msg.linear.x)))
        rr.log(f"{base_path}/linear/y", rr.Scalar(float(msg.linear.y)))
        rr.log(f"{base_path}/linear/z", rr.Scalar(float(msg.linear.z)))
        rr.log(f"{base_path}/angular/x", rr.Scalar(float(msg.angular.x)))
        rr.log(f"{base_path}/angular/y", rr.Scalar(float(msg.angular.y)))
        rr.log(f"{base_path}/angular/z", rr.Scalar(float(msg.angular.z)))

    def _handle_pose(self, topic: str, translation: Point, rotation: Quaternion) -> None:
        rr.log(
            entity_path(self._unit_name, topic, "pose"),
            rr.Transform3D(
                translation=[float(translation.x), float(translation.y), float(translation.z)],
                rotation=rr.Quaternion(xyzw=[float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)]),
            ),
        )

    def _handle_pose_message(self, topic: str, msg: Pose) -> None:
        self._handle_pose(topic, msg.position, msg.orientation)

    def _handle_pose_stamped(self, topic: str, msg: PoseStamped) -> None:
        self._handle_pose_message(topic, msg.pose)

    def _handle_point_stamped(self, topic: str, msg: PointStamped) -> None:
        rr.log(
            entity_path(self._unit_name, topic, "sensors"),
            rr.Points3D(positions=[[float(msg.point.x), float(msg.point.y), float(msg.point.z)]]),
        )

    def _handle_odometry(self, topic: str, msg: Odometry) -> None:
        self._handle_pose(topic, msg.pose.pose.position, msg.pose.pose.orientation)
        base_path = entity_path(self._unit_name, topic, "graphs")
        rr.log(f"{base_path}/twist/linear/x", rr.Scalar(float(msg.twist.twist.linear.x)))
        rr.log(f"{base_path}/twist/linear/y", rr.Scalar(float(msg.twist.twist.linear.y)))
        rr.log(f"{base_path}/twist/linear/z", rr.Scalar(float(msg.twist.twist.linear.z)))
        rr.log(f"{base_path}/twist/angular/x", rr.Scalar(float(msg.twist.twist.angular.x)))
        rr.log(f"{base_path}/twist/angular/y", rr.Scalar(float(msg.twist.twist.angular.y)))
        rr.log(f"{base_path}/twist/angular/z", rr.Scalar(float(msg.twist.twist.angular.z)))

    def _handle_path(self, topic: str, msg: Path) -> None:
        positions = [
            [float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z)]
            for p in msg.poses
        ]
        if positions:
            rr.log(
                entity_path(self._unit_name, topic, "paths"),
                rr.LineStrips3D([positions]),
            )

    def _handle_string(self, topic: str, msg: String) -> None:
        rr.log(entity_path(self._unit_name, topic, "logs"), rr.TextLog(msg.data))

    def _handle_float64_multi_array(self, topic: str, msg: Float64MultiArray) -> None:
        base_path = entity_path(self._unit_name, topic, "graphs")
        for idx, value in enumerate(msg.data):
            rr.log(f"{base_path}/{idx}", rr.Scalar(float(value)))

    def _handle_imu(self, topic: str, msg: Imu) -> None:
        base_path = entity_path(self._unit_name, topic, "sensors")
        rr.log(
            f"{base_path}/orientation",
            rr.Transform3D(
                translation=[0.0, 0.0, 0.0],
                rotation=rr.Quaternion(
                    xyzw=[
                        float(msg.orientation.x),
                        float(msg.orientation.y),
                        float(msg.orientation.z),
                        float(msg.orientation.w),
                    ]
                ),
            ),
        )
        origin = [[0.0, 0.0, 0.0]]
        rr.log(
            f"{base_path}/angular_velocity",
            rr.Arrows3D(origins=origin, vectors=[[float(msg.angular_velocity.x), float(msg.angular_velocity.y), float(msg.angular_velocity.z)]]),
        )
        rr.log(
            f"{base_path}/linear_acceleration",
            rr.Arrows3D(origins=origin, vectors=[[float(msg.linear_acceleration.x), float(msg.linear_acceleration.y), float(msg.linear_acceleration.z)]]),
        )

    def _handle_point_cloud(self, topic: str, msg: PointCloud2) -> None:
        points_iter = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = [
            [float(x), float(y), float(z)]
            for x, y, z in points_iter
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
        ]
        if points:
            rr.log(entity_path(self._unit_name, topic, "sensors"), rr.Points3D(positions=points))

    # ------------------------------------------------------------------
    # Generic handler fallback
    # ------------------------------------------------------------------
    def _handle_as_scalars(self, topic: str, msg: Any) -> None:
        result = FlattenResult(numeric={}, texts=[])
        self._flatten_value("", msg, result, depth=0)

        base_path = entity_path(self._unit_name, topic, "graphs")
        for name, value in sorted(result.numeric.items()):
            rr.log(f"{base_path}/{name}", rr.Scalar(value))
        if result.texts:
            rr.log(f"{base_path}/text", rr.TextLog("\n".join(result.texts)))

    def _flatten_value(self, prefix: str, value: Any, result: FlattenResult, depth: int) -> None:
        if depth > 32:
            result.texts.append(f"{prefix}: maximum recursion depth reached")
            return
        if value is None:
            return
        if isinstance(value, bool):
            result.numeric[prefix or "value"] = 1.0 if value else 0.0
            return
        if isinstance(value, (int, float)):
            if math.isfinite(float(value)):
                result.numeric[prefix or "value"] = float(value)
            return
        if isinstance(value, str):
            key = prefix or "value"
            result.texts.append(f"{key}: {value}")
            return
        if isinstance(value, Time):
            result.numeric[prefix or "stamp"] = time_to_seconds(value)
            return
        if isinstance(value, Sequence) and not isinstance(value, (bytes, bytearray, str)):
            limit = min(len(value), 16)
            for idx in range(limit):
                child_prefix = f"{prefix}/{idx}" if prefix else str(idx)
                self._flatten_value(child_prefix, value[idx], result, depth + 1)
            if len(value) > limit:
                result.texts.append(f"{prefix}: sequence truncated at {limit} items")
            return
        if hasattr(value, "__slots__"):
            for slot in value.__slots__:
                attr = getattr(value, slot)
                child_prefix = f"{prefix}/{slot}" if prefix else slot
                self._flatten_value(child_prefix, attr, result, depth + 1)
            return
        if hasattr(value, "_fields_and_field_types"):
            for name in value.get_fields_and_field_types().keys():
                attr = getattr(value, name)
                child_prefix = f"{prefix}/{name}" if prefix else name
                self._flatten_value(child_prefix, attr, result, depth + 1)
            return
        key = prefix or "value"
        result.texts.append(f"{key}: unsupported field")


def build_topic_list(unit_name: str) -> List[TopicConfig]:
    sensor_qos = "sensor"

    prefixed: Iterable[TopicConfig] = [
        TopicConfig(f"/{unit_name}/can/board_status", "pkr_msgs/msg/EcuSTAT"),
        TopicConfig(f"/{unit_name}/can/eps_control", "pkr_msgs/msg/MotorCMD"),
        TopicConfig(f"/{unit_name}/can/eps_status", "pkr_msgs/msg/MotorSTAT"),
        TopicConfig(f"/{unit_name}/ctrl/task_request", "rtm_msgs/msg/IpkcComm"),
        TopicConfig(f"/{unit_name}/dkc/task_request", "rtm_msgs/msg/IpkcComm"),
        TopicConfig(f"/{unit_name}/driving_mode", "std_msgs/msg/UInt16"),
        TopicConfig(f"/{unit_name}/err_hdlr/cmd", "std_msgs/msg/UInt8"),
        TopicConfig(f"/{unit_name}/feedforward/motion_status", "dual_msgs/msg/MotionStatus"),
        TopicConfig(f"/{unit_name}/filt_rel_pose_1", "geometry_msgs/msg/Pose"),
        TopicConfig(f"/{unit_name}/front/ground_coeff", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/imu", "sensor_msgs/msg/Imu", sensor_qos),
        TopicConfig(f"/{unit_name}/itf/task_request", "rtm_msgs/msg/ItfComm"),
        TopicConfig(f"/{unit_name}/mon_cmd", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/monitoring/obstacle", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/monitoring/task", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/motion_Cmd", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/obstacle_detection/critical_path", "nav_msgs/msg/Path"),
        TopicConfig(f"/{unit_name}/obstacle_detection/obstacle_points", "sensor_msgs/msg/PointCloud2", sensor_qos),
        TopicConfig(f"/{unit_name}/obstacle_detection/warning_path", "nav_msgs/msg/Path"),
        TopicConfig(f"/{unit_name}/odom", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/other_err_hdlr/error", "std_msgs/msg/UInt64"),
        TopicConfig(f"/{unit_name}/other_obs_dualM", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/other_obs_dual_motion", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/other_tk_ctrl/init", "geometry_msgs/msg/PointStamped"),
        TopicConfig(f"/{unit_name}/pose_COG", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/pose_car", "geometry_msgs/msg/PoseStamped"),
        TopicConfig(f"/{unit_name}/rear/ground_coeff", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/rjh/task_request", "pkr_msgs/msg/RobotMissionTmp"),
        TopicConfig(f"/{unit_name}/rjh/task_response", "std_msgs/msg/String"),
        TopicConfig(f"/{unit_name}/rtm/stat", "rtm_msgs/msg/RtmStat"),
        TopicConfig(f"/{unit_name}/rtm/task_result", "pkr_msgs/msg/Tm2ActrRSLT"),
        TopicConfig(f"/{unit_name}/rtm/task_result_response", "std_msgs/msg/String"),
        TopicConfig(f"/{unit_name}/target_and_now_wassy_angle", "std_msgs/msg/Float64MultiArray"),
        TopicConfig(f"/{unit_name}/this/obs_dualM", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/this/obs_dual_motion", "geometry_msgs/msg/Twist"),
        TopicConfig(f"/{unit_name}/this/pose_COG", "nav_msgs/msg/Odometry"),
        TopicConfig(f"/{unit_name}/this/tk_ctrl/init", "geometry_msgs/msg/PointStamped"),
        TopicConfig(f"/{unit_name}/spatial_logmsg", "rcl_interfaces/msg/Log"),
    ]

    globals_: Iterable[TopicConfig] = [
        TopicConfig("/diagnostics_agg", "diagnostic_msgs/msg/DiagnosticArray"),
        TopicConfig("/hesai/pandar", "sensor_msgs/msg/PointCloud2", sensor_qos),
        TopicConfig("/right/hesai/pandar", "sensor_msgs/msg/PointCloud2", sensor_qos),
    ]

    return [*prefixed, *globals_]


def initialize_rerun(unit: str) -> None:
    rr.init(f"{unit}_ros_bridge")
    mode = os.environ.get("RERUN_MODE", "connect").lower()
    try:
        if mode == "spawn":
            rr.spawn()
        else:
            server = os.environ.get("RERUN_SERVER", "127.0.0.1:9876")
            rr.connect(server)
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Failed to initialize Rerun in {mode} mode: {exc}", file=sys.stderr)


def main(argv: Optional[Sequence[str]] = None) -> int:
    unit = normalize_unit(os.environ.get("P13_NAME"))
    initialize_rerun(unit)

    rclpy.init(args=argv)
    node = RerunBridge(unit)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
