"""Specialized message handlers for the Rerun bridge."""

from __future__ import annotations

import math
from typing import Any, Callable, Dict

import rerun as rr
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64MultiArray, String

from .utils import entity_path

HandlerFn = Callable[[str, Any], None]


class SpecializedHandlers:
    # 이 생성자는 처리기에 사용될 단위 이름을 저장합니다.
    def __init__(self, unit_name: str) -> None:
        self._unit_name = unit_name

    # 이 메서드는 Twist 메시지를 받아 선형 및 각속도 정보를 그래프로 기록합니다.
    def handle_twist(self, topic: str, msg: Twist) -> None:
        base_path = entity_path(self._unit_name, topic, "graphs")
        rr.log(f"{base_path}/linear/x", rr.Scalar(float(msg.linear.x)))
        rr.log(f"{base_path}/linear/y", rr.Scalar(float(msg.linear.y)))
        rr.log(f"{base_path}/linear/z", rr.Scalar(float(msg.linear.z)))
        rr.log(f"{base_path}/angular/x", rr.Scalar(float(msg.angular.x)))
        rr.log(f"{base_path}/angular/y", rr.Scalar(float(msg.angular.y)))
        rr.log(f"{base_path}/angular/z", rr.Scalar(float(msg.angular.z)))

    # 이 메서드는 Pose 데이터를 받아 Rerun에서 사용할 변환으로 변환합니다.
    def handle_pose(self, topic: str, translation: Point, rotation: Quaternion) -> None:
        rr.log(
            entity_path(self._unit_name, topic, "pose"),
            rr.Transform3D(
                translation=[float(translation.x), float(translation.y), float(translation.z)],
                rotation=rr.Quaternion(
                    xyzw=[
                        float(rotation.x),
                        float(rotation.y),
                        float(rotation.z),
                        float(rotation.w),
                    ]
                ),
            ),
        )

    # 이 메서드는 Pose 메시지를 받아 pose 처리기로 전달합니다.
    def handle_pose_message(self, topic: str, msg: Pose) -> None:
        self.handle_pose(topic, msg.position, msg.orientation)

    # 이 메서드는 PoseStamped 메시지를 받아 pose 처리기로 전달합니다.
    def handle_pose_stamped(self, topic: str, msg: PoseStamped) -> None:
        self.handle_pose_message(topic, msg.pose)

    # 이 메서드는 PointStamped 메시지를 받아 단일 점을 기록합니다.
    def handle_point_stamped(self, topic: str, msg: PointStamped) -> None:
        rr.log(
            entity_path(self._unit_name, topic, "sensors"),
            rr.Points3D(positions=[[float(msg.point.x), float(msg.point.y), float(msg.point.z)]]),
        )

    # 이 메서드는 Odometry 메시지를 받아 자세와 속도 정보를 기록합니다.
    def handle_odometry(self, topic: str, msg: Odometry) -> None:
        self.handle_pose(topic, msg.pose.pose.position, msg.pose.pose.orientation)
        base_path = entity_path(self._unit_name, topic, "graphs")
        rr.log(f"{base_path}/twist/linear/x", rr.Scalar(float(msg.twist.twist.linear.x)))
        rr.log(f"{base_path}/twist/linear/y", rr.Scalar(float(msg.twist.twist.linear.y)))
        rr.log(f"{base_path}/twist/linear/z", rr.Scalar(float(msg.twist.twist.linear.z)))
        rr.log(f"{base_path}/twist/angular/x", rr.Scalar(float(msg.twist.twist.angular.x)))
        rr.log(f"{base_path}/twist/angular/y", rr.Scalar(float(msg.twist.twist.angular.y)))
        rr.log(f"{base_path}/twist/angular/z", rr.Scalar(float(msg.twist.twist.angular.z)))

    # 이 메서드는 Path 메시지를 받아 경로를 선분으로 시각화합니다.
    def handle_path(self, topic: str, msg: Path) -> None:
        positions = [
            [float(p.pose.position.x), float(p.pose.position.y), float(p.pose.position.z)]
            for p in msg.poses
        ]
        if positions:
            rr.log(
                entity_path(self._unit_name, topic, "paths"),
                rr.LineStrips3D([positions]),
            )

    # 이 메서드는 문자열 메시지를 받아 로그 텍스트로 기록합니다.
    def handle_string(self, topic: str, msg: String) -> None:
        rr.log(entity_path(self._unit_name, topic, "logs"), rr.TextLog(msg.data))

    # 이 메서드는 Float64MultiArray 메시지를 받아 각 요소를 개별 그래프로 기록합니다.
    def handle_float64_multi_array(self, topic: str, msg: Float64MultiArray) -> None:
        base_path = entity_path(self._unit_name, topic, "graphs")
        for idx, value in enumerate(msg.data):
            rr.log(f"{base_path}/{idx}", rr.Scalar(float(value)))

    # 이 메서드는 IMU 메시지를 받아 자세와 가속도 및 각속도 벡터를 기록합니다.
    def handle_imu(self, topic: str, msg: Imu) -> None:
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
            rr.Arrows3D(
                origins=origin,
                vectors=[[float(msg.angular_velocity.x), float(msg.angular_velocity.y), float(msg.angular_velocity.z)]],
            ),
        )
        rr.log(
            f"{base_path}/linear_acceleration",
            rr.Arrows3D(
                origins=origin,
                vectors=[[float(msg.linear_acceleration.x), float(msg.linear_acceleration.y), float(msg.linear_acceleration.z)]],
            ),
        )

    # 이 메서드는 PointCloud2 메시지를 받아 유효한 포인트만 추출하여 기록합니다.
    def handle_point_cloud(self, topic: str, msg: PointCloud2) -> None:
        points_iter = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = [
            [float(x), float(y), float(z)]
            for x, y, z in points_iter
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z)
        ]
        if points:
            rr.log(entity_path(self._unit_name, topic, "sensors"), rr.Points3D(positions=points))


# 이 함수는 특수화된 메시지 처리기의 사전을 생성합니다.
def create_handler_map(unit_name: str) -> Dict[str, HandlerFn]:
    handler = SpecializedHandlers(unit_name)
    return {
        "geometry_msgs/msg/Twist": handler.handle_twist,
        "geometry_msgs/msg/Pose": handler.handle_pose_message,
        "geometry_msgs/msg/PoseStamped": handler.handle_pose_stamped,
        "geometry_msgs/msg/PointStamped": handler.handle_point_stamped,
        "nav_msgs/msg/Odometry": handler.handle_odometry,
        "nav_msgs/msg/Path": handler.handle_path,
        "std_msgs/msg/String": handler.handle_string,
        "std_msgs/msg/Float64MultiArray": handler.handle_float64_multi_array,
        "sensor_msgs/msg/Imu": handler.handle_imu,
        "sensor_msgs/msg/PointCloud2": handler.handle_point_cloud,
    }
