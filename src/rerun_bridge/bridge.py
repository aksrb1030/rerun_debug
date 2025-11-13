"""ROS 2 node that bridges messages into Rerun."""

from __future__ import annotations

from typing import Any, List, Optional

import rerun as rr
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles, QoSProfile
from rosidl_runtime_py.utilities import get_message

from builtin_interfaces.msg import Time

from .flatten import flatten_message
from .handlers import create_handler_map
from .topics import TopicConfig, build_topic_list
from .utils import entity_path, time_to_seconds


class RerunBridge(Node):
    # 이 생성자는 브리지를 초기화하고 필요한 구독을 생성합니다.
    def __init__(self, unit_name: str) -> None:
        super().__init__("p13_rerun_bridge")
        self._unit_name = unit_name
        self._handlers = create_handler_map(unit_name)
        self._subscriptions: List[Any] = []

        for config in build_topic_list(unit_name):
            try:
                self._create_subscription(config)
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(
                    f"Failed to subscribe {config.topic} ({config.type_string}): {exc}"
                )

    # 이 메서드는 토픽 설정에 맞춰 ROS 2 구독을 생성합니다.
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
        self.get_logger().info(
            f"Subscribed to {config.topic} ({config.type_string})"
        )

    # 이 메서드는 수신된 메시지를 적절한 처리기로 라우팅합니다.
    def _handle_message(self, config: TopicConfig, msg: Any) -> None:
        stamp = self._extract_header_stamp(msg)
        if stamp is not None:
            rr.set_time_seconds("ros_time", stamp)

        handler = self._handlers.get(config.type_string)
        if handler is not None:
            handler(config.topic, msg)
        else:
            self._handle_as_scalars(config.topic, msg)

    # 이 메서드는 메시지의 헤더에서 타임스탬프를 추출합니다.
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

    # 이 메서드는 특수화된 처리기가 없는 메시지를 스칼라 값으로 기록합니다.
    def _handle_as_scalars(self, topic: str, msg: Any) -> None:
        result = flatten_message(msg)
        base_path = entity_path(self._unit_name, topic, "graphs")
        for name, value in sorted(result.numeric.items()):
            rr.log(f"{base_path}/{name}", rr.Scalar(value))
        if result.texts:
            rr.log(f"{base_path}/text", rr.TextLog("\n".join(result.texts)))
