"""General utility helpers for the ROS 2 to Rerun bridge."""

from __future__ import annotations

from typing import Optional

from builtin_interfaces.msg import Time

# 이 함수는 단위 이름이 비어 있을 때 기본값을 적용하고 불필요한 슬래시를 제거합니다.
def normalize_unit(value: Optional[str], default: str) -> str:
    if value is None:
        return default
    cleaned = value.strip()
    if not cleaned:
        return default
    while cleaned.startswith("/"):
        cleaned = cleaned[1:]
    return cleaned or default


# 이 함수는 토픽과 카테고리를 결합하여 Rerun에서 사용할 엔티티 경로를 만듭니다.
def entity_path(unit: str, topic: str, category: str) -> str:
    prefix = f"/{unit}/"
    if topic.startswith(prefix):
        relative = topic[len(prefix) :].lstrip("/")
        return f"{unit}/{category}/{relative}"
    stripped = topic.lstrip("/")
    return f"global/{category}/{stripped}"


# 이 함수는 ROS 2 Time 메시지를 초 단위의 부동소수로 변환합니다.
def time_to_seconds(time_msg: Time) -> float:
    return float(time_msg.sec) + float(time_msg.nanosec) * 1e-9
