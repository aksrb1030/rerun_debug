"""Main entry point for the ROS 2 to Rerun bridge."""

from __future__ import annotations

from typing import Optional, Sequence

import rerun as rr
import rclpy
import sys

from .bridge import RerunBridge
from .config import get_rerun_mode, get_rerun_server, get_unit_name
from .layout import send_default_blueprint


# 이 함수는 Rerun SDK를 초기화하고 연결 또는 스폰 모드를 선택합니다.
def initialize_rerun(unit: str, *, mode: Optional[str] = None, server: Optional[str] = None) -> None:
    rr.init(f"{unit}_ros_bridge")
    chosen_mode = (mode or get_rerun_mode()).lower()
    chosen_server = server or get_rerun_server()

    try:
        if chosen_mode == "spawn":
            rr.spawn()
        elif chosen_mode == "connect":
            if hasattr(rr, "connect"):
                rr.connect(chosen_server)
            else:
                print(
                    "rerun.connect is not available; falling back to rerun.spawn().",
                    file=sys.stderr,
                )
                rr.spawn()
        else:
            print(
                f"Unknown RERUN_MODE '{chosen_mode}', defaulting to spawn.",
                file=sys.stderr,
            )
            rr.spawn()
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Failed to initialize Rerun in {chosen_mode} mode: {exc}", file=sys.stderr)

    send_default_blueprint(unit)


# 이 함수는 ROS 2 노드를 생성하고 스핀을 수행합니다.
def run_bridge(unit: str, argv: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=argv)
    node = RerunBridge(unit)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


# 이 함수는 브리지 실행의 진입점을 제공하며 종료 코드를 반환합니다.
def main(argv: Optional[Sequence[str]] = None) -> int:
    unit = get_unit_name()
    initialize_rerun(unit)
    run_bridge(unit, argv)
    return 0
