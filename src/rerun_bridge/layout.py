"""Blueprint layout helpers for the ROS 2 to Rerun bridge."""

from __future__ import annotations

from typing import Any

import rerun as rr
import rerun.blueprint as rrb


# 이 함수는 pose_COG 위치 시계열을 보기 좋은 레이아웃으로 구성합니다.
def build_default_blueprint(unit_name: str) -> Any:
    position_origin = f"{unit_name}/graphs/pose_COG/pose/position"
    return rrb.Vertical(
        rrb.TimeSeriesView(
            origin=position_origin,
            name="pose_COG XY",
        ),
    )


# 이 함수는 생성된 블루프린트를 Rerun 뷰어에 전송합니다.
def send_default_blueprint(unit_name: str) -> None:
    rr.send_blueprint(build_default_blueprint(unit_name))
