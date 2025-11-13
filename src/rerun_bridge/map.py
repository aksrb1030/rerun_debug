"""Map visualization helpers for the ROS 2 to Rerun bridge."""

from __future__ import annotations

import sys
from os import PathLike as OsPathLike
from pathlib import Path
from typing import Optional, Union

import rerun as rr


PathLike = Union[str, OsPathLike[str]]


# 이 함수는 제공된 파일 경로에서 3D 맵 자산을 불러와 뷰어에 기록합니다.
def send_map_asset(unit_name: str, map_path: PathLike) -> bool:
    resolved = Path(map_path).expanduser()
    try:
        resolved = resolved.resolve()
    except Exception:  # pylint: disable=broad-except
        # 절대 경로 변환에 실패하더라도 이후 단계에서 처리합니다.
        resolved = resolved.absolute()

    if not resolved.exists():
        print(f"Map file does not exist: {resolved}", file=sys.stderr)
        return False

    try:
        rr.log(f"{unit_name}/scene/map", rr.Asset3D(path=str(resolved)))
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Failed to load map asset '{resolved}': {exc}", file=sys.stderr)
        return False

    return True


# 이 함수는 맵 경로가 주어졌을 때만 로깅을 시도하고 성공 여부를 반환합니다.
def maybe_send_map(unit_name: str, map_path: Optional[PathLike]) -> bool:
    if map_path is None:
        return False
    return send_map_asset(unit_name, map_path)

