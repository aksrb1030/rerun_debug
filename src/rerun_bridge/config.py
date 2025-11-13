"""Configuration helpers for the ROS 2 to Rerun bridge."""

from __future__ import annotations

import os
from typing import Mapping, MutableMapping

from .utils import normalize_unit

DEFAULT_UNIT_NAME = "P13"
UNIT_ENV_VAR = "P13_NAME"
RERUN_MODE_ENV_VAR = "RERUN_MODE"
RERUN_SERVER_ENV_VAR = "RERUN_SERVER"
DEFAULT_RERUN_MODE = "connect"
DEFAULT_RERUN_SERVER = "127.0.0.1:9876"


# 이 함수는 환경 변수에서 단위 이름을 읽어 정규화된 문자열로 반환합니다.
def get_unit_name(env: Mapping[str, str] | MutableMapping[str, str] | None = None) -> str:
    env_map = env if env is not None else os.environ
    return normalize_unit(env_map.get(UNIT_ENV_VAR), DEFAULT_UNIT_NAME)


# 이 함수는 Rerun 실행 모드를 환경 변수에서 읽어 소문자로 정규화합니다.
def get_rerun_mode(env: Mapping[str, str] | MutableMapping[str, str] | None = None) -> str:
    env_map = env if env is not None else os.environ
    return env_map.get(RERUN_MODE_ENV_VAR, DEFAULT_RERUN_MODE).lower()


# 이 함수는 Rerun 서버 주소를 환경 변수에서 읽어 반환합니다.
def get_rerun_server(env: Mapping[str, str] | MutableMapping[str, str] | None = None) -> str:
    env_map = env if env is not None else os.environ
    return env_map.get(RERUN_SERVER_ENV_VAR, DEFAULT_RERUN_SERVER)
