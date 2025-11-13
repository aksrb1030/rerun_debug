"""Utilities for flattening ROS messages for logging."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Sequence

from builtin_interfaces.msg import Time

from .utils import time_to_seconds


# 이 데이터 클래스는 평탄화된 결과를 수치와 텍스트로 분리해 저장합니다.
@dataclass
class FlattenResult:
    numeric: Dict[str, float]
    texts: List[str]


# 이 함수는 메시지를 순회하기 위한 빈 결과 객체를 생성합니다.
def create_flatten_result() -> FlattenResult:
    return FlattenResult(numeric={}, texts=[])


# 이 함수는 주어진 메시지를 순회하며 수치 데이터와 문자열 로그를 수집합니다.
def flatten_message(value: Any) -> FlattenResult:
    result = create_flatten_result()
    _flatten_value("", value, result, depth=0)
    return result


# 이 함수는 재귀적으로 값을 검사하여 결과에 수치 또는 텍스트를 저장합니다.
def _flatten_value(prefix: str, value: Any, result: FlattenResult, depth: int) -> None:
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
            _flatten_value(child_prefix, value[idx], result, depth + 1)
        if len(value) > limit:
            result.texts.append(f"{prefix}: sequence truncated at {limit} items")
        return
    if hasattr(value, "__slots__"):
        for slot in value.__slots__:
            attr = getattr(value, slot)
            child_prefix = f"{prefix}/{slot}" if prefix else slot
            _flatten_value(child_prefix, attr, result, depth + 1)
        return
    if hasattr(value, "_fields_and_field_types"):
        for name in value.get_fields_and_field_types().keys():
            attr = getattr(value, name)
            child_prefix = f"{prefix}/{name}" if prefix else name
            _flatten_value(child_prefix, attr, result, depth + 1)
        return
    key = prefix or "value"
    result.texts.append(f"{key}: unsupported field")
