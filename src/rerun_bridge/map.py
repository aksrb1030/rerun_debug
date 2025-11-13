"""Map visualization helpers for the ROS 2 to Rerun bridge."""

from __future__ import annotations

import sys
from os import PathLike as OsPathLike
from pathlib import Path
from typing import Optional, Union

import numpy as np
import rerun as rr

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False


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
        map_path_str = str(resolved)
        suffix = resolved.suffix.lower()
        print(f"Loading map from: {map_path_str}", file=sys.stderr)
        
        # PCD 또는 메시 파일 처리
        if suffix in ['.pcd', '.ply', '.obj', '.gltf', '.glb']:
            if not HAS_OPEN3D:
                print(
                    "open3d not installed. Install with: pip install open3d",
                    file=sys.stderr,
                )
                return False
            
            # open3d로 읽기
            print(f"Reading {suffix} file, please wait...", file=sys.stderr)
            if suffix == '.pcd':
                geom = o3d.io.read_point_cloud(map_path_str)
            else:
                # 메시 파일 읽기
                geom = o3d.io.read_triangle_mesh(map_path_str)
            
            print(f"File read complete, processing...", file=sys.stderr)
            
            # 포인트클라우드 처리
            if isinstance(geom, o3d.geometry.PointCloud):
                xyz = np.asarray(geom.points, dtype=np.float32)
                colors = None
                
                # 색상이 있으면 사용
                if len(geom.colors) > 0:
                    col = np.asarray(geom.colors, dtype=np.float32)
                    colors = (col * 255.0).clip(0, 255).astype(np.uint8)
                
                print(f"Loaded point cloud with {len(xyz)} points", file=sys.stderr)
                rr.log(
                    "world/map",
                    rr.Points3D(xyz, colors=colors, radii=0.02),
                )
                print(f"✓ Map successfully logged to Rerun", file=sys.stderr)
            # 메시 처리
            elif isinstance(geom, o3d.geometry.TriangleMesh):
                vertices = np.asarray(geom.vertices, dtype=np.float32)
                triangles = np.asarray(geom.triangles, dtype=np.uint32)
                
                colors = None
                if len(geom.vertex_colors) > 0:
                    col = np.asarray(geom.vertex_colors, dtype=np.float32)
                    colors = (col * 255.0).clip(0, 255).astype(np.uint8)
                
                print(f"Loaded mesh with {len(vertices)} vertices", file=sys.stderr)
                rr.log(
                    "world/map",
                    rr.Mesh3D(
                        vertex_positions=vertices,
                        triangle_indices=triangles,
                        vertex_colors=colors,
                    ),
                )
                print(f"✓ Map successfully logged to Rerun", file=sys.stderr)
        else:
            # 다른 형식은 Asset3D 시도
            rr.log(f"world/map", rr.Asset3D(path=map_path_str))
            
    except Exception as exc:  # pylint: disable=broad-except
        print(f"Failed to load map asset '{resolved}': {exc}", file=sys.stderr)
        return False

    return True


# 이 함수는 맵 경로가 주어졌을 때만 로깅을 시도하고 성공 여부를 반환합니다.
def maybe_send_map(unit_name: str, map_path: Optional[PathLike]) -> bool:
    if map_path is None:
        return False
    return send_map_asset(unit_name, map_path)

