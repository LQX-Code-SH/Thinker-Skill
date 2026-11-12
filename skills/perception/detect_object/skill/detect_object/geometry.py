"""3D 定位：mask 质心 + 3×3 中值深度 + 针孔反投影 + 光线延伸（纯 numpy）。"""
from __future__ import annotations

import numpy as np

DEPTH_SCALE = 1e-3          # 深度图单位 mm → m


def depth_at_uv(depth, u, v, win=1):
    """(u,v) 周围 (2*win+1)² 的有效深度中值（mm）；全无效返回 None。"""
    h, w = depth.shape[:2]
    u0, u1 = max(0, u - win), min(w, u + win + 1)
    v0, v1 = max(0, v - win), min(h, v + win + 1)
    patch = np.asarray(depth[v0:v1, u0:u1], dtype=np.float32)
    valid = patch[patch > 0]
    return float(np.median(valid)) if valid.size else None


def backproject(u, v, z_m, k):
    """针孔反投影：像素 + 深度(m) → 相机系 (X, Y, Z)。"""
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if fx <= 0 or fy <= 0:
        raise ValueError(f"invalid intrinsics: fx={fx}, fy={fy}")
    return np.array([(u - cx) * z_m / fx, (v - cy) * z_m / fy, z_m], dtype=np.float64)


def extend_along_ray(point, offset=0.03):
    """沿相机系光线单位向量延伸 offset 米。"""
    norm = float(np.linalg.norm(point))
    return point if norm <= 0 else point + point / norm * offset


def _to_camera(u, v, depth, k, offset, z_range):
    z_mm = depth_at_uv(depth, u, v)
    if z_mm is None:
        return None, "no_depth"
    z = z_mm * DEPTH_SCALE
    if not (z_range[0] < z < z_range[1]):
        return None, "depth_out_of_range"
    return extend_along_ray(backproject(u, v, z, k), offset), {"u": u, "v": v, "z_m": z}


def estimate_from_mask(mask, depth, k, *, offset=0.03, min_area=10, z_range=(0.05, 3.0)):
    ys, xs = np.nonzero(mask)
    if xs.size < min_area:
        return None, "mask_too_small"
    return _to_camera(int(np.median(xs)), int(np.median(ys)), depth, k, offset, z_range)


def estimate_from_box(box, depth, k, *, offset=0.03, z_range=(0.05, 3.0)):
    x1, y1, x2, y2 = box[:4]
    return _to_camera(int((x1 + x2) / 2), int((y1 + y2) / 2), depth, k, offset, z_range)
