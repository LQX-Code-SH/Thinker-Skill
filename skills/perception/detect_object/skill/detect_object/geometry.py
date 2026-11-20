"""3D 定位：mask 质心 + 3×3 中值深度 + 针孔反投影 + 光线延伸（纯 numpy）。"""
from __future__ import annotations

import numpy as np

DEPTH_SCALE = 1e-3          # 深度图单位 mm → m


def _as_hw(depth):
    """统一成 H×W：兼容带通道维的深度图（H×W×1）。"""
    arr = np.asarray(depth)
    return arr[..., 0] if arr.ndim == 3 else arr


def depth_at_uv(depth, u, v, win=1):
    """(u,v) 周围 (2*win+1)² 的有效深度中值（mm）；全无效返回 None。"""
    depth = _as_hw(depth)
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


# ---------- 点云版本（mask → 3D 中位数，替代单点深度） ----------
def point_cloud(mask, depth, k, *, z_range=(0.05, 3.0)):
    """mask 内所有有效深度像素 → 相机系点云 ``N×3``（米）。"""
    ys, xs = np.nonzero(np.asarray(mask))
    if xs.size == 0:
        return np.zeros((0, 3))
    depth = _as_hw(depth)
    z = np.asarray(depth[ys, xs], dtype=np.float32) * DEPTH_SCALE
    ok = (z > z_range[0]) & (z < z_range[1])
    xs, ys, z = xs[ok], ys[ok], z[ok]
    if xs.size == 0:
        return np.zeros((0, 3))
    fx, fy, cx, cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])
    if fx <= 0 or fy <= 0:
        raise ValueError(f"invalid intrinsics: fx={fx}, fy={fy}")
    X = (xs.astype(np.float64) - cx) * z / fx
    Y = (ys.astype(np.float64) - cy) * z / fy
    return np.stack([X, Y, z.astype(np.float64)], axis=1)


def estimate_from_plane(mask, depth, k, *, box, plane_z, to_base,
                        min_points=30, z_range=(0.05, 3.0), mad_k=2.5):
    """平面约束定位：物体立在已知高度 ``plane_z`` 的平面上（base 系）。

    单视角只能看到物体的一半，从可见面推中心必然偏；但"物体底面贴着平面"
    是硬约束，可以直接定出高度方向：

    - 物体物理高度 ``h = 框像素高 × 深度 / fy``（框内不会有背景，比点云
      z_max 稳；斜视时会被透视压缩，属于已知残差）；
    - 中心高度 ``z = plane_z + h/2``；
    - 横向 ``x, y`` 取可见表面点中位数（同样偏相机侧，离轴越远越明显）。

    实测（仿真，低头 15°）：苹果 z 误差 +1 mm，四物体 z 误差 1–47 mm，
    优于"表面 + 半径补偿"的 ~100 mm。``to_base`` 是把相机系点转到 base 系
    的可调用对象（真机与仿真各自的坐标链不同）。
    """
    pts = point_cloud(mask, depth, k, z_range=z_range)
    if pts.shape[0] < min_points:
        return None, "cloud_too_small"

    center = np.median(pts, axis=0)
    dist = np.linalg.norm(pts - center, axis=1)
    med = float(np.median(dist))
    mad = float(np.median(np.abs(dist - med))) + 1e-9
    keep = dist <= med + float(mad_k) * 1.4826 * mad
    kept = pts[keep] if int(keep.sum()) >= min_points else pts

    surface = np.median(kept, axis=0)
    z_cam = float(surface[2])
    fy = float(k[4])
    box_h = abs(float(box[3]) - float(box[1])) if box is not None else 0.0
    h = box_h * z_cam / fy if fy > 0 else 0.0
    if h <= 0.0:
        return None, "invalid_box"

    base = np.asarray(to_base(surface), dtype=float)
    center_base = np.array([base[0], base[1], float(plane_z) + h / 2.0])
    info = {
        "method": "plane",
        "height": round(h, 4),
        "plane_z": float(plane_z),
        "points": int(kept.shape[0]),
    }
    return center_base, info


def estimate_from_cloud(mask, depth, k, *, radius=None, min_points=30,
                        z_range=(0.05, 3.0), mad_k=2.5, trim_ratio=0.0,
                        surface_quantile=0.05, face_band=0.012):
    """mask 点云的 3D 定位：最近表面 + 物理半径补偿。

    单目/单视角只能看到物体的一半，点云中心必然偏向可见面。这里按几何做：

    1. 剔除离群（MAD），去掉 mask 边缘混进来的桌面/背景；
    2. 取**最近表面**：相机系 z 的 ``surface_quantile`` 分位附近（``face_band``
       米内）的点集作为"最凸面"，用它的中位数作为该面的代表点；
    3. 沿视线（远离相机）补偿一个**物理半径** —— 从最凸点走一个半径即到
       几何中心，与物体尺寸无关地成立；
    4. 半径默认取横向尺寸 ``min(extent_x, extent_y)/2``（可见轮廓宽度的一半），
       显式传 ``radius`` 时用给定值。
    """
    pts = point_cloud(mask, depth, k, z_range=z_range)
    if pts.shape[0] < min_points:
        return None, "cloud_too_small"

    center = np.median(pts, axis=0)
    dist = np.linalg.norm(pts - center, axis=1)
    med = float(np.median(dist))
    mad = float(np.median(np.abs(dist - med))) + 1e-9
    keep = dist <= med + float(mad_k) * 1.4826 * mad
    kept = pts[keep] if int(keep.sum()) >= min_points else pts

    if trim_ratio and trim_ratio > 0.0:
        lo = np.quantile(kept, trim_ratio, axis=0)
        hi = np.quantile(kept, 1.0 - trim_ratio, axis=0)
        sel = np.all((kept >= lo) & (kept <= hi), axis=1)
        if int(sel.sum()) >= min_points:
            kept = kept[sel]

    extent = kept.max(axis=0) - kept.min(axis=0)

    # 最近表面：z 分位附近的点（最凸面朝向相机）
    z_near = float(np.quantile(kept[:, 2], float(surface_quantile)))
    face = kept[kept[:, 2] <= z_near + float(face_band)]
    if face.shape[0] < max(10, min_points // 3):
        face = kept
    surface = np.median(face, axis=0)

    r = float(radius) if radius is not None else float(min(extent[0], extent[1])) / 2.0
    norm = float(np.linalg.norm(surface))
    center = surface + surface / norm * r if norm > 1e-6 else surface

    info = {
        "points": int(kept.shape[0]),
        "outliers": int(pts.shape[0] - kept.shape[0]),
        "surface_points": int(face.shape[0]),
        "radius": round(r, 4),
        "extent": [float(v) for v in extent],
    }
    return center, info



