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


def estimate_from_plane(mask, depth, k, *, plane_z, to_base,
                        offset_xy=(0.0, 0.0), min_points=30,
                        z_range=(0.05, 3.0), mad_k=2.5,
                        plane_tol=0.006, normal=None):
    """点云平面定位（可选层）：沿桌面法向定 z，横向用投影中位数 + 标定偏移。

    这是"精度增强"层，默认关闭。只做两件事：

    1. **z 方向**：物体底面贴着已知平面，沿法向高度 ``t = (p - p0)·n`` 的
       高分位就是物体高度 ``H``，于是 ``z = plane_z + H/2``。这比"像素框高
       反算"稳得多——俯视时框高混了物体高度与直径，细高物体能差 30%。
    2. **xy 方向**：投影到平面后取中位数，再叠加配置里的标定偏移
       ``offset_xy``。单视角只能看到半面，中位数天然偏相机侧（实测
       1-2 cm），偏移量按场景标定即可。

    刻意**不做** PCA / 圆等形状拟合：在遮挡、粘连和离轴场景下增益不稳，
    且需要额外输出轴向与 3D 框。需要 6D（轴向 + 3D 框）时走 FoundationPose。

    ``normal`` 为平面法向（base 系），默认竖直向上；倾斜桌面由调用方传入。
    """
    pts = point_cloud(mask, depth, k, z_range=z_range)
    if pts.shape[0] < min_points:
        return None, "cloud_too_small"

    # 批量转 base（三次基向量变换，避免逐点调用）
    origin = np.asarray(to_base(np.zeros(3)), dtype=float)
    basis = np.column_stack([np.asarray(to_base(e), dtype=float) - origin
                             for e in np.eye(3)])
    pts_base = pts @ basis.T + origin
    pts_base = pts_base[np.isfinite(pts_base).all(axis=1)]
    if pts_base.shape[0] < min_points:
        return None, "cloud_too_small"

    n = np.asarray(normal if normal is not None else (0.0, 0.0, 1.0), dtype=float)
    norm_n = float(np.linalg.norm(n))
    if norm_n < 1e-9:
        return None, "invalid_normal"
    n = n / norm_n

    p0 = np.array([0.0, 0.0, float(plane_z)])
    rel = pts_base - p0
    t = rel @ n
    above = t > float(plane_tol)           # 去平面内点与平面以下噪声
    rel, t = rel[above], t[above]
    if t.size < min_points:
        return None, "cloud_too_small"

    # 高度：1-99 分位裁掉背景/孤立点，再取高分位
    t_lo, t_hi = np.percentile(t, [1.0, 99.0])
    sel = (t >= max(float(t_lo), 0.0)) & (t <= float(t_hi))
    if int(sel.sum()) >= min_points:
        rel, t = rel[sel], t[sel]
    height = float(np.percentile(t, 99.0))
    if height <= 1e-3:
        return None, "invalid_height"

    # 平面内正交基
    helper = (np.array([1.0, 0.0, 0.0]) if abs(float(n[0])) < 0.9
              else np.array([0.0, 1.0, 0.0]))
    u_axis = np.cross(n, helper)
    nu = float(np.linalg.norm(u_axis))
    if nu < 1e-9:
        return None, "invalid_normal"
    u_axis = u_axis / nu
    v_axis = np.cross(n, u_axis)

    uv = np.stack([rel @ u_axis, rel @ v_axis], axis=1)
    uv_med = np.median(uv, axis=0)
    rad = np.linalg.norm(uv - uv_med, axis=1)
    r_med = float(np.median(rad))
    r_mad = float(np.median(np.abs(rad - r_med))) + 1e-9
    keep = rad <= r_med + float(mad_k) * 1.4826 * r_mad   # 平面内离群剔除
    if int(keep.sum()) >= min_points:
        uv = uv[keep]

    center_uv = np.median(uv, axis=0) + np.asarray(offset_xy, dtype=float)
    center = (p0
              + u_axis * float(center_uv[0])
              + v_axis * float(center_uv[1])
              + n * (height / 2.0))

    # ── 目标轴 ────────────────────────────────────────────────
    # 桌面上的物体分两类，画轴方式不同：
    #  * 立着的（高度 >= 0.7 x 平面内最长边）→ 轴 = 桌面法向，轴心 = 目标中心；
    #  * 平躺的（香蕉、倒伏水瓶等）→ 轴 = 平面内最长方向（平行于桌面）。
    # 这里只借 PCA 判断"最长方向"，不参与定位。
    centered = uv - np.median(uv, axis=0)
    if centered.shape[0] >= 3:
        eigval, eigvec = np.linalg.eigh(np.cov(centered.T))
        major2d = eigvec[:, int(np.argmax(eigval))]
    else:
        major2d = np.array([1.0, 0.0])
    minor2d = np.array([-major2d[1], major2d[0]])
    proj_major = centered @ major2d
    proj_minor = centered @ minor2d
    length = float(np.percentile(proj_major, 98) - np.percentile(proj_major, 2))
    wide = float(np.percentile(proj_minor, 98) - np.percentile(proj_minor, 2))

    if height >= 0.7 * max(length, 1e-6):
        axis_vec = n                      # 立着：垂直于桌面
        axis_kind = "vertical"
    else:
        axis_vec = major2d[0] * u_axis + major2d[1] * v_axis   # 平躺：沿最长方向
        axis_kind = "longitudinal"

    info = {
        "method": "plane",
        "height": round(height, 4),
        "length": round(length, 4),
        "width": round(wide, 4),
        "offset_xy": [round(float(offset_xy[0]), 4), round(float(offset_xy[1]), 4)],
        "plane_z": float(plane_z),
        "points": int(uv.shape[0]),
        "axis": [round(float(v), 4) for v in axis_vec],
        "axis_kind": axis_kind,
    }
    return center, info


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



