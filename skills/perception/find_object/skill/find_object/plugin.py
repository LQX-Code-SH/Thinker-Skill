"""find_object：主动搜索 —— 头部扫描找目标，居中后跑 L2 定位。

`detect_object` 是无副作用的纯感知技能；主动搜索要移动头部，属于"带副作用
的感知"，单独成一个技能更清晰，也便于任务层决定"找不到就放弃"。

流程三步：

  ① 扫描：按 yaw×pitch 网格逐个位置 → 到位且稳定 → 调 detect_object(level=1)
  ② 居中：按 2D 框中心与图像中心的像素差反算头部角度，迭代 ≤N 次
  ③ 定位：居中后调 detect_object(level=2)，取最终 obj_pose

为什么每步都必须"到位并稳定再取帧"：实测头部运动中取帧会让 x 偏 7 cm ——
桥发布的 `/ob_camera_head/pose` 是**当前**姿态，而 `transform_to_base` 用的是
**取帧时刻**的关节角，运动时两者差一个检测周期。所以这里先下发目标、等关节角
进入容差并连续 N 个 tick 稳定，才触发检测。

为什么先 level=1 再 level=2：扫描/居中只关心"有没有、框在哪"，L1 不跑点云、
更快；确认居中后才用 L2 做正式定位。
"""
from __future__ import annotations

import math

from thinker_skill_sdk import ErrorCode, Step, StepResult

DEFAULT_YAW_DEG = 40.0
DEFAULT_PITCH_LIST = (12.0, 30.0, 45.0)
PITCH_MIN_RAD = math.radians(-15.0)   # 抬头极限（与真机 SDK 一致）
PITCH_MAX_RAD = math.radians(58.0)    # 低头极限
YAW_MAX_RAD = math.radians(60.0)


class FindObject:
    name = "find_object"
    requires = ["body.head"]
    postcondition = {}
    timeout = 150.0

    # ---------- 内部工具 ----------
    @staticmethod
    def _call(c, skill: str, payload: dict):
        """同步调用子技能；host 不支持组合时返回 None。"""
        call = getattr(c, "call_skill", None)
        if call is None:
            return None
        return call(skill, dict(payload))

    @staticmethod
    def _data(res) -> dict:
        return dict(getattr(res, "data", None) or {})

    @staticmethod
    def _ok(res) -> bool:
        return res is not None and getattr(res, "state", "") == "succeed"

    def build_steps(self, ctx, params):
        object_name = str(params.get("object_name", "")).strip()
        cfg = dict(ctx.cfg or {})

        if not object_name:
            return [Step(name="参数校验", kind="io", weight=1,
                         run=lambda c: StepResult(
                             status="failed", code=ErrorCode.SYS_CONFIG_ERROR,
                             data={"reason": "缺少 object_name"}))]

        level = int(params.get("level", cfg.get("level", 2)))
        yaw_deg = float(cfg.get("yaw_range_deg", DEFAULT_YAW_DEG))
        pitch_list = tuple(cfg.get("pitch_deg_list") or DEFAULT_PITCH_LIST)
        tol = math.radians(float(cfg.get("tol_deg", 3.0)))
        settle_ticks = max(1, int(cfg.get("settle_ticks", 3)))
        center_tol_px = float(cfg.get("center_tol_px", 20.0))
        center_iters = max(0, int(cfg.get("center_iters", 2)))
        yaw_sign = float(cfg.get("yaw_sign", 1.0))
        pitch_sign = float(cfg.get("pitch_sign", 1.0))
        img_w = float(cfg.get("image_width", 640.0))
        img_h = float(cfg.get("image_height", 480.0))
        fx = float(cfg.get("fx", 460.06))
        fy = float(cfg.get("fy", 460.06))

        # 扫描网格：先低头（桌面最可能在深俯仰），每个俯仰内先中间后两侧
        grid = []
        for p in sorted(pitch_list, reverse=True):
            for y in (0.0, -yaw_deg, yaw_deg):
                grid.append((y, p))

        st = {"idx": 0, "settle": 0, "found": None, "target": (0.0, 0.0),
              "center_left": center_iters, "center_offset": None,
              "trace": [], "last_pose": None}

        def _clamp(yaw, pitch):
            return (max(-YAW_MAX_RAD, min(YAW_MAX_RAD, yaw)),
                    max(PITCH_MIN_RAD, min(PITCH_MAX_RAD, pitch)))

        def _move(c, yaw, pitch):
            c.body.head.look_at(yaw=yaw, pitch=pitch)

        def _settled(c, yaw, pitch) -> bool:
            """到位 + 静止：连续 settle_ticks 个 tick 满足两者才算就绪。

            "静止"是与上一 tick 的姿态几乎不变（<0.11°）。只看"到位"不够：
            关节可能在容差内缓慢移动，此时取帧的位姿与图像仍不同步
            （实测运动中取帧 x 偏 7 cm）。
            """
            cur = c.body.head.get_pose()
            last = st["last_pose"]
            st["last_pose"] = cur

            if abs(cur[0] - yaw) > tol or abs(cur[1] - pitch) > tol:
                st["settle"] = 0
                return False
            if last is not None:
                moved = max(abs(cur[0] - last[0]), abs(cur[1] - last[1]))
                if moved >= 0.002:            # ≈0.11°，仍在动
                    st["settle"] = 0
                    return False
            st["settle"] += 1
            return st["settle"] >= settle_ticks

        # ① 扫描 -----------------------------------------------------------
        def scan(c):
            if st["idx"] >= len(grid):
                return StepResult(
                    status="failed",
                    code=ErrorCode.PERCEPTION_OBJECT_NOT_FOUND,
                    retryable=True,
                    data={"object_name": object_name,
                          "scanned": len(grid),
                          "reason": "扫描完所有位置仍未找到"},
                )
            yaw_i, pitch_i = grid[st["idx"]]
            yaw, pitch = _clamp(math.radians(yaw_i), math.radians(pitch_i))
            _move(c, yaw, pitch)
            if not _settled(c, yaw, pitch):
                return StepResult(status="running")

            st["settle"] = 0
            res = self._call(c, "detect_object",
                             {"object_name": object_name, "level": 1})
            if res is None:
                return StepResult(
                    status="failed", code=ErrorCode.SYS_CONFIG_ERROR,
                    data={"reason": "当前 host 不支持技能组合调用"})
            data = self._data(res)
            if self._ok(res) and data.get("box"):
                st["found"] = data
                st["target"] = (yaw, pitch)
                return StepResult(status="done",
                                  data={"box": data.get("box"),
                                        "scanned": st["idx"] + 1})
            st["idx"] += 1
            return StepResult(status="running")

        # ② 居中 -----------------------------------------------------------
        def center(c):
            box = (st["found"] or {}).get("box") or []
            if len(box) != 4:
                return StepResult(status="done", data={"center": "no_box"})

            cu = 0.5 * (float(box[0]) + float(box[2]))
            cv = 0.5 * (float(box[1]) + float(box[3]))
            du, dv = cu - img_w / 2.0, cv - img_h / 2.0
            st["center_offset"] = [round(du, 1), round(dv, 1)]
            if math.hypot(du, dv) <= center_tol_px:
                return StepResult(status="done",
                                  data={"center": "ok", "offset_px": st["center_offset"]})
            if st["center_left"] <= 0:
                return StepResult(status="done",
                                  data={"center": "gave_up",
                                        "offset_px": st["center_offset"]})

            # 像素偏差 → 角度增量（小角度近似：Δθ ≈ atan(Δpx / f)）
            yaw0, pitch0 = st["target"]
            yaw, pitch = _clamp(yaw0 + yaw_sign * math.atan2(du, fx),
                                pitch0 + pitch_sign * math.atan2(dv, fy))
            _move(c, yaw, pitch)
            if not _settled(c, yaw, pitch):
                return StepResult(status="running")

            st["settle"] = 0
            st["center_left"] -= 1
            st["target"] = (yaw, pitch)
            cur_yaw, cur_pitch = c.body.head.get_pose()
            st["trace"].append({
                "du": round(du, 1), "dv": round(dv, 1),
                "target_yaw_deg": round(math.degrees(yaw), 1),
                "target_pitch_deg": round(math.degrees(pitch), 1),
                "actual_yaw_deg": round(math.degrees(cur_yaw), 1),
                "actual_pitch_deg": round(math.degrees(cur_pitch), 1),
            })
            res = self._call(c, "detect_object",
                             {"object_name": object_name, "level": 1})
            data = self._data(res)
            if self._ok(res) and data.get("box"):
                st["found"] = data
                return StepResult(status="running")     # 再判一次是否已居中
            return StepResult(status="done",
                              data={"center": "lost",
                                    "offset_px": st["center_offset"]})

        # ③ 正式定位 -------------------------------------------------------
        def locate(c):
            # 居中后头部可能仍在微动：再等一次"到位 + 静止"才取帧
            if not _settled(c, *st["target"]):
                return StepResult(status="running")
            st["settle"] = 0
            st["last_pose"] = None
            res = self._call(c, "detect_object",
                             {"object_name": object_name, "level": level})
            if res is None:
                return StepResult(
                    status="failed", code=ErrorCode.SYS_CONFIG_ERROR,
                    data={"reason": "当前 host 不支持技能组合调用"})
            data = self._data(res)
            pose = data.get("obj_pose")
            if not self._ok(res) or not pose:
                return StepResult(
                    status="failed",
                    code=getattr(res, "code", None) or ErrorCode.PERCEPTION_POSE_INVALID,
                    retryable=bool(getattr(res, "retryable", False)),
                    data={"object_name": object_name, "reason": "居中后定位失败",
                          "center_offset_px": st["center_offset"],
                          "center_trace": st["trace"], "head_target_deg":
                          [round(math.degrees(st["target"][0]), 1),
                           round(math.degrees(st["target"][1]), 1)],
                          "detail": data},
                )
            c.state.set("last_detection", data)
            out = dict(data)
            out["center_offset_px"] = st["center_offset"]
            out["center_trace"] = st["trace"]
            out["head_target"] = [round(math.degrees(st["target"][0]), 1),
                                  round(math.degrees(st["target"][1]), 1)]
            return StepResult(status="done", data=out)

        return [
            Step(name="扫描", kind="perception", weight=3, tick_budget=3.0, run=scan),
            Step(name="居中", kind="perception", weight=2, tick_budget=3.0, run=center),
            Step(name="定位", kind="perception", weight=3, tick_budget=3.0, run=locate),
        ]
