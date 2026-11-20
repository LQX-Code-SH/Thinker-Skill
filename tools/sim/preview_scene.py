#!/usr/bin/env python3
"""静态场景预览：把 MuJoCo 模型直接渲染出来，并高亮碰撞体。

和 `ros2 launch manip_sim sim.launch.py` 的区别：

- 桥的查看器只显示视觉组（`geomgroup[0]=0`，见 manip_sim/bridge.py），
  所以看不到碰撞体；
- 这里把 0~5 所有 geomgroup 全打开，并把 group 0（碰撞体）染成半透明红色，
  桌面物体的视觉网格调成半透明，方便检查"视觉 vs 碰撞"的尺寸/位置偏差。

用法（容器内，需要 DISPLAY）：
    source /thinker/tools/sim/env.sh
    python3 /thinker/tools/sim/preview_scene.py
    python3 /thinker/tools/sim/preview_scene.py --model <其它场景.xml>
"""
from __future__ import annotations

import argparse
import glob
import time


def default_model_path() -> str:
    hits = glob.glob(
        "/thinker/.colcon-sim/*/share/manip_description/assets/scene/"
        "custom_kitchen_living_robot.xml")
    if hits:
        return hits[0]
    return ("/thinker/sim/robot_sim/src/manip_description/assets/scene/"
            "custom_kitchen_living_robot.xml")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=None, help="场景 XML 路径")
    ap.add_argument("--visual-alpha", type=float, default=0.25,
                    help="桌面物体视觉网格透明度（0~1）")
    ap.add_argument("--all-collisions", action="store_true",
                    help="高亮所有碰撞体（默认只高亮 demo_* 桌面物体）")
    args = ap.parse_args()

    import mujoco
    import mujoco.viewer

    path = args.model or default_model_path()
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    collision = 0
    for g in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        is_collision = int(model.geom_group[g]) == 0
        if not is_collision:
            if name.startswith("demo_"):
                rgba = model.geom_rgba[g].copy()
                rgba[3] = args.visual_alpha       # 透明以露出内部碰撞体
                model.geom_rgba[g] = rgba
            continue
        if args.all_collisions or name.startswith("demo_"):
            model.geom_rgba[g] = [1.0, 0.15, 0.15, 0.35]   # 半透明红
            collision += 1
        else:
            # 场景/机器人的碰撞体默认隐藏，避免整屏全红
            rgba = model.geom_rgba[g].copy()
            rgba[3] = 0.0
            model.geom_rgba[g] = rgba

    print(f"场景: {path}")
    print(f"高亮碰撞体 {collision} 个（红色半透明），按 Esc 退出")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.opt.geomgroup[:] = 1          # 0~5 全开，含碰撞组
        viewer.cam.distance = 2.2
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -22
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
