#!/usr/bin/env python3
"""检测看门狗：轮询请求文件 → 调用技能 → 写结果文件。

默认调 find_object（扫描 → 居中 → 居中标定下定 L2），与技能链一致；
请求文件里可以用 ``skill`` 字段改成 detect_object（原地检测、不动头部）。

为什么单独一个进程：

    image_view_server 需要在同一进程里维持 rclpy 订阅（相机流）。一旦在该
    进程里 subprocess 调技能服务，fork 会破坏 rclpy 的 context，之后所有
    ROS 调用都报 "rcl node's context is invalid"，页面还在但流只出一帧。

    所以取流服务只往 /thinker/.detect_req.json 写请求，本进程读取请求、
    调用 tools/sim/sim_call.py，再把结果写回 /thinker/.detect_res.json。

    两个容器共享 /thinker 挂载，所以请求/结果文件可以跨容器传递。

用法（技能容器内）：
    python3 tools/sim/detect_watcher.py
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

REQ = Path("/thinker/.detect_req.json")
RES = Path("/thinker/.detect_res.json")
SIM_CALL = Path(__file__).with_name("sim_call.py")


def _call(object_name: str, skill: str = "find_object", timeout: int = 120) -> dict:
    cmd = [sys.executable, str(SIM_CALL), skill,
           "--params", json.dumps({"object_name": object_name}, ensure_ascii=False),
           "--timeout", str(timeout)]
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "42")
    env.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE",
                   "/thinker/sim/robot_sim/config/fastdds/fastdds_shm.xml")
    env.setdefault("THINKER_WEIGHTS_DIR", "/thinker/weights")
    proc = subprocess.run(cmd, capture_output=True, text=True,
                          timeout=timeout + 30, check=False, env=env)
    data: dict = {}
    for line in (proc.stdout or "").splitlines():
        if line.startswith("data="):
            try:
                data = json.loads(line[5:])
            except ValueError:
                data = {}
        elif line.startswith("state="):
            parts = dict(p.split("=", 1) for p in line.split() if "=" in p)
            data["_state"] = parts.get("state", "")
        elif line.startswith("msg="):
            data["_msg"] = line[4:]
    if not data:
        data = {"_state": "failed",
                "_msg": (proc.stderr or proc.stdout or "无输出").strip()[:200]}
    return data


def main() -> int:
    last_mtime = 0.0
    print(f"检测看门狗已启动：{REQ} → {RES}", flush=True)
    while True:
        try:
            mtime = REQ.stat().st_mtime
        except FileNotFoundError:
            time.sleep(0.2)
            continue
        if mtime <= last_mtime:
            time.sleep(0.2)
            continue
        last_mtime = mtime
        try:
            request = json.loads(REQ.read_text())
        except (OSError, ValueError):
            continue
        name = str(request.get("object") or "apple")
        # 默认走 find_object：与技能链一致（扫描 + 居中 + 居中标定）；
        # 传 skill=detect_object 可退回"原地检测、不移动头部"。
        skill = str(request.get("skill") or "find_object")
        print(f"收到检测请求: {name} ({skill})", flush=True)
        started = time.time()
        data = _call(name, skill)
        payload = {"object": name, "data": data, "at": time.time(),
                   "elapsed": round(time.time() - started, 2)}
        try:
            RES.write_text(json.dumps(payload, ensure_ascii=False))
        except OSError as exc:
            print(f"写结果失败: {exc}", flush=True)
        print(f"检测完成 {name}: {data.get('_state')} ({payload['elapsed']}s)", flush=True)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
