#!/usr/bin/env python3
"""通过 ROS Service 调一次技能（仿真/真机同一通道）。

走的是真实调用链：Service -> Host.execute -> Runtime -> Pipeline -> Step，
因此能验证适配器、能力校验、租约与技能逻辑，而不是只测算法。

用法（容器内，host 已启动）：
    python3 tools/sim/sim_call.py look_down_table --params '{"pitch_deg": 45}'
"""
from __future__ import annotations

import argparse
import json
import sys
import time


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="调用技能 ControlCommand 服务")
    parser.add_argument("skill", help="技能名，如 look_down_table")
    parser.add_argument("--params", default="{}", help="参数 JSON 对象")
    parser.add_argument("--timeout", type=float, default=30.0, help="等待结果秒数")
    args = parser.parse_args(argv)

    try:
        params = json.loads(args.params)
    except ValueError as exc:
        print(f"params 不是合法 JSON: {exc}")
        return 2
    if not isinstance(params, dict):
        print("params 必须是 JSON 对象")
        return 2

    import rclpy
    from rclpy.node import Node
    from thinker_interfaces.srv import ControlCommand

    rclpy.init()
    node = Node("thinker_sim_call")
    service = f"/thinker/skill/{args.skill}/command"
    client = node.create_client(ControlCommand, service)
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            print(f"服务未就绪: {service}（先 make sim-host）")
            return 2

        request = ControlCommand.Request()
        request.command = args.skill
        request.params_json = json.dumps(params, ensure_ascii=False)
        future = client.call_async(request)

        deadline = time.monotonic() + max(1.0, args.timeout)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            print(f"调用超时（{args.timeout:.0f}s）")
            return 2

        result = future.result()
        print(f"state={result.state} code={result.code} "
              f"retryable={result.retryable}")
        print(f"msg={result.msg}")
        print(f"data={result.data_json}")
        return 0 if result.state == "succeed" else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
