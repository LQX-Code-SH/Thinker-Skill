#!/usr/bin/env bash
# 容器内运行：xmigcs FSM + MuJoCo 桥 + 遥控器（半身 HBWALK 路径）
#   VIEWER=1 /thinker/tools/sim/run_fsm.sh   # 前台带窗口
#   VIEWER=0 /thinker/tools/sim/run_fsm.sh   # 无窗口（后台自动化）
set -eo pipefail

VIEWER="${VIEWER:-1}"
source /thinker/tools/sim/env.sh
source /thinker/.colcon-sim/robot-install/setup.bash

# 用仓库参数覆盖厂商出厂值（摇杆速度/斜坡、SWR 指令平滑）
for f in control_tool swr; do
    src="/thinker/sim/robot_sim/config/xmigcs/$f.yaml"
    [ -f "$src" ] || continue
    dst=$(find /opt/xmigcs-venv/lib/python3.12/site-packages/xmigcs -name "$f.yaml" 2>/dev/null | head -1)
    if [ -n "$dst" ]; then cp "$src" "$dst"; echo "[sim] 已应用 $f.yaml"; fi
done

INIT_ARGS="hang_height:=${SIM_HANG_Z:-1.00}"
for kv in "init_x:SIM_INIT_X" "init_y:SIM_INIT_Y" "init_z:SIM_INIT_Z" \
          "init_yaw_deg:SIM_INIT_YAW"; do
    key="${kv%%:*}"
    var="${kv##*:}"
    val="${!var:-}"
    if [ -n "$val" ]; then
        INIT_ARGS="$INIT_ARGS $key:=$val"
    fi
done

if [ "$VIEWER" = "1" ]; then
    ros2 launch manip_sim sim.launch.py fixed_base:=false viewer:=true headless:=false $INIT_ARGS &
else
    ros2 launch manip_sim sim.launch.py fixed_base:=false viewer:=false headless:=true $INIT_ARGS &
fi
LAUNCH_PID=$!
trap 'kill $LAUNCH_PID 2>/dev/null || true' EXIT INT TERM

if [ "$VIEWER" = "1" ] && python3 -c "import tkinter" 2>/dev/null; then
    ros2 run manip_sim teleop_gui > /tmp/teleop_gui.log 2>&1 &
    echo "[sim] 遥控器 GUI 已启动（/tmp/teleop_gui.log）"
fi

xmigcs_joystick > /tmp/xmigcs_joy.log 2>&1 &
sleep 6
echo "[sim] 遥控器长按 A: gotoSTART -> SWR，再 gotoHBWALK 进半身模式"
xmigcs_sim --ros-args -p hric_server_enabled:=false