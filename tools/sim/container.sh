#!/usr/bin/env bash
# robot_sim + Thinker-Skill 的 Jazzy 容器封装。
# 用法：tools/sim/container.sh up|down|build|probe|bridge|fsm|host|selftest|call|image-view|shell
#       bridge-bg / host-bg / logs 用于非交互终端（日志落 /tmp）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMAGE="${IMAGE:-manip_local:v4}"
CONTAINER="${CONTAINER:-thinker_sim}"
ENV="source /thinker/tools/sim/env.sh"
ROBOT_BUILD="/thinker/.colcon-sim/robot-build"
ROBOT_INSTALL="/thinker/.colcon-sim/robot-install"
THINKER_BUILD="/thinker/.colcon-sim/build"
THINKER_INSTALL="/thinker/.colcon-sim/install"

# 交互终端才要 -it；从脚本/CI 调用没有 TTY 时直接 exec
EXEC_TTY=""
if [ -t 0 ] && [ -t 1 ]; then
    EXEC_TTY="-it"
fi

# 仿真初始位姿（世界系）：x/y/z 米、yaw 度
INIT_ARGS=""
for kv in "init_x:SIM_INIT_X" "init_y:SIM_INIT_Y" "init_z:SIM_INIT_Z" \
          "init_yaw_deg:SIM_INIT_YAW"; do
    key="${kv%%:*}"
    var="${kv##*:}"
    val="${!var:-}"
    if [ -n "$val" ]; then
        INIT_ARGS="$INIT_ARGS $key:=$val"
    fi
done

need_container() {
    if ! docker inspect -f "{{.State.Running}}" "$CONTAINER" 2>/dev/null | grep -q true; then
        echo "容器 $CONTAINER 未运行，先执行 make sim-up" >&2
        exit 2
    fi
}

case "${1:-}" in
  up)
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    X11_ARGS=""
    if [ -n "${DISPLAY:-}" ] && [ -S "/tmp/.X11-unix/X${DISPLAY#*:}" ]; then
      xhost +local: >/dev/null 2>&1 || true
      X11_ARGS="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw"
    fi
    # SDK 可选挂载：容器镜像不含 tienkung_dex/ros2_bridge_msgs，
    # 用 XHUMANOID_SDK_DIR 指向已构建的 SDK（含 install 树或 python 目录）
    SDK_ARGS=""
    if [ -n "${XHUMANOID_SDK_DIR:-}" ]; then
      if [ -d "$XHUMANOID_SDK_DIR" ]; then
        SDK_ARGS="-v $XHUMANOID_SDK_DIR:/opt/xhumanoid_sdk:ro"
      else
        echo "警告: XHUMANOID_SDK_DIR 不存在: $XHUMANOID_SDK_DIR" >&2
      fi
    fi
    DRI_ARGS=""
    if compgen -G "/dev/dri/card*" >/dev/null 2>&1; then
      DRI_ARGS="--device /dev/dri --group-add video"
    fi
    docker run -d --name "$CONTAINER" --network host --ipc host \
      -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" $X11_ARGS $SDK_ARGS $DRI_ARGS \
      -v "$ROOT":/thinker:rw \
      --entrypoint bash "$IMAGE" -lc "sleep infinity"
    echo "容器已启动: $CONTAINER（$IMAGE）"
    ;;
  down)
    docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
    echo "容器已停止"
    ;;
  build)
    need_container
    docker exec "$CONTAINER" bash -lc "$ENV
      cd /thinker/sim/robot_sim
      colcon build --symlink-install --base-paths src \
        --build-base $ROBOT_BUILD --install-base $ROBOT_INSTALL
      source $ROBOT_INSTALL/setup.bash
      cd /thinker
      colcon build --symlink-install --base-paths core robots skills \
        --build-base $THINKER_BUILD --install-base $THINKER_INSTALL
    "
    ;;
  bridge)
    need_container
    # 前台：带窗口；后台自动化请用 bridge-bg
    docker exec $EXEC_TTY "$CONTAINER" bash -lc "$ENV
      source $ROBOT_INSTALL/setup.bash
      ros2 launch manip_sim sim.launch.py fixed_base:=true viewer:=true headless:=false $INIT_ARGS
    "
    ;;
  fsm)
    need_container
    docker exec $EXEC_TTY "$CONTAINER" bash -lc "VIEWER=1 SIM_INIT_X=${SIM_INIT_X:-} SIM_INIT_Y=${SIM_INIT_Y:-} SIM_INIT_Z=${SIM_INIT_Z:-} SIM_INIT_YAW=${SIM_INIT_YAW:-} SIM_HANG_Z=${SIM_HANG_Z:-1.00} /thinker/tools/sim/run_fsm.sh"
    ;;
  fsm-bg)
    need_container
    docker exec -d "$CONTAINER" bash -lc "VIEWER=0 SIM_INIT_X=${SIM_INIT_X:-} SIM_INIT_Y=${SIM_INIT_Y:-} SIM_INIT_Z=${SIM_INIT_Z:-} SIM_INIT_YAW=${SIM_INIT_YAW:-} SIM_HANG_Z=${SIM_HANG_Z:-1.00} /thinker/tools/sim/run_fsm.sh > /tmp/sim_fsm.log 2>&1"
    sleep 5
    echo "FSM 后台启动（无窗口），日志: docker exec $CONTAINER tail -f /tmp/sim_fsm.log"
    ;;
  bridge-bg)
    need_container
    # 后台：无窗口（detach 后没有可用的 GLFW 上下文）
    docker exec -d "$CONTAINER" bash -lc "$ENV
      source $ROBOT_INSTALL/setup.bash
      ros2 launch manip_sim sim.launch.py fixed_base:=true viewer:=false headless:=true $INIT_ARGS \
        > /tmp/sim_bridge.log 2>&1
    "
    sleep 3
    echo "bridge 后台启动（无窗口），日志: docker exec $CONTAINER tail -f /tmp/sim_bridge.log"
    ;;
  host)
    need_container
    docker exec $EXEC_TTY "$CONTAINER" bash -lc "$ENV
      source $ROBOT_INSTALL/setup.bash
      source $THINKER_INSTALL/setup.bash
      THINKER_CONFIG_DIR=/thinker/configs/sim \
      THINKER_SKILLS_DIR=/thinker/skills \
      ros2 run thinker_skill_core thinker_host
    "
    ;;
  host-bg)
    need_container
    docker exec -d "$CONTAINER" bash -lc "$ENV
      source $ROBOT_INSTALL/setup.bash
      source $THINKER_INSTALL/setup.bash
      THINKER_CONFIG_DIR=/thinker/configs/sim \
      THINKER_SKILLS_DIR=/thinker/skills \
      THINKER_ROBOT_DIR=/thinker/robots/tienkung_dex \
      ros2 run thinker_skill_core thinker_host > /tmp/thinker_host.log 2>&1
    "
    sleep 3
    echo "host 后台启动，日志: docker exec $CONTAINER tail -f /tmp/thinker_host.log"
    ;;
  image-view)
    need_container
    PORT="${SIM_VIEW_PORT:-8090}"
    echo "相机视图: http://localhost:$PORT （Ctrl-C 退出）"
    docker exec $EXEC_TTY "$CONTAINER" bash -lc "$ENV
      python3 /thinker/tools/sim/image_view_server.py --port $PORT
    "
    ;;
  logs)
    need_container
    docker exec "$CONTAINER" bash -lc 'tail -n 40 /tmp/sim_bridge.log /tmp/thinker_host.log 2>/dev/null || true'
    ;;
  pose)
    need_container
    shift || true
    docker exec "$CONTAINER" bash -lc "$ENV
      python3 /thinker/tools/sim/sim_pose.py $*
    "
    ;;
  probe)
    need_container
    docker exec "$CONTAINER" bash -lc "$ENV
      source $ROBOT_INSTALL/setup.bash
      python3 /thinker/tools/sim/sim_probe.py --check
    "
    ;;
  selftest)
    need_container
    shift || true
    docker exec "$CONTAINER" bash -lc "$ENV
      source $ROBOT_INSTALL/setup.bash
      source $THINKER_INSTALL/setup.bash
      python3 /thinker/tools/sim/sim_selftest.py $*
    "
    ;;
  call)
    need_container
    shift || true
    docker exec "$CONTAINER" bash -lc "$ENV
      source $ROBOT_INSTALL/setup.bash
      source $THINKER_INSTALL/setup.bash
      python3 /thinker/tools/sim/sim_call.py $*
    "
    ;;
  shell)
    need_container
    docker exec $EXEC_TTY "$CONTAINER" bash -lc "$ENV
      [ -f $ROBOT_INSTALL/setup.bash ] && source $ROBOT_INSTALL/setup.bash
      [ -f $THINKER_INSTALL/setup.bash ] && source $THINKER_INSTALL/setup.bash
      cd /thinker
      exec bash
    "
    ;;
  *)
    sed -n '2,8p' "$0"
    exit 1
    ;;
esac