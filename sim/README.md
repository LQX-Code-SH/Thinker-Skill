# robot_sim 仿真子模块

本目录以 git submodule 引入 [robot_sim](https://gitlab.ubtrobot.com/tienkung/robot_sim)，
用于在 MuJoCo 中运行 Thinker-Skill 的技能。它**不参与**默认 `make build`：

- robot_sim 是 ROS 2 **Jazzy** + MuJoCo 工作区；
- Thinker-Skill 宿主机当前是 Humble，两者 DDS 不可靠；
- 仿真与 `thinker_host` 必须在同一个 `manip_local:v4` 容器内运行。

## 初始化

```bash
make sim-init
```

等价于：

```bash
GIT_LFS_SKIP_SMUDGE=1 git submodule update --init --depth 1
git -C sim/robot_sim lfs pull --include "src/manip_description/assets/**"
```

全量 LFS 约 1 GB；这里只拉机械臂/场景资产（约 283 MB），不拉 655 MB 的
`docker/manip_local-v4.tar.zst` 和 wheel。需要容器镜像时直接使用本机已有的
`manip_local:v4`，或用 robot_sim 的 `docker/` 脚本单独加载。

## 前置条件（缺一不可）

| 条件 | 检查 | 说明 |
|---|---|---|
| 镜像 | `docker image inspect manip_local:v4` | Jazzy + MuJoCo，本机已有 |
| 子模块接口 | `make sim-probe` 全 OK | robot_sim 必须与真机 **同话题/同消息/同语义** |
| SDK 可见 | `make sim-shell` 后 `python3 -c "import tienkung_dex"` | 镜像不带 SDK，需挂载，见下 |

镜像里没有 `tienkung_dex` 与 `ros2_bridge_msgs`。把已构建好的 SDK 工作空间挂进来：

```bash
XHUMANOID_SDK_DIR=/path/to/xhumanoid_sdk_ws make sim-up
# 容器内挂到 /opt/xhumanoid_sdk；env.sh 会 source 其中的 install/setup.bash
```

不提供该变量时，robot_sim 桥和相机仍能跑（`sim-probe` 的相机项 OK），
但 `thinker_host` 会在装配期报 `tienkung_dex SDK 不可用`。

子模块接口不对时，`sim-probe` 会报 `/robot_state: MISSING`。当前 pin `f7570ee`
仍是 `/arm|waist|leg/cmd_ctrl`，需换成官方兼容真机接口的分支/commit：

```bash
git -C sim/robot_sim fetch origin
git -C sim/robot_sim checkout <官方兼容 ref>
git add sim/robot_sim && git commit -m "chore(sim): 更新 robot_sim 到真机兼容版本"
```

## 脚本一览

| Make 目标 | 实际脚本 | 作用 |
|---|---|---|
| `make sim-init` | git submodule + LFS | 拉子模块与 `manip_description` 资产（一次性） |
| `make sim-up` | `tools/sim/container.sh up` | 起 `manip_local:v4` 容器，仓库挂到 `/thinker` |
| `make sim-build` | `container.sh build` | 容器内先建 robot_sim，再建 Thinker-Skill |
| `make sim-probe` | `container.sh probe` | 检查真机兼容话题与相机帧 |
| `make sim-bridge` | `container.sh bridge` | 前台起桥（`fixed_base` 降级，只看机械臂） |
| `make sim-bridge-bg` | `container.sh bridge-bg` | 同上，后台无窗口（日志 `/tmp/sim_bridge.log`） |
| `make sim-fsm` | `container.sh fsm` → `run_fsm.sh` | 前台起 FSM + 遥控器 + 桥，带窗口 |
| `make sim-fsm-bg` | `container.sh fsm-bg` | 同上，后台无窗口（日志 `/tmp/sim_fsm.log`） |
| `make sim-host` / `sim-host-bg` | `container.sh host` | 起 `thinker_host`（前台 / 后台） |
| `make sim-selftest` | `container.sh selftest` | 进程内装配后直接调技能 |
| `make sim-call` | `container.sh call` | 走 ROS Service 调技能（真实调用链） |
| `make sim-logs` | `container.sh logs` | 看后台 bridge/host 日志 |
| `make sim-shell` | `container.sh shell` | 进容器（已 source Jazzy 与工作空间） |
| `make sim-down` | `container.sh down` | 删容器，不动挂载目录 |

`container.sh` 内两套安装树：robot_sim 在 `/thinker/.colcon-sim/robot-install`，
Thinker-Skill 在 `/thinker/.colcon-sim/install`，互不覆盖。

## 看仿真画面（MuJoCo 查看器）

只有两个命令：**前台看画面，后台跑自动化**。窗口只能在前台打开
（`docker exec -d` 没有可交互 GLFW 上下文）。

```bash
# 宿主机需要有 X（DISPLAY 指向可用显示，/tmp/.X11-unix 有对应 socket）
echo $DISPLAY              # 例如 :1

make sim-up                # up 会带 -e DISPLAY 并挂 /tmp/.X11-unix
make sim-bridge            # 前台 + 查看器窗口
make sim-bridge-bg         # 后台无窗口，日志 /tmp/sim_bridge.log
```

启动成功的日志：

```text
[xmigcs_manip_bridge]: 已打开 MuJoCo 查看器
[xmigcs_manip_bridge]: 模型 custom_kitchen_living_robot.xml: ... fixed_base=True
```

常见现象：

| 日志 | 含义 |
|---|---|
| `MESA: error: Failed to query drm device` / `failed to load driver: iris` | 走了软件渲染，不影响窗口与物理 |
| `打开查看器失败（没有显示器？改用 --headless）` | 容器里没有可用 DISPLAY；检查 `make sim-up` 时的 `DISPLAY` 与 socket |
| 有窗口但画面不动 | 桥在等 `wait_state=SWR`，或物理被吊带挂住；`fixed_base:=true` 时属正常降级 |

## 机器人位姿与初始位置

桥按状态频率发布基座位姿（世界系）：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/manip/base_pose` | `std_msgs/Float64MultiArray` | `[x, y, z, qw, qx, qy, qz]`（米 / 四元数） |

没有单独的 yaw 字段，用下面的命令直接看：

```bash
make sim-pose                  # 打印一帧：x/y/z + yaw/roll/pitch（度）
make sim-pose SIM_POSE_ARGS=""  # 持续打印
```

手算：`yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))`。

场景默认出生点是 **(4.73, -2.60, 0.988)**，不是原点；只改朝向时要同时给 x/y。

```bash
# 使用默认出生点（Makefile 已填好实测值）
make sim-bridge-bg

# 指定初始位置与朝向（世界系，米 / 度）
make sim-bridge-bg SIM_INIT_X=4.40 SIM_INIT_Y=-1.80 SIM_INIT_YAW=20

# 也可只改朝向，位置沿用默认值
make sim-fsm SIM_INIT_YAW=90

# 直接调 launch 时参数为空表示“保持场景默认”
ros2 launch manip_sim sim.launch.py fixed_base:=true init_x:=4.4 init_y:=-1.8 init_yaw_deg:=20
```

`SIM_INIT_Z` 留空表示用反解出的站姿骨盆高度（≈0.99 m）；只有需要吊高/压低时才显式给值。
FSM 模式等 SWR 期间的吊带高度用 `SIM_HANG_Z`（默认 1.00，原来 1.03 偏高）：

```bash
make sim-fsm SIM_HANG_Z=0.98
```
初始位姿在启动时应用一次，`backend.reset()`（桥的复位服务）后会重新应用。
## 带 FSM 和遥控器（半身模式）

`sim-bridge` 是 `fixed_base:=true` 的降级模式，只验证机械臂；要走路、保持平衡，必须用 FSM 模式。

```bash
make sim-up
make sim-build
make sim-fsm          # 前台：桥 + 查看器 + teleop GUI + xmigcs FSM + 摇杆桥
# 另一个终端：
make sim-host
```

`make sim-fsm` 做的事（对应 robot_sim 的 `scripts/run_fsm_sim.sh`）：

| 组件 | 作用 |
|---|---|
| `ros2 launch manip_sim sim.launch.py fixed_base:=false viewer:=true wait_state:=SWR` | 物理桥，先吊在 1.03 m 等 FSM |
| `xmigcs_sim` | 厂商运控 FSM（HBWALK 平衡） |
| `xmigcs_joystick` | 遥控器桥：`/sbus_data` → `/hric/robot/*` |
| `teleop_gui` | 遥控器 GUI（需要 `DISPLAY` + tkinter），日志 `/tmp/teleop_gui.log` |

进入半身模式的顺序（与真机一致）：

```text
gotoZERO  -> STANDZERO
gotoSTART -> SWR        # 桥在这一步摘吊带、放开物理
gotoHBWALK-> HBWALK     # 双腿交给运控，双臂/腰交给技能
```

日志应出现：

```text
FSM: None -> STOP
FSM: STOP -> STANDZERO
FSM: STANDZERO -> SWR
FSM 已进入 SWR，摘吊带放开物理
t= 11.3s z=0.989m ... rtf=1.00
```

`make sim-fsm` 已经包含桥，**不要再叠加 `make sim-bridge`**；后台跑用 `make sim-fsm-bg`
（无窗口，遥操作用不到 GUI，日志 `/tmp/sim_fsm.log`）。
## 启动顺序

**必须 bridge 先起、host 后起**：SDK 的 `robot.start()` 依赖 `/robot_state` 等真机接口，
桥没起来 host 会卡在装配期。

```bash
make sim-init                     # 仅首次
make sim-up
make sim-build                    # robot_sim + Thinker-Skill

# 终端 A：MuJoCo 桥（可换 make sim-bridge-bg 落后台）
make sim-bridge

# 终端 B：确认接口齐了
make sim-probe                    # /robot_state, /arm|head|waist/cmd, 相机、手

# 终端 C：host（可换 make sim-host-bg）
make sim-host
# 日志出现 thinker_host ready: ... skills=[...] 即可调用
```

后台模式用 `make sim-logs` 看两边日志；结束用 `make sim-down`。

`configs/sim/sim.yaml` 使用 `model: tienkung_dex`、`backend: robosim`、`hand_variant: inspire`；
仿真和真机共用 `robots/tienkung_dex` 的关节表、URDF、IK、手实现与能力契约，
能力差异只在 `capabilities.yaml` 的 `backend_variants.robosim`（去掉 `body.base`、`sensor.imu` 等）。

话题层由 `tienkung_dex_robot/robosim/` 适配器翻译：桥内 `/arm|waist|head/cmd_ctrl` +
`/status`，相机 `/ob_camera_head/*`（内参取 `camera_info`），手 `/left|right_hand/*`。
对外口径仍是 `docs/robots/dex.md` 的 `/robot_state`、`/arm|head|waist/cmd`——等 robot_sim
对齐后可退回 SDK real backend。头部符号与限位在 `resources/sim_mapping.yaml`
（`head.pitch_sign: -1`，MJCF 低头为负）。

## 测试分级

| 级别 | 跑在哪 | 命令 | 验证什么 |
|---|---|---|---|
| L0 单测 | 宿主机 Humble，无 ROS | `make test` | 技能算法、Pipeline、通道纯逻辑、适配器纯函数 |
| L1 接口 | 容器内，bridge 在跑 | `make sim-probe` | 真机兼容话题齐全、相机有帧 |
| L2 装配 | 容器内，bridge 在跑 | `make sim-selftest` | host + 适配器 + 技能能否装配并执行 |
| L3 调用 | 容器内，bridge+host 在跑 | `make sim-call` | Service → Host → Pipeline → Step 全链路信封 |

L0 不需要仿真，改技能逻辑先跑它；L1/L2 确认环境；L3 才是“和调用方一样”的验证。

## 测试例子：低头看桌面

`look_down_table` 只依赖 `body.head`，是最短的端到端样例：

```bash
# L3：走 Service，和外部调用方完全一致
make sim-call SIM_SKILL=look_down_table SIM_PARAMS='{"pitch_deg": 45}'
```

期望输出：

```text
state=succeed code=OK retryable=False
msg=...
data={"pitch_deg": 45.0, "requested_pitch_deg": 45.0, "actual_pitch_deg": 44.9}
```

判据：`state=succeed`；`actual_pitch_deg` 与目标差值在 `configs/sim/sim.yaml`
的 `tol_deg`（1.0°）内。肉眼可见机器人头部转到约 45°。

单测层面的同款例子（不需要仿真）：

```bash
PYTHONPATH=core/skill_sdk:core/skill_core:core/common:core/sensor:core/body:\
robots/tienkung_dex:skills/manipulation/look_down_table/skill:tests \
  python3 -m pytest -q tests/test_look_down_table.py
```

其它技能：

```bash
make sim-call SIM_SKILL=fetch_object_mock SIM_PARAMS='{}'   # 机械臂 + 因时手，验证 holding
make sim-selftest SIM_SKILL=detect_object                  # 需要权重，见 configs/sim/sim.yaml
```

## 排障

| 现象 | 原因 / 处理 |
|---|---|
| `服务未就绪: /thinker/skill/...` | host 没起，或技能被跳过；看 `make sim-logs` 的 `skipped=` |
| `sim-probe` 报 MISSING | robot_sim pin 与真机接口不一致，换官方兼容 commit 后 `git add sim/robot_sim` |
| host 卡在装配、无 `ready` | 桥没起或 `/robot_state` 没数据，先 `make sim-bridge` |
| `actual_pitch_deg` 偏差大 | 头部符号/零点或 `max_pitch_deg` 配置与模型不符 |
## 版本

- 分支：`v3`
- 基线 commit：`f7570ee80e28c7c94d6b4c47a5506641420cb0b0`
- 前提：robot_sim 必须与真机 SDK **同话题、同消息类型、同语义/QoS**：
  `/robot_state`、`/arm|head|waist/cmd`、`/ob_camera_*`、`/left|right_hand/*`。
- `backend: robosim` 只是复用 SDK real backend，不再维护私有 `/head/cmd_ctrl`。
- 如果官方兼容版本不是 `f7570ee`，以官方文档/分支为准更新 pin；本仓库不添加私有话题补丁。

## 红线

- 不要把 `sim/robot_sim/src` 加进默认 `make build` 的 `--base-paths`。
- 不要在宿主机 Humble 里直接 `ros2 launch` robot_sim；必须在 Jazzy 容器内。
- 子模块内改动先走 robot_sim 仓库的 MR，再更新本仓库的 gitlink。
