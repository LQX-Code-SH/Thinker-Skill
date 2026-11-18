# CODEBUDDY.md

This file provides guidance to CodeBuddy Code when working with code in this repository.

## 项目定位

部署在机器人本体上的技能库：每个技能是一个独立可装卸的 ament 包，通过 **ROS Action**（工作流主接口）与 **MCP**（LLM Agent 工具层）对外提供能力。两条通道共用统一信封 `{state, code, msg, retryable, data}` 与同一执行内核。仓库处于 M0 前后：协议、执行内核、`look_down_table` / `detect_object` 已实现，`fetch_object` 已移除（`fetch_object_mock` 仍保留为 mock）。

## 常用命令

```bash
make build                            # colcon build --base-paths core robots skills，产物在 .colcon/
make build-skill SKILL=detect_object  # 只构建单个技能
make test                             # pytest -q tests（需先 source 安装空间或设 PYTHONPATH）
make clean

# 无 ROS 环境下跑单测（仓库未提供 conftest，必须显式给出所有包路径）
PYTHONPATH=core/skill_sdk:core/skill_core:core/common:core/sensor:core/body:\
robots/tienkung_dex:\
skills/perception/detect_object/skill:skills/manipulation/look_down_table/skill:\
skills/mock/fetch_object_mock/skill:tests \
  python3 -m pytest -q tests

# 单个测试
python3 -m pytest -q tests/test_pipeline.py::test_timeout

# ROS 集成自测（FakeBody + mock 技能，无需真机）
source /opt/ros/humble/setup.bash && source .colcon/install/setup.bash
python3 tools/demo_ros.py --selftest

# 仿真（robot_sim 在 Jazzy 容器内；与真机共用话题/消息/SDK real backend）
make sim-init       # 初始化 sim/robot_sim 子模块 + 选择性 LFS
make sim-up         # 起 manip_local:v4 容器
make sim-build      # 容器内构建 robot_sim + Thinker-Skill
make sim-bridge     # 前台起 MuJoCo 桥 + 查看器窗口（需 DISPLAY）
make sim-bridge-bg  # 后台起桥，无窗口（日志 /tmp/sim_bridge.log）
# 初始位姿默认 (4.73, -2.60)（场景出生点），可覆盖：
#   make sim-bridge-bg SIM_INIT_X=4.4 SIM_INIT_Y=-1.8 SIM_INIT_YAW=20
# 当前基座位姿由桥发布：/manip/base_pose（Float64MultiArray: x,y,z,qw,qx,qy,qz）
make sim-pose       # 换算成 x/y/z + yaw（度）打印一帧
# 吊带高度（等 SWR 期间）默认 1.00，可覆盖：SIM_HANG_Z=0.98
make sim-fsm        # 带 xmigcs FSM + 遥控器 GUI 的半身模式（前台，带窗口）
make sim-fsm-bg     # 同上，后台无窗口（日志 /tmp/sim_fsm.log）
make sim-host       # 起 thinker_host（configs/sim）
make sim-probe      # 检查 /robot_state、/arm|head|waist/cmd、相机/手话题
make sim-selftest   # 进程内装配后直接调技能（SIM_SKILL 指定技能）
make sim-call SIM_SKILL=look_down_table SIM_PARAMS='{"pitch_deg":45}'  # 走 Service
make sim-image-view # 网页相机视图（容器内取流，浏览器开 http://localhost:8090）
make sim-logs       # 后台 bridge/host 日志
make sim-down

# 启动 host
ros2 run thinker_skill_core thinker_host
# 环境变量：THINKER_CONFIG_DIR（默认 configs）、THINKER_SKILLS_DIR（默认 skills，冒号分隔多路径）、
#           THINKER_ROBOT_DIR（缺省按 ament_index 解析 <model>_robot 的 share）

# 打包 / 安装技能（部署用）
tools/pack_skill.sh detect_object [输出目录]     # 生成 <skill>-<ver>-<distro>-<arch>.tar.gz
tools/install_skill.sh <pkg.tar.gz> [安装根目录]  # 装到 <root>/skills/<skill>/<version>/ 并切 current 软链
```

IK 相关测试（`tests/test_ik.py`、`test_arm_move_to_pose_via_ik`）需要 `numpy`/`scipy`/`daqp`，缺失时会收敛失败。

## 执行链路（核心架构）

```
MCP tool / Action goal / Service request
        └────────────► Host.execute(skill, params)          core/skill_core/host.py
                          └► SkillRuntime.execute            runtime.py
                               ├ 能力校验（requires ⊄ capabilities → PRECOND_CAPABILITY_MISSING）
                               ├ BodyArbiter.acquire/preempt 租约（按 head/upper/lower 区域互斥）
                               ├ skill.build_steps(ctx, params) → list[Step]
                               ├ run_pipeline(...)           pipeline.py：10Hz tick、超时、重试、进度、取消
                               └ 成功后写 skill.postcondition 到 StateStore
```

关键约定：

- **单一出口**：通道层只做 JSON 编解码与信封映射，执行逻辑只在 `runtime`。
- **msg 不由技能产生**：技能只返回 `code`（`thinker_skill_sdk.ErrorCode`），出口按 `configs/errors.yaml` 模板渲染 `msg`，缺失占位符保留原文。
- **Step.run 是 tick 函数**：每 tick 调用一次，返回 `running`/`done`/`failed`；`retryable` 决定是否消耗 `retries`；`on_failure="skip"` 可降级为警告；单次 run 超过 `tick_budget` 只告警不中断。`ctx.append_steps` 与 Pipeline 共享同一列表，可运行期追加步骤。
- **租约按能力推导区域**：`body.head`→head，`body.arm/gripper/waist`→upper，`body.base`→lower；`sensor.*` 只读能力不占租约。抢占会回调软急停。
- **取消三路归一**：Action cancel 的 `threading.Event`、`ctx.check_cancel()` 主动检查、超时，最终都收敛为 `state=cancelled` / `SYS_CANCELLED`。
- **跨技能调用走进程内**：场景技能用 `ctx.call_skill(name, params)`，不走 ROS Action，子技能各自申请租约（D-019）。
- **无热加载**：装/卸/升级技能后必须重启 `thinker_host`。

## 包布局与依赖方向

```
core/interfaces     Skill.action（通用型 goal: skill + params_json）、ControlCommand.srv
core/skill_sdk      技能作者唯一依赖：Step/StepCtx/Protocol/ErrorCode，纯 Python 无 ROS
core/common         ConfigLoader（includes 递归合并）、ErrorCatalog
core/sensor|body    仅占位，数据通道与控制器实现在各机型包内
core/skill_core     registry / runtime / pipeline / body_arbiter / state_store / frontends
robots/<model>      机型适配：create_adapter() → body + sensor；能力契约 capabilities.yaml
skills/<类别>/<技能> 两棵并列子树：skill/（colcon 包）+ mcp_interface/（平台注册壳，不构建）
```

硬边界：

- 技能只能 import `thinker_skill_sdk`（及消息包），不得 import `thinker_skill_core` / `thinker_body` 等 core 实现。能力全部经 `ctx.body` / `ctx.sensor` 注入。
- `core` 是封闭集合：新增技能类别不应改 `core`；硬件驱动/数据通道/通用工具才进 core，业务算法与词表留在技能包。
- `python/`（MCP 桥/聚合器）与 `core/` 不互相 import，目前只有 `.gitkeep`。
- `skill.yaml` 必须在 `skill/` 子目录内（ament data_files 不能引用包外路径）；Python 包目录名 = ROS 包名，不设 `src/`。

## 技能开发

技能目录结构（以 detect_object 为例）：`skill/skill.yaml`、`skill/<pkg>/plugin.py`、`skill/<pkg>/`（检测/几何算法）、`mcp_interface/{mcp_interface.py,metadata.json,requirements.txt}`、`DESIGN.md`。

- `skill.yaml` 字段：`name/version/category/min_core/entry/commands/requires/precondition/postcondition/timeout/interfaces/errors/mcp/resources`。`entry: <module>:<Class>` 指向提供 `build_steps(ctx, params)` 的类（`build_steps` 可被重入，不能依赖一次性副作用）。`resources.weights` 的目录会随离线包打到 `<ver>/weights/`。
- 技能属性（`name`/`requires`/`postcondition`/`timeout`）需与 `skill.yaml` 保持一致：registry 用 yaml 校验与展示，runtime 实际读实例属性。
- 参数来源：`ctx.params`（调用入参）+ `ctx.cfg`（`configs/robot.yaml` 的 `skills.<name>` 段，只读）。技能私有参数不要写进 `skill.yaml`。
- 安装契约：`skill.yaml` 装到 `share/<pkg>/skill.yaml` 并打 ament index marker，host 扫描发现，装了就发现、卸了就消失。
- 装配期容错：`min_core` 不满足或 `requires` 能力缺失时，该技能跳过并出现在 `skipped` 告警中，不阻塞其他技能。
- MCP 工具：`mcp_interface.py` 的函数签名与 docstring 就是 schema / LLM 描述（函数体不实现，host 注入）；工具名 = 技能名 = 目录名 = `metadata.json.functionNames`，改动需同步。`mcp_interface.py` 必须保持明文（需反射签名）。
- 部署后模型路径：技能根目录优先（`ctx.skill_dir`），再试 cwd、`THINKER_WEIGHTS_DIR`，参见 `detect_object/plugin.py:resolve_model_path`。

## 机器人接口（以 `docs/robots/dex.md` 为准）

真机的对外话题以官方文档为准；技能只经 `BodyAPI`/`SensorAPI` 访问，不直接订阅这些话题。

| 用途 | 话题 | 类型 |
|---|---|---|
| 全量状态 | `/robot_state` | `ros2_bridge_msgs/RobotState`（head/waist/leg/arm/imu） |
| 手臂控制 | `/arm/cmd` | `ros2_bridge_msgs/ArmCtrl` |
| 腰部控制 | `/waist/cmd` | `ros2_bridge_msgs/WaistCtrl` |
| 头部控制 | `/head/cmd` | `ros2_bridge_msgs/HeadCtrl` |
| 腿部控制 | `/leg/cmd` | `ros2_bridge_msgs/LegCtrl` |
| 因时手 | `/left|right_hand/{angle_set,angle_actual,force_set,force_actual,speed_set,touch_data}` | `inspire_hand_msgs` |
| 强脑手 | `/left|right_hand/{set_motor_single,set_motor_multi,motor_status,touch_status}` | `brainco_hand_msgs` |
| 头/腰相机 | `/ob_camera_{head,waist}/{color,depth}/{image_raw,camera_info,metadata}` | `sensor_msgs` |
| 雷达 / IMU | `/livox/lidar`、`/livox/imu` | `sensor_msgs` |
| 遥控器 | `/sbus_data`、`/sbus_data/event` | `sensor_msgs/Joy`、`bodyctrl_msgs` |
| 电源 / 灯效 | `/power/board/{status,ctrl,key_status}`、`/power/battery/status`、`/xsys/light/ctrl` | `bodyctrl_msgs` |

控制消息用 `mode` 区分：`0` 位置模式（`pos/spd/cur`），`1` 力位混合（`kp/kd/pos/spd/tor`）；
`ctrl[].name` 是电机 id（头 1/2、臂 11-17/21-27、腰 31-33、腿 51-66）。

仿真侧约定：

- `backend: robosim` 的适配器在 `robots/tienkung_dex/tienkung_dex_robot/robosim/`，把 robot_sim 的桥内话题
  （`/arm|waist|head/cmd_ctrl`、`/arm|waist|head/status`）翻译成同一套 `BodyAPI`/`SensorAPI`；
  话题与增益只在 `robots/tienkung_dex/resources/sim_mapping.yaml` 里改，不动技能。
- robot_sim 已发布 `/robot_state` 并接受 `/arm|head|waist|leg/cmd`（类型为 `ros2_bridge_msgs`，副本在 `sim/robot_sim/src/ros2_bridge_msgs`）；宿主机适配器仍走 `cmd_ctrl`，等厂商 SDK 可用后可切 real backend 并删除 robosim 适配器。
- `/robot_state` 由 4 路 `status` + `/imu/status` 聚合，默认 100 Hz；`/…/cmd` 只按 mode 0/1 的位置/力位混合语义执行，HBWALK 下忽略 `/leg/cmd`。
- 官方未定义相机位姿话题；仿真额外发布 `/ob_camera_head/pose`（pelvis 系），供 `transform_to_base` 用，真机仍走 URDF 正运动学。
- 头部符号：URDF 低头为正，MJCF 低头为负，映射在 `sim_mapping.yaml` 的 `head.pitch_sign`。
## 机型适配

`configs/robot.yaml` 的 `model` 决定 `import <model>_robot` 并调用 `create_adapter(node, backend, hand_variant, ...)`（`core/skill_core/app.py:load_robot_adapter`）。`robots/<model>/capabilities.yaml` 声明基础能力 + `hand_variants`（brainco/inspire/fixed），装配期按 `hand_variant` 合并后用于技能 `requires` 子集校验；`fixed` 假手没有 `body.gripper`。`robot_dir` 显式路径不存在时走 ament_index，机器人上无需手配路径。IK（QP+DAQP）属于机型包内部实现（D-017）。仿真能力差异只由 `capabilities.yaml` 的 `backend_variants.robosim` 表达，不新增 `robots/*_sim` 机型；`backend: robosim` 走 `tienkung_dex_robot/robosim/` 适配器（复用同一 `TienkungDexBody`/`TienkungDexSensor`，只换话题层），因此不需要 SDK 也能跑。仿真配置在 `configs/sim`，子模块在 `sim/robot_sim`，不参与默认 `make build`。

## 部署布局与发现

```
<root>/skills/<skill>/<version>/{install/, mcp_interface/, weights/}
<root>/skills/<skill>/current -> <version>
```

`discover()` 一旦发现 `*/current` 或 `*/*/current` 就只扫这些目录（避免多版本重复发现）；`host_main` 会把 `<version>/install/*/lib/python3.*/site-packages` 注入 `sys.path`。`skill_root()` 从 `skill.yaml` 向上回溯到含 `mcp_interface/mcp_interface.py` 的目录。

## 测试

- 全部单测走 `tests/fakes.py`（FakeBody/FakeSensor/FakeState + 可注入假时钟），不依赖 ROS 与真机；通过 `make_ctx()` 构造 `StepCtx`。
- 通道测试用纯逻辑函数 `frontends/action.py:handle_goal`、`frontends/service.py:handle_request`，避免导入 ROS 类型。
- 技能测试直接 import 已构建/已加 PYTHONPATH 的技能包（如 `from look_down_table.plugin import LookDownTable`），因此不构建时至少要补 `skills/*/*/skill` 到 PYTHONPATH。

## 多人协作与 Git 提交规则

**分支与合并**

- `main` 始终可构建、可跑通 `make test`；不直接 push，所有改动走 GitLab MR。
- 分支名 `<type>/<简短主题>`，如 `feat/sim-submodule`、`fix/lease-expiry`、`docs/git-rules`；合并后删除。
- 开工前 `git pull --rebase origin main`；MR 用 **Squash** 合入，保持 `main` 单线历史；合并后本地 `git pull --ff-only`。
- 只对自己的未共享分支用 `--force-with-lease`；`main` 禁止 force push。

**提交信息**：`<type>(<scope>): <中文摘要>`

- type：`feat` / `fix` / `refactor` / `perf` / `test` / `docs` / `build` / `chore` / `revert`
- scope 取改动所在层或模块：`interfaces`、`skill_sdk`、`skill_core`、`frontends`、`common`、`robots/<model>`、`skills/<skill>`、`deploy`、`configs`、`docs`、`tests`；跨层改动可省略。
- 摘要 ≤ 50 字，动宾结构，句末不加句号；正文写「为什么改 + 影响面」，每行 ≤ 72 字。
- 契约类改动（`Skill.action`、`StepCtx`/`Step`、`skill.yaml` 字段、`errors.yaml`、`capabilities.yaml`）必须在正文写 `Refs: D-0xx`，并同步《决策记录》《接口文档》与技能 `DESIGN.md`（D-018）。
- 破坏性改动加 `BREAKING CHANGE:` 脚注；关联 issue 写 `Closes #<id>`。

**提交粒度与内容**

- 一个提交只做一件事；格式化、重命名不与行为改动混提。
- 不提交构建产物、模型权重与缓存：`.colcon/`、`build/`、`install/`、`log/`、`dist/`、`.pack/`、`weights/`、`__pycache__/`。
- 生成物（如 `tools/scaffold_packages.py` 的产物）与手写代码分开提交。
- 使用本人 Git 身份提交，不共用账号；`user.name` / `user.email` 与 GitLab 账号一致。

**MR 要求**

- 标题与 squash 后的提交信息一致；描述包含：改了什么 / 怎么验证（贴命令与结果）/ 影响面（契约、技能版本、配置、文档）。
- 至少 1 人 review；`make test` 通过（无 ROS 时用上文 PYTHONPATH 命令）；涉及 ROS 或真机的改动注明验证环境（FakeBody / 仿真 / 真机）。
- 技能行为或契约变化时同步升 `skill.yaml` 的 `version`；`core` 不兼容变更调整 `min_core` 边界。
- 不合并 WIP 或测试未通过的 MR；review 意见用新提交回应，确需改写自己分支时用 `--force-with-lease` 并在 MR 中说明。

## 文档与决策

`docs/` 四份设计文档：概要设计、详细设计、接口文档、决策记录；仿真操作与排障见 `docs/仿真说明.md`。**代码注释里的 D-00x 指《决策记录》条目；文档与代码冲突时以《决策记录》为准。** 修改契约/接口时同步更新对应文档与技能 `DESIGN.md`（D-018）。
