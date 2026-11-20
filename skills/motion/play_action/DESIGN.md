# play_action

固定动作播放：把录制端导出的动作文件（`schemaVersion: 2` JSON）在真机上回放成关节轨迹。

| 项 | 内容 |
|---|---|
| 类别 | `motion`（不感知、不操作物体，只回放关节轨迹） |
| 依赖能力 | `body.arm.left`、`body.arm.right`、`body.waist`、`body.head`（占 `head` + `upper` 区域） |
| 可选能力 | `body.gripper.left/right`（无手或 `play_hands=false` 时自动跳过手指） |
| 参数 | `action`（必填）、`clip`、`speed`、`transition_s`、`play_hands`、`hand_mode` |
| 步骤 | 读取动作 → 姿态衔接 → 播放动作 |
| 后置状态 | 无（不写语义状态） |
| 私有配置 | `configs/robot.yaml` 的 `skills.play_action` 段 |

## 为什么只驱动上半身（D-024）

SDK 明确「腿由运控（locomotion policy）持有，禁止关节级下发」，`BodyAPI` 也没有腿接口。
动作文件里必然带 `hip_* / knee_* / ankle_*` 轨迹，本技能**一律忽略**，只在返回 `data`
里给出 `ignored_joints` 与 `leg_motion_rad`（腿轨迹幅度），便于确认"这条动作是否需要腿"。

忽略腿的副作用是：动作中若包含下蹲/迈步，上半身在空间中的绝对位置不会跟着变，只在
当前站姿上做上身动作。需要走动时由上层先调用导航/运控技能。

## 起步衔接（与当前姿态）

不做衔接直接下发首帧会瞬间跳变；先停稳再播又会多一次加减速。做法是 **crossfade**：

```
target(t) = cur + smoothstep(t / T) · (traj(t) − cur)
```

- `cur`：解析动作时读到的当前姿态（臂 `get_joints()`、腰 `get_pose()`、头 `get_pose()`；
  手按口径取 `get_positions()` 的 6 维归一化值或 `get_joint_positions()` 的 13 维 rad）；
- `traj(t)`：动作文件在 `t` 时刻的采样值，`t` 从片段起点开始走；
- `T = transition_s`（默认 2s，`0` 表示直接跳到首帧）。

`t=0` 时 `s=0` → 精确等于当前姿态（无跳变）；`t=T` 时 `s=1` 且 `s'=0` → 位置与速度
都连续。读不到当前值的关节退化为"直接用首帧"。

## 机型与手型（hand_mode）

技能本身不 import 机型包，机型差异通过 `BodyAPI`、手型差异通过 `hand_mode` 显式区分：

- **臂/腰/头**：沿用 `tienkung_dex` 的 URDF 关节名映射（`action_file.ARM_URDF_TO_BODY` 等），
  换机型时靠同名关节匹配；`prepare` 读动作文件 `robot.name`，不含 `tienkung` 时在
  `data.warnings` 里提示"机型不一致、仅按同名关节回放"，不静默通过；
- **手型**：`hand_mode` = `auto | 6d | 13d`，`auto` 按机型能力自动选：
  - `13d`：要求 `gripper.hand_format == "inspire13"` 且提供
    `set_joint_positions(dict)` / `get_joint_positions()`（因时 RH56 13 通道）。
    文件里的 URDF 关节角（rad）直接下发，不经过 6 维上采样；
  - `6d`：强脑 6 电机手，或显式要求兼容口径。按文件自带 `robot.joints` 限位把
    `thumb_mcp/thumb_yaw/index_mcp/middle_mcp/ring_mcp/pinky_mcp` 归一化成
    `0=张开 / 1=闭合`，走 `GripperAPI.set_positions`；
  - 动作文件没有对应手部轨道时该侧退化为 `none`（不下发）；显式 `13d` 在非因时手
    或文件缺 13 维轨道时直接 `INPUT_INVALID_PARAMS`，不静默降级；
- 13 维通道表与角码换算（零位/方向/行程）**留在机型包**
  （`robots/tienkung_dex/.../inspire_hand.py` 的 `INSPIRE_JOINT_TO_CHANNEL`），
  技能只传 `{urdf 关节名: rad}`，避免复制硬件常数；
- `data.hand_mode` / `hand_plan` 报告实际口径（`13d` / `6d` / `mixed` / `none`），
  上层可据此判断本次是否真的按 13 维回放。

## 采样与下发

- 关键帧插值：`interpolation=step/hold` 保持、`linear` 线性、其余（含 `auto`）用
  **三次 Hermite**，切线优先取关键帧 `inHandle/outHandle`（`dy/dxMs`），没有手柄时用
  相邻点割线；
- 每 tick 幂等下发一次：臂 `move_to_joints`、腰 `set_pose`、头 `look_at`；
  手按 `hand_mode` 走 `set_positions`（6d）或 `set_joint_positions`（13d）；
- 双臂必须**合并成一条 `ArmCtrl`**：真机控制桥每个周期只消费一条手臂命令，
  分侧各发一条时后一条会被丢弃（实测同 tick 先左后右，右臂完全不动）。
  机型包用 `body.move_arms({side: targets})` 一次下发 14 个关节；技能在缺少
  该接口时退回逐侧下发（其它机型兼容）；
- 手指 6d：文件里 13 个 URDF 手关节按**文件自带 `robot.joints` 限位**归一化成 6 维
  `0=张开 / 1=闭合`（`thumb_pitch/thumb_roll/index/middle/ring/little`），`*_dip_joint`
  为跟随关节不单独下发；该 6 维口径与机型 `GripperAPI.set_positions` 一致；
- 手指 13d：直接采样 13 个非 mimic 关节（`pinky_mcp/ring_mcp/middle_mcp/middle_yaw/
  index_mcp/index_yaw/pinky_pip/ring_pip/middle_pip/index_pip/thumb_yaw/thumb_mcp/
  thumb_pip`）的 rad，未出现在文件里的通道由机型包保持当前角码；起始 crossfade 用
  `get_joint_positions()` 读回当前角，因此因时手不再出现"手指首帧跳变"；
- 时间精度受 `tick_rate` 限制（默认 10Hz → 100ms 一个目标点），动作本身 30fps。
  需要更平滑时调大 `configs/robot.yaml` 的 `tick_rate`；
- 文件里的 `loop` **不生效**：一次调用只播一遍，播完停在末帧；重复播放由上层再次调用。

## 动作文件解析

`skill/play_action/action_file.py`（纯 Python，无 ROS，可离机单测）：

- `schemaVersion` 必须为 `2`，否则 `INPUT_INVALID_PARAMS`；
- `clip` 参数按 **名称或 id** 匹配，留空用 `activeClipId`，仍找不到用第一个片段；
- 时间轴取 `playRange.startMs/endMs`（按 `durationMs` 夹紧），`speed` 只影响墙钟；
- 未知关节 / 腿关节只记录不动作；文件里一个可驱动关节都没有时直接失败。

## 动作文件放哪

按顺序解析 `action`：

1. 动作清单里注册的 key / `display_name` / `aliases`（见下节）；
2. 绝对路径（或用 `./`、`../` 的相对路径）；
3. `skills.play_action.actions_dir`；
4. `<技能根>/actions/<name>.json`（源码树 = `skills/motion/play_action/actions/`；
   部署后 = `<root>/skills/play_action/<ver>/actions/`）；
5. `$THINKER_ACTIONS_DIR`；
6. 当前工作目录。

无后缀自动补 `.json`。清单缺失或 `action` 未注册时，完全回退到 2~6 的路径搜索（旧行为）。

**技能根目录的 `actions/` 会随离线包一起发布**：`pack_skill.sh` 把它拷到包内根目录，
`install_skill.sh` 解压后即 `<root>/skills/play_action/<ver>/actions/`，正好是运行期的
`<skill_dir>/actions`。所以自带动作直接调用 `action: goready` 即可。

现场新增/替换动作有两种方式，都不需要重装技能（每次调用时现读文件）：

- 拷到 `<root>/skills/play_action/<ver>/actions/`（`current` 软链目录下）；
- 或放到任意目录，配 `skills.play_action.actions_dir` 或 `$THINKER_ACTIONS_DIR`。

约定**一个动作一个文件**：一个 JSON 只保留一个片段，并在 `actions.yaml` 里登记一条
对应项（详见下节）。

只改动作文件不需要重启 `thinker_host`；只有装/卸/升级技能包才需要重启（无热加载）。

## 动作清单（actions.yaml）

`actions/actions.yaml` 是可选的声明式索引，用来把"动作名/别名"映射到文件，并声明
适用机型、末端类型、默认参数与所需能力。**它只做选择与校验，不改变执行路径**；
没有清单时行为与旧版本完全一致。

```yaml
version: 1
actions:
  goready:
    file: goready.json          # 必需；相对 actions_dir / <skill>/actions / cwd
    clip: goready               # 可选：默认片段（调用参数优先）
    display_name: 就绪
    aliases: [ready]            # 别名；display_name 本身也可查找
    tags: [greeting]
    models: [tienkung_dex]      # 空 = 不限机型
    hand_variants: [inspire, brainco, fixed]   # 空 = 不限末端
    hand_mode: auto             # 默认手部口径：auto | 6d | 13d
    defaults: {speed: 1.0, transition_s: 2.0}
    requires: [body.arm.left, body.arm.right, body.waist, body.head]
    on_mismatch: strict         # strict（默认，失败）| warn（放行并写 warnings）
    verified: {env: real, date: 2026-11-18}
    safety: {max_speed: 1.5}    # 预留元数据，本技能暂不解释
```

查找与优先级：

- 清单位置：`actions_dir` → `<技能根>/actions` → `<技能根>` → `$THINKER_ACTIONS_DIR`，
  取第一个存在的 `actions.yaml`；
- 参数优先级：**调用参数 > 动作清单 > `configs/robot.yaml` > 内置默认值**；
- 机型/末端来源：`body.model`（如 `tienkung_dex`）与 `body.hand_variant`
  （`inspire` / `brainco` / `fixed`，缺失时按 `gripper.hand_format` 推断）；
- 校验结果分三档：`ok`（全部匹配）、`warn`（缺标识无法确认，或 `on_mismatch: warn`
  放行）、`fail`（机型/末端/能力明确不匹配，`INPUT_INVALID_PARAMS`）；
- 校验项：`models`、`hand_variants`、`hand_mode: 13d` 与末端是否自洽、`requires`
  是否具备（`sensor.*` 交给装配期，技能侧不重复判定）；
- 清单存在但 `action` 未注册：回退路径搜索，并在 `warnings` 里提示"未注册"；
- 返回值新增 `registry`、`registry_key`、`display_name`、`tags`、`compat`、
  `hand_variant`，便于上层/日志定位。

**组织约定：一个动作一个文件。** 每个 JSON 只保留一个片段，`activeClipId` 指向它，
清单里的 `clip` 可省略；文件内仍有多片段时照常按 `clip`/`activeClipId` 播放，但
`prepare` 会在 `warnings` 里提示不符合约定（避免一个文件里混入多条动作后难以检索和
上线）。新增动作 = 新增一个 JSON + 一条清单项，不改其它动作文件。

内置清单随 `pack_skill.sh` 一起发布（`cp -a actions`），现场改清单同样不需要重装技能。
清单与文件是两处真相：`file` 必须与磁盘同步，缺失时按"动作文件不存在"失败并带上
`registry_key`。

## 失败语义

| code | 场景 |
|---|---|
| `INPUT_INVALID_PARAMS` | 缺 `action`、文件不存在、版本不是 2、clip 不存在、无可驱动关节、`hand_mode` 非法、显式 `13d` 但手型/文件不支持、动作清单非法、机型/末端/能力不匹配 |
| `PRECOND_CAPABILITY_MISSING` | 装配期已判定（缺臂/腰/头能力，技能直接不进 registry） |
| `PRECOND_BODY_BUSY` | 上臂/腰/头被其它技能占用（租约冲突） |
| `SYS_TIMEOUT` | 动作时长 + 衔接超过 `timeout`（300s）或看门狗兜底 |
| `SYS_INTERNAL_ERROR` | 机型下发异常（如手部未启动） |

## 已知限制

- 不做碰撞/自碰撞检查，动作文件需录制端保证可达、不撞身；
- 只做关节空间回放，不平滑掉超速；速度由机型 `move_to` 的 `spd` 参数与 tick 目标共同决定；
- 手部只有 `13d` 是逐通道无损回放；`6d`（含 brainco、以及因时手显式降级）是有损映射，
  侧摆/指中细节丢失，`*_dip_joint` 在所有模式下都不单独下发；
- 机型适配仅按同名关节匹配：动作文件 `robot.name` 只做软校验，非 `tienkung` 机型若关节命名
  不同，可能只下发部分关节（`ignored_joints` 会列出未识别项）；
- 动作清单只校验机型/末端/能力声明，不校验动作本身在该机型上是否可达、是否安全；
  清单与动作文件是两处真相，需一起维护；
- 腿轨迹一律忽略，见上文。
