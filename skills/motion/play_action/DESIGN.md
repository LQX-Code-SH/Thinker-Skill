# play_action

固定动作播放：把录制端导出的动作文件（`schemaVersion: 2` JSON）在真机上回放成关节轨迹。

| 项 | 内容 |
|---|---|
| 类别 | `motion`（不感知、不操作物体，只回放关节轨迹） |
| 依赖能力 | `body.arm.left`、`body.arm.right`、`body.waist`、`body.head`（占 `head` + `upper` 区域） |
| 可选能力 | `body.gripper.left/right`（无手或 `play_hands=false` 时自动跳过手指） |
| 参数 | `action`（必填）、`clip`、`speed`、`transition_s`、`play_hands` |
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

- `cur`：解析动作时读到的当前姿态（臂 `get_joints()`、腰 `get_pose()`、头 `get_pose()`、
  手 `get_positions()` 的 6 维归一化值）；
- `traj(t)`：动作文件在 `t` 时刻的采样值，`t` 从片段起点开始走；
- `T = transition_s`（默认 2s，`0` 表示直接跳到首帧）。

`t=0` 时 `s=0` → 精确等于当前姿态（无跳变）；`t=T` 时 `s=1` 且 `s'=0` → 位置与速度
都连续。读不到当前值的关节（如手部接口返回 13 维原始角码）退化为"直接用首帧"。

## 采样与下发

- 关键帧插值：`interpolation=step/hold` 保持、`linear` 线性、其余（含 `auto`）用
  **三次 Hermite**，切线优先取关键帧 `inHandle/outHandle`（`dy/dxMs`），没有手柄时用
  相邻点割线；
- 每 tick 幂等下发一次：臂 `move_to_joints`、腰 `set_pose`、头 `look_at`、手 `set_positions`；
- 手指：文件里 13 个 URDF 手关节按**文件自带 `robot.joints` 限位**归一化成 6 维
  `0=张开 / 1=闭合`（`thumb_pitch/thumb_roll/index/middle/ring/little`），`*_dip_joint`
  为跟随关节不单独下发；该 6 维口径与机型 `GripperAPI.set_positions` 一致；
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

1. 绝对路径（或用 `./`、`../` 的相对路径）；
2. `skills.play_action.actions_dir`；
3. `<技能根>/actions/<name>.json`（源码树 = `skills/motion/play_action/actions/`；
   部署后 = `<root>/skills/play_action/<ver>/actions/`）；
4. `$THINKER_ACTIONS_DIR`；
5. 当前工作目录。

无后缀自动补 `.json`。

**技能根目录的 `actions/` 会随离线包一起发布**：`pack_skill.sh` 把它拷到包内根目录，
`install_skill.sh` 解压后即 `<root>/skills/play_action/<ver>/actions/`，正好是运行期的
`<skill_dir>/actions`。所以自带动作直接调用 `action: goready` 即可。

现场新增/替换动作有两种方式，都不需要重装技能（每次调用时现读文件）：

- 拷到 `<root>/skills/play_action/<ver>/actions/`（`current` 软链目录下）；
- 或放到任意目录，配 `skills.play_action.actions_dir` 或 `$THINKER_ACTIONS_DIR`。

只改动作文件不需要重启 `thinker_host`；只有装/卸/升级技能包才需要重启（无热加载）。

## 失败语义

| code | 场景 |
|---|---|
| `INPUT_INVALID_PARAMS` | 缺 `action`、文件不存在、版本不是 2、clip 不存在、无可驱动关节 |
| `PRECOND_CAPABILITY_MISSING` | 装配期已判定（缺臂/腰/头能力，技能直接不进 registry） |
| `PRECOND_BODY_BUSY` | 上臂/腰/头被其它技能占用（租约冲突） |
| `SYS_TIMEOUT` | 动作时长 + 衔接超过 `timeout`（300s）或看门狗兜底 |
| `SYS_INTERNAL_ERROR` | 机型下发异常（如手部未启动） |

## 已知限制

- 不做碰撞/自碰撞检查，动作文件需录制端保证可达、不撞身；
- 只做关节空间回放，不平滑掉超速；速度由机型 `move_to` 的 `spd` 参数与 tick 目标共同决定；
- 手部 6 维归一化是有损映射（13→6），`dip` 与部分侧摆细节丢失；
- 腿轨迹一律忽略，见上文。
