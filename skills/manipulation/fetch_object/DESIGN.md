# fetch_object

场景技能：把桌面上的简单物体（水瓶/罐、苹果、香蕉等）抓到机器人持物位。

| 项 | 内容 |
|---|---|
| 依赖能力 | `body.arm.left`、`body.arm.right`、`body.gripper.left`、`body.gripper.right`（占 `upper` 区域） |
| 依赖技能 | `look_down_table`（低头）、`detect_object`（定位），运行期经 `ctx.call_skill` 调用，不 import 源码（D-006/D-019） |
| 参数 | `object_name`（默认 `bottle`）、`arm`（`auto` / `left` / `right`）、`overrides`（调试用参数覆写） |
| 步骤 | 低头预备 → 感知定位 → 规划抓取 → 接近 → 抓取位姿 → 合手 → 提拉 → 视觉确认 → 回收 |
| 后置状态 | `holding=true`、`holding_object=<类别>`（成功时统一写入） |
| 私有配置 | `configs/robot.yaml` 的 `skills.fetch_object` 段 |

## 为什么不声明 `body.head`

`Host.call_skill` → `runtime.execute` 会以**子技能自己的名字**重新
`arbiter.acquire(..., timeout=0)`。若本技能声明 `body.head`，调用
`look_down_table`（`requires: [body.head]`）时会立刻 `PRECOND_BODY_BUSY`。
所以低头必须外置为子技能，本技能只持有上肢租约。

## 步骤与判据

| 步骤 | 行为 | 判据 / 失败语义 |
|---|---|---|
| 低头预备 | `call_skill("look_down_table", {pitch_deg})` | 子技能失败原样上抛 |
| 感知定位 | `call_skill("detect_object", {object_name})` | `PERCEPTION_OBJECT_NOT_FOUND` 可重试（`probe_retries`） |
| 规划抓取 | 类别归一化 → 选臂 → 算接近/抓取/提拉位姿 → 可达性预检 | 越界/不可达 → `MOTION_IK_UNREACHABLE`，不动作 |
| 接近 / 抓取位姿 / 提拉 | 每 tick 幂等下发 `move_to_pose(tcp=True)` | 位置 ≤ `approach_tol`、姿态 ≤ `rot_tol` 且连续 `settle_ticks` 成立 |
| 合手 | `set_positions(grasp_positions)` + `grasp_settle_s` 等待 | 用 `ctx.now()` 计时，不阻塞 |
| 视觉确认 | 再调一次 `detect_object`，比较提拉前后位置 | 未再检出或 `z` 抬升不足 / 水平位移过大 → `MOTION_GRASP_FAILED`（可重试） |
| 回收 | `move_to_joints(hold_joints[side])` | 未配置则空操作；失败 `on_failure="skip"` |

## 抓取位姿

两层偏移，职责分开（详见《决策记录》D-020）：

1. 技能层（规划偏置，工具系）：`p_tcp = p_goal + R_goal · [Δx, Δy, Δz]`；
   接近与抓取只差 Δ 的量级。
2. 机型层（机械偏置，腕 roll 系）：`p_wrist = p_tcp − R_tcp · tcp_offset`，
   由 `move_to_pose(tcp=True)` 内部完成，本技能不参与。

姿态：`R_goal = Rz(ψ)·Ry(−90°)`，`ψ` 由目标横向方位角经 tanh 映射（上限 42°，
scale 28°，gain 1.6），左右手取反号；中心补偿 `x−0.01·cf`、`y±0.015·cf`。

## 目标物与参数分组

| 类别 | 别名 | 差异 |
|---|---|---|
| `bottle` | bottle / 水瓶 / 瓶子 / 饮料 / can / 罐 | 基准参数 |
| `apple` | apple / 苹果 / 水果 | `z_bias` 上抬、张角偏大、抬升判据略降 |
| `banana` | banana / 香蕉 | 张角偏小、`grasp_delta_y` 收紧 |
| `generic` | 其他任意名称 | 同 bottle |

归一化在技能内（`objects.py`），配置里不堆别名；未识别落 `generic` 不报错。

参数装配顺序：`DEFAULTS` → `skill cfg` → `objects.<类别>` → `params.overrides`。

## 与参考实现（kaiwu_agent/fetch_object）的差异

| 项 | 参考实现 | 本技能 |
|---|---|---|
| 动作循环 | QP 连发 20 次 ×0.04s | 每 tick 幂等下发 + 到位/稳定判据 |
| 腰部 | 升降腰适配高低物（0.61/0.85m） | 本机型腰无升降，不做等值迁移 |
| 成败判定 | 六维力 0.3N / ≥2 指 | 提拉后视觉二次确认（z 抬升 + 水平距离） |
| 感知 | 开集检测（可选 SAM2 mask） | 复用 `detect_object`（GroundingDINO，框中心/质心） |
| 物体位姿 | mask 质心 + 深度反投影 | 相同，且空间过滤口径一致 |
| 左右手 | 目标 `y<0` 用右手 | 相同，另支持 `arm` 参数强制 |

## 已知限制

- **香蕉等细长物体**：长轴朝向不参与规划。长轴与夹爪开合方向平行时会抓空，
  视觉确认判失败并返回可重试（不是 bug）。
- **不做 6-DoF 物体位姿**：非对称物体（马克杯把手定向、薄片边缘）不在支持范围；
  需要时按《决策记录》D-023 引入独立感知技能。
- **视觉确认的遮挡风险**：提拉后手部可能遮挡目标，此时判失败（可重试）。
- **参数未真机标定**：`tcp_offset` 与 Δ 初值来自参考实现，需按手型实测。
- **单次尝试**：抓空不在技能内自动重抓，由上层重新调用。

## 失败语义

| code | 场景 |
|---|---|
| `PERCEPTION_OBJECT_NOT_FOUND` | 重试后仍未检出目标（可重试） |
| `PERCEPTION_POSE_INVALID` | 检测结果缺位置 / 深度无效 |
| `MOTION_IK_UNREACHABLE` | 超出臂展或姿态不可解（不动作） |
| `MOTION_GRASP_FAILED` | 视觉确认判未抓起（可重试） |
| `PRECOND_CAPABILITY_MISSING` | 指定侧手臂/手部不存在 |
| `PRECOND_BODY_BUSY` | 上肢被其他技能占用 |
| `SYS_CONFIG_ERROR` | host 不支持技能组合调用 |
