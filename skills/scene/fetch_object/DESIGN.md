# fetch_object

场景技能：编排 `detect_object` 与手臂抓取。

| 项 | 内容 |
|---|---|
| 依赖能力 | `body.arm`、`body.gripper` |
| 依赖技能 | `detect_object`（运行期经 `ctx.call_skill`） |
| 参数 | `object_name`（默认 `bottle`）、`approach_joints`（默认右臂预设） |
| 步骤 | 识别 → 接近 → 抓取 → 抬起 |
| 后置状态 | `holding: true` |

## 与 D-006 的关系

- **设计期**：`package.xml` 的 `exec_depend` 声明依赖（供安装校验）
- **运行期**：`ctx.call_skill("detect_object", ...)` 调用，**不 import 源码**
- 子技能未安装时，`call_skill` 返回 `INPUT_UNSUPPORTED_COMMAND`，本技能如实失败

## 已知限制

- 当前无 IK，`approach_joints` 用预设关节角；接入 `move_to_pose` 后改为按 `obj_pose` 求解
- 抓取只有一次重试（`retries=1`）
