# fetch_object_mock

M0 的 mock 技能：**不碰真机**，用于验证「发现 → 装配 → 执行 → 反馈」全链路。

| 项 | 内容 |
|---|---|
| 步骤 | 感知定位 → 接近 → 抓取 → 回收 |
| 依赖能力 | `body.arm`、`body.gripper` |
| 后置状态 | `holding: true` |
| 超时 | 30s |

只调用 `ctx.body.arm/gripper`，不依赖任何 `core` 实现包，可用 `FakeBody` 在开发机跑通。
