# Thinker-Skill

部署在机器人本体上的**技能库**：把「感知 + 运控 + 状态机」闭环封装成可独立安装的技能单元，通过 **MCP Server** 与 **ROS Action Server** 两条通用接口对外提供服务，两者不绑定调用方。

- **可安装**：一个技能 = 一个 ament 包，装/卸不影响 `core` 与其他技能。
- **可组合**：技能命名空间隔离，运控技能按身体区域（头 / 上肢 / 下肢）仲裁。
- **双接口**：MCP 与 ROS Action 共用统一信封 `{state, code, msg, retryable, data}` 与同一执行内核。

## 开发进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| 设计 | 概要 / 详细 / 接口 / 决策记录四份文档 | ✅ 已评审 |
| M0 | 协议 + `skill_sdk` + `skill_core` 最小实现 + mock 技能 | ⬜ 待开始 |
| M1 | 感知类技能 `detect_object` | ⬜ 待开始 |
| M2 | 操作类技能 `fetch_object` + 聚合器 | ⬜ 待开始 |
| M3 | 类别扩容 `navigation` / `speech` / `camera` | ⬜ 待开始 |
| M4 | 离线安装包与卸载 | ⬜ 待开始 |

## 文档

- [概要设计](docs/Thinker-Skill%20概要设计文档.md) · [详细设计](docs/Thinker-Skill%20详细设计文档.md) · [接口文档](docs/接口文档.md) · [决策记录](docs/决策记录.md)
- 架构图：[docs/assets/系统架构图.png](docs/assets/系统架构图.png)
