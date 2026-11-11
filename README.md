# Thinker-Skill

> 部署在机器人本体上的**技能库**：把「感知 + 运控 + 状态机」闭环封装成可独立安装的技能单元，通过 **MCP Server** 与 **ROS Action Server** 两条通用接口对外提供服务，两者不绑定调用方。

## 项目定位

- **痛点**：参考实现三包平台代码重复、同机互斥、仅同步 Service 无进度/取消。
- **价值**：可安装（一个技能一个包）、可组合（命名空间隔离 + 区域仲裁）、双接口（共用执行内核）。
- **调用方**：LLM Agent、工作流编排。
- **边界**：编排在外部主机完成；应用层与算法实现不属本库（算法随技能走）。

## 核心特性

| 特性 | 说明 |
|---|---|
| 独立安装 | 一个技能 = 一个 ament 包，装/卸不影响 `core` 与其他技能 |
| 可组合 | 技能命名空间隔离；运控技能按身体区域（头/上肢/下肢）仲裁，同区串行、跨区并行 |
| 双接口 | MCP 面向 LLM Agent，ROS Action 面向工作流编排，共用统一信封 `{state, code, msg, retryable, data}` |
| 单一出口 | 三条通道（MCP / Action / Service）都调 `skill_runtime.execute()`，接口同构 |
| 可测试 | 每个技能可用 `FakeBody` / `FakeSensor` 在开发机跑通全步骤 |

## 架构

```
应用层  LLM Agent / 工作流编排 / 现场调试
  │
服务层  MCP Server（聚合 8000）· Skill.action · ControlCommand
  │
执行层  core/skill_core：registry · runtime · pipeline · body_arbiter · state_store
  │
技能层  skills/<类别>/<技能>/（业务实现，只依赖 skill_sdk）
  │
基础层  core/{interfaces, skill_sdk, common, sensor, body}
  │
本体层  控制器 · 相机 · 机械臂 · 底盘
```

依赖方向：`技能 → skill_sdk ← skill_core → body / sensor / common`。技能永远不依赖 `core` 实现包。

## 目录结构

```
Thinker-Skill/
├── skills/          # 技能，按类别分组
├── core/            # 平台核心，六个 ament 包，封闭集合
├── python/          # 协议适配：MCP 桥 / 聚合器 / 客户端 SDK
├── configs/         # 跨技能共享配置（机型 / 相机 / 场地 / errors.yaml）
├── weights/         # 模型权重（不入库）
├── docs/            # 设计文档
├── tools/           # 构建、打包、架构图脚本
├── tests/           # 跨包测试
├── .colcon/         # colcon 产物（gitignore）
└── Makefile
```

## 快速开始

### 环境要求

| 项 | 要求 |
|---|---|
| 系统 | Ubuntu 22.04 / 24.04 |
| ROS | ROS2 Humble（Py3.10）/ Jazzy（Py3.12），双版本对等 |
| Python | ≥ 3.10 |
| 硬件 | Jetson aarch64（开发机 x86_64） |

### 构建

```bash
make build                     # 全量构建，产物落 .colcon/
make build SKILL=fetch_object  # 只构建一个技能
make test                      # 单测，走 Fake 后端，不需要 ROS
```

### 调用

```bash
# MCP（聚合单端口，推荐）
curl http://localhost:8000/mcp ...

# ROS Action（需要进度/取消/抢占）
ros2 action send_goal /thinker/skill/fetch_object/action ...
```

## 技能结构

```
skills/manipulation/fetch_object/
├── skill/                # ament 包：skill.yaml + 业务源码 + 私有配置
├── mcp_interface/        # 平台注册壳：mcp_interface.py + metadata.json
└── DESIGN.md
```

技能只依赖 `thinker_skill_sdk`，能力经 `StepCtx` 注入，详见设计文档。

## 文档

| 文档 | 内容 |
|---|---|
| [概要设计](docs/Thinker-Skill%20概要设计文档.md) | 产品定义、系统架构、模块划分 |
| [详细设计](docs/Thinker-Skill%20详细设计文档.md) | 各模块的具体设计与思路 |
| [接口文档](docs/接口文档.md) | 接口契约、信封、错误码 |
| [决策记录](docs/决策记录.md) | 逐条决策与待澄清问题 |

架构图：[docs/assets/系统架构图.png](docs/assets/系统架构图.png)（由 `tools/render_architecture.py` 生成）

## 当前状态

设计阶段：四份设计文档已完成评审，代码实现进行中（里程碑 M0~M4 见决策记录 D-002）。
