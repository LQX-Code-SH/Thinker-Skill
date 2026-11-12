# Thinker-Skill 详细设计文档

| 项 | 内容 |
|---|---|
| 版本 | v1.1 |
| 日期 | 2026-11-11 |
| 本文定位 | 详细设计：各模块的具体设计、数据结构与设计思路 |
| 读者 | 平台开发者、技能开发者 |
| 配套文档 | [`Thinker-Skill 概要设计文档.md`](<Thinker-Skill 概要设计文档.md>) · [`接口文档.md`](接口文档.md) · [`决策记录.md`](决策记录.md) |

> 本文继承概要设计的结构，逐章展开模块的具体设计。为什么这样切见概要设计文档，怎么调用见接口文档，构建/部署/调试/验收见附录。

---

## 1. 设计目标与约束

### 1.1 设计目标

| 目标 | 可验证指标 |
|---|---|
| G1 独立安装 | 卸载任意技能后，其余技能正常运行 |
| G2 可组合 | ≥2 个技能同机共存、命名零冲突，运控不抢硬件 |
| G3 扩展成本低 | 新增技能只改 3 处，**不改 `core/`** |
| G4 接口一致 | 同一技能经 MCP 与 Action 调用，`state`/`msg` 语义一致 |
| G5 长任务可控 | 可拿阶段反馈、可取消，取消后本体安全停止 |
| G6 失败可判 | 中文 `msg` 给 LLM，枚举 `code` 给程序 |

### 1.2 设计约束

| 约束 | 说明 |
|---|---|
| 硬件 | Jetson aarch64，开发机 x86_64 |
| 系统 | Ubuntu 22.04 / 24.04 |
| ROS | ROS2 Humble / Jazzy（双版本对等，见 D-013） |
| Python | 3.10 / 3.12，**下限 3.10**，代码以 3.10 为基线 |
| 部署 | 离线安装包下发，不联网构建 |
| 代码保护 | Cython 编译，保护边界是 `skill_sdk` 公共符号 |

### 1.3 设计原则

五条贯穿全文的原则，后文不再重复论证：

1. **依赖单向**：`技能 → skill_sdk ← skill_core → body / sensor / common`，技能永远不依赖 `core` 实现包。
2. **`core` 封闭**：包数不随技能增长，只加函数不加包（详见 3.2.7）。
3. **契约稳定**：`skill_sdk` 是技能唯一依赖，只增不改，`min_core` 是兼容边界（详见 3.2.2）。
4. **业务下沉**：算法、词表、阈值随技能走，`core` 不含业务（详见 3.1.5）。
5. **执行受控**：tick 驱动、区域仲裁、看门狗兜底，三者保证组合部署不出事（详见 4.3、4.6、4.7）。

---

## 2. 总体架构实现

### 2.1 分层到代码

| 层 | 代码载体 |
|---|---|
| 0 应用层 | 外部（LLM Agent、编排器），不在本库 |
| 1 服务层 | `core/skill_core/frontends` + `python/`（桥、聚合器） |
| 2 执行层 | `core/skill_core`（registry / runtime / pipeline / body_arbiter / state_store） |
| 3 技能层 | `skills/<类别>/<技能>/` |
| 4 基础能力层 | `core/{interfaces, skill_sdk, common, sensor, body}` |
| 5 机器人本体层 | 硬件（控制器、相机、机械臂、底盘） |

### 2.2 执行链路

三条通道汇到同一出口，这是「接口同构」的结构保证：

```
MCP Tool ──┐
Action Goal ┼─▶ skill_runtime.execute() ─▶ Pipeline ─▶ Step.run(ctx) ─▶ ctx.body / ctx.sensor
Service ───┘                                     │
                                                 └─▶ 出口统一封 { state, code, msg, retryable, data }
```

设计要点：

- **单一出口**：适配层只做参数翻译与错误归一化，不写执行逻辑。
- **信封归一**：三条通道返回结构完全一致，只有序列化形式不同（dict vs JSON 字符串）。
- **出口生成 msg**：技能只产出 `code`，`msg` 由出口按 `configs/errors.yaml` 模板生成（见 4.7.3）。

### 2.3 依赖方向与 import 约束

```
技能包 ──▶ skill_sdk ◀── skill_core ──▶ body / sensor / common
                 ▲
        body / sensor（实现 BodyAPI / SensorAPI Protocol）
```

| 约束 | 强制手段 |
|---|---|
| 技能只 import `thinker_skill_sdk` 与消息包 | CI import 白名单检查，出现 `thinker_body` / `thinker_skill_core` 即失败 |
| `python/` 与 `core/` 不互相 import | 只经 ROS service/action 通信 |
| `body` / `sensor` 依赖 `skill_sdk` | 二者实现其 Protocol |

---

## 3. 模块设计

### 3.1 Skill 模块设计

#### 3.1.1 目录结构

一个技能目录 = 一个 ament 包 + 一个平台注册壳，两棵子树并列：

```
skills/manipulation/fetch_object/
├── skill/                            # ament 包（包名 fetch_object）
│   ├── package.xml  setup.py         # exec_depend thinker_skill_sdk + thinker_interfaces
│   ├── skill.yaml                    # 安装契约（命令、能力、超时、错误码）
│   ├── src/fetch_object/             # 业务源码
│   │   ├── plugin.py                 #   入口：build_steps()
│   │   ├── targets.py                #   目标选择策略（业务）
│   │   └── geometry.py               #   本技能特有几何（业务）
│   ├── configs/skills/fetch_object.yaml   # 私有参数：偏置、阈值、重试次数
│   ├── launch/skill.launch.py
│   └── start.sh                      # SESSION_NAME=thinker_fetch_object
├── mcp_interface/                    # 平台注册壳（非 colcon 包）
│   ├── mcp_interface.py              #   工具函数 + docstring
│   ├── metadata.json                 #   平台注册清单
│   ├── generate_md5.py               #   指纹生成
│   └── requirements.txt              #   fastmcp / rclpy 客户端代理依赖
└── DESIGN.md
```

**为什么拆两棵子树**：

| 原因 | 说明 |
|---|---|
| 构建边界 | `skill/` 走 colcon；`mcp_interface/` 不进工作空间，改工具描述或端口无需重新构建 |
| 编译边界 | `skill/` 参与 Cython；`mcp_interface.py` 必须明文（FastMCP 需反射签名生成 schema） |
| 平台路径 | `metadata.json` 的 `modulePath` 与目录一一对应 |

约束：`skill.yaml` 必须在 `skill/` 内（ament `data_files` 不能引用包外路径）；用 `src/fetch_object/` 避免目录名/包名/模块名三层同名。

#### 3.1.2 技能契约 `skill.yaml`

```yaml
name: detect_object
version: 0.1.0
category: perception
min_core: 0.1.0             # skill_sdk API 兼容边界（见 3.2.2）
entry: detect_object.plugin:DetectObject     # 提供 build_steps()，由 host 装配
commands: [detect_object]
requires: [body.head, sensor.rgbd]           # 能力名带前缀；区域映射见 4.6
precondition: []            # 见 4.5
postcondition: {}           # 成功后写回状态；detect_object 不写
timeout: 30                 # 秒；走 MCP 时须小于客户端 read timeout
interfaces: [service, action]                # ROS 通道；MCP 由下方 mcp 段声明
errors:                     # 本技能可能产出的 code，供测试与编排期校验
  - PERCEPTION_OBJECT_NOT_FOUND
  - PERCEPTION_POSE_INVALID
  - SYS_CONFIG_ERROR
mcp: {tools: [detect_object], port: 8032}
```

安装到 `share/<pkg>/skill.yaml` 并在 ament index 打 marker，**装了就发现、卸了就消失**。

#### 3.1.3 技能入口与 `build_steps`

`entry: <module>:<Class>` 指向技能类，host 装配时导入并构造实例，调用：

```python
def build_steps(self, ctx: StepCtx, params: dict) -> list[Step]:
    ...
```

设计约束：

- **只产步骤，不写调度**：返回 `Step` 列表，循环、计时、进度、失败处理由 Pipeline 负责（契约见 4.3）。
- **参数来自 `ctx.cfg` 与 `ctx.params`**：不读全局单例，保证跨技能参数隔离。
- **可重入**：`build_steps` 可能被重试重入，不得依赖一次性副作用。

#### 3.1.4 注册壳 `mcp_interface`

| 文件 | 职责 |
|---|---|
| `mcp_interface.py` | 工具函数 + docstring（**docstring 即 LLM prompt**） |
| `metadata.json` | 平台注册清单：`name` / `version` / `modulePath` / `functionNames` / `port` |
| `requirements.txt` | 独立声明 `fastmcp` 等依赖（不进 colcon 构建） |

设计思路：注册壳**必须保持明文**（FastMCP 需反射函数签名生成 schema），且改动不需重新构建；工具名 = 技能名 = 目录名，三者一致由自动化测试校验。

#### 3.1.5 技能边界

| 放 | 不放 |
|---|---|
| 步骤编排（`build_steps`） | 10Hz 循环、超时与重试机制 |
| 业务语义：物体词表、选择策略、位姿算法、重试次数 | 控制器封装、QP/moveit 调用细节 |
| 参数与文案（`configs/skills/<skill>.yaml`） | 力订阅、tf、变换工具等平台能力 |
| `skill.yaml`（契约） | ROS 节点装配（由 host 统一装配） |
| `DESIGN.md`、`mcp_interface/` | 其他技能的任何源码 |

归属判定（一段业务逻辑该放哪）：

```
这段逻辑是硬件驱动、数据通道或纯通用工具吗？
├─ 是 → core/body / core/sensor / core/common（只在既有包内加函数）
└─ 否 → 留在技能包（检测、估计、识别等算法与业务逻辑都属此类），
        允许与其他技能相似但不共享
```

示例对照：

| 代码 | 归属 | 理由 |
|---|---|---|
| `filter_detections(...)` | `core/common` | 通用筛选工具，不含业务词 |
| GroundingDINO 加载与推理 | 技能包 | 算法随技能走，模型延迟加载 |
| `BOTTLE_LABELS = {...}` | 技能包 | 业务词表 |
| `waist.set_height(z)`（含 clamp） | `core/body` | 安全边界属平台 |
| “目标高度 < 0.6 时把腰降到 0.61” | 技能包 | 业务决策 |

**允许的重复边界**：参考三包的问题是**平台代码重复**（Pipeline、控制器、感知流水线各存一份），业务逻辑重复百余行是可接受成本。

| 情况 | 处理 |
|---|---|
| 两个技能有一小段相似逻辑 | 各自实现，不抽 |
| 三个以上技能逐行相同 | 先判断是否通用能力被写成业务代码，是则上移 `core` 既有包 |
| 只差参数的同源逻辑 | 逻辑一份在技能包，参数进 `configs/skills/<skill>.yaml` |
| 跨技能组合 | 场景技能经 `ctx.call_skill` 进程内编排，不共享代码（D-019） |

### 3.2 Core 模块设计

六个包按角色分三类：**契约**（`interfaces`、`skill_sdk`）、**能力**（`body`、`sensor`、`common`）、**机制**（`skill_core`）。

#### 3.2.1 `interfaces`（协议定义）

| 定义 | 内容 |
|---|---|
| `ControlCommand.srv` | `{command, params}` → `{state, code, msg, retryable, data}`（调试通道） |
| `Skill.action` | Goal `{skill, params_json}`；Result；Feedback `{phase, progress, msg}` |
| 消息包 | 检测/抓取等跨包消息 |

设计思路：`Skill.action` 定为**通用型**（goal 带技能名），新增技能不改协议、只加命令名（D-002 第 3 条）。

#### 3.2.2 `skill_sdk`（契约层）

**定位** —— 技能作者唯一依赖：只含类型、Protocol、错误码常量，**纯 Python，无 ROS 运行时依赖**，可单独 `pip install -e` 供纯单测。

##### 编排数据结构

```python
@dataclass
class Step:
    name: str                     # 唯一阶段名 → feedback.phase
    run: Callable[[StepCtx], StepResult]   # 每 tick 调一次
    kind: Literal["motion", "nav", "perception", "wait", "io"] = "motion"
    weight: float = 1.0           # 进度权重
    timeout: float | None = None  # 单步超时，默认继承实际超时
    retries: int = 0              # 重试次数，耗尽后按 on_failure 处理
    on_failure: Literal["abort", "skip"] = "abort"
    tick_budget: float = 0.1      # 单次 run 的最长耗时（秒）

@dataclass
class StepResult:
    status: Literal["running", "done", "failed"]
    code: str | None = None       # status=failed 时必填，见《接口文档》§8
    retryable: bool = False       # 与信封字段语义一致，决定 retries 是否消耗
    data: dict = field(default_factory=dict)
```

##### `StepCtx`（能力注入）

```python
@dataclass
class StepCtx:
    skill: str                        # 技能名
    params: dict                      # 本次调用入参
    cfg: Mapping                      # 仅本技能私有配置（只读）
    globals: Mapping                  # 全局白名单项（open_nav / tick_rate）

    body: BodyAPI                     # 抽象能力，不暴露具体控制器类型
    sensor: SensorAPI                 # rgbd() / transform_to_base()

    state: StateAPI                   # get/set，key 白名单
    log: Logger                       # 结构化，自动带 skill + trace_id
    say: Callable[[str], None]        # 语音播报

    cancelled: bool                   # 取消标志
    check_cancel: Callable[[], None]  # 抛 Cancelled，长步骤内部也要调
    now: Callable[[], float]          # 可注入假时钟
    append_steps: Callable[[list], None]
```

##### `BodyAPI` / `SensorAPI`（能力抽象，按能力切不按机型切）

```python
class BodyAPI(Protocol):
    arm: ArmAPI          # move_to_pose / move_to_joints / get_tcp_pose / is_moving
    gripper: GripperAPI  # grasp / release / width / effort / is_holding
    waist: WaistAPI      # set_height / set_pose
    head: HeadAPI        # look_at / look_down
    base: BaseAPI        # navigate_to / arrived / stop / get_pose

class SensorAPI(Protocol):
    def rgbd(self) -> Frame: ...
    def transform_to_base(self, pose) -> Pose: ...
```

**所有方法必须 tick 友好**：要么幂等，要么提供可查询的完成状态。导航不能是阻塞式 `navigate_to()` 一直等，而是 `navigate_to(pose)` 幂等启动 + `arrived()` 查询——否则技能作者会写出阻塞调用，破坏 4.3 的 tick 契约。

##### 兼容策略（`min_core` 是唯一边界）

| 级别 | 范围 | 规则 |
|---|---|---|
| 稳定 | `Step`/`StepResult`/`StepCtx` 字段语义、错误码常量 | 不可改 |
| 可演进 | `BodyAPI`/`SensorAPI` 方法、`StepCtx` 字段 | 只增不改，新增给默认/可选实现 |
| 破坏性 | 既有方法签名/语义变更 | 走 `min_core` 大版本升级 |

技能用 `skill.yaml.min_core` 声明最低 SDK 版本，装配期校验，不满足则跳过并告警（D-011）。

#### 3.2.3 `common`（通用工具）

| 组件 | 职责 |
|---|---|
| `DataBuffer` | 时间戳对齐的滑窗缓冲（感知/运动共用） |
| `TransformUtil` | 通用几何变换（`tag_to_gripper` 等） |
| `ConfigLoader` | `includes` 递归合并（见附录 B） |
| `logger` | 结构化日志，带 `skill` + `trace_id` |
| `TTS` | 语音播报封装 |

**设计思路**：纯 Python、无 ROS 依赖，与 `skill_sdk` 同档，可脱离 ROS 单测。`NodeAdapter` 不在此包（属 ROS 节点装配，归 `skill_core`）。

#### 3.2.4 `sensor`（感知数据通道）

| 职责 | 内容 |
|---|---|
| 数据获取 | 相机订阅、RGB-D 对齐、内参 |
| 坐标变换 | `cam→base` 变换、时间戳缓冲 |
| **不含** | 检测/估计/识别算法（随技能包）、机型实现（随 `robots/`，D-007 / D-014） |

**设计思路**：只做「数据 + 坐标」的抽象与通用处理（时间戳、对齐）；算法属业务、机型实现属 `robots/<model>`，两者都不进 `core`，避免 `core` 随算法/机型膨胀。

> **实现状态**：当前为骨架（仅 `__init__.py`）。时间戳缓冲、RGB-D 对齐、录制回放待实现；数据获取与 `cam→base` 已在 `robots/<model>/sensor.py` 可用。

#### 3.2.5 `body`（抽象与通用安全层）

| 职责 | 内容 |
|---|---|
| 契约 | `BodyAPI` 抽象与协议实现约束 |
| 通用安全层 | 限幅、急停钩子、有效空间校验（与机型无关的部分） |
| **不含** | 控制器、QP、moveit、具体机型适配（随 `robots/<model>`，D-014） |

**设计思路**：`core/body` 只放与机型无关的抽象和通用安全逻辑；具体硬件实现由 `robots/<model>/body.py` 提供，按能力切分（`arm`/`gripper`/`waist`/`head`/`base`）且 tick 友好。这样 `core` 不随机型增长。

> **实现状态**：当前为骨架（仅 `__init__.py`）。限幅与急停收敛已下沉到 `robots/<model>/safety.py`（机型差异），`core/body` 的通用安全层尚待实现。

#### 3.2.6 `skill_core`（执行机制）

| 组件 | 职责 |
|---|---|
| `registry` | 技能发现与装配：扫描 `share/*/skill.yaml` → 校验 → 构造实例 |
| `runtime` | 单一出口 `execute()`：参数校验、租约、Pipeline 调度、信封封装 |
| `pipeline` | 驱动 `Step` 列表：10Hz tick、进度、超时、重试、失败处理 |
| `body_arbiter` | 区域仲裁与租约（见 4.6） |
| `state_store` | 语义状态存取，key 白名单（见 4.5） |
| `frontends` | MCP / Action / Service 三个入口，只做翻译 |
| `NodeAdapter` | ROS 节点装配与回调组管理 |

**设计思路**：技能**不依赖**本包，所以内部可自由重构（稳定性低不代表质量低，而是允许演进）；`frontends` 只翻译不执行，保证三条通道同构。

#### 3.2.7 `core` 准入标准与命名

`core` 是**封闭集合**：包数与机器人形态相关，**不随技能数量增长**。进 `core` 必须四条全满足：

| # | 标准 | 反例 |
|---|---|---|
| 1 | 与具体技能、物体、场景无关 | `select_bottle()` |
| 2 | 平台自身或多个技能确实需要 | 只有一个技能用的辅助函数 |
| 3 | 代码中不出现业务词表（`bottle`/`box`/`drawer`） | 出现即说明是业务逻辑 |
| 4 | **只在既有包内新增函数/类型，不新增包** | 新增包需架构评审 |

包命名对照：

| 目录 | ROS 包名 | 谁依赖 |
|---|---|---|
| `core/interfaces` | `thinker_interfaces` | 全部 |
| `core/skill_sdk` | `thinker_skill_sdk` | 技能包、`skill_core`、`body`、`sensor` |
| `core/common` | `thinker_common` | `skill_core`、`body`、`sensor` |
| `core/sensor` | `thinker_sensor` | `skill_core` |
| `core/body` | `thinker_body` | `skill_core` |
| `core/skill_core` | `thinker_skill_core` | 部署单元（技能**不**依赖） |
| `skills/<类别>/<技能>` | `<技能>` 原名 | — |

### 3.3 协议适配模块设计（`python/`）

| 组件 | 职责 |
|---|---|
| `thinker_mcp_bridge` | 技能注册壳（`mcp_interface/`）的平台侧桥接：把工具调用转为 ROS 请求 |
| `thinker_mcp_aggregator` | 聚合单端口（8000）：聚合全部技能的 MCP 工具，技能增删只重启它 |
| 客户端 SDK | 供调用方接入 MCP / Action 的轻量封装 |

**设计思路**：

- **聚合单端口**：MCP 客户端不支持热加载，因此对外只暴露一个聚合端口（8000），技能增删只重启聚合器；3.1.2 的 `mcp.port`（如 8023）是技能独立端口，仅用于平台注册，对外统一走 8000。
- **边界**：`python/` 与 `core/` **不互相 import**，只经 ROS service/action 通信；本模块不含业务逻辑。
- **框架**：统一用第三方 `fastmcp`（`from fastmcp import FastMCP`），服务端声明 `fastmcp>=2.12.3,<4`，按 Python 版本自适应（D-013）。

### 3.4 机型适配模块设计（`robots/`）

**定位** —— 一机型一包，把机型 SDK 的硬件接口适配成统一 `BodyAPI` / `SensorAPI`。技能不感知机型。

**结构**：

```
robots/tienkung_dex/                # ament 包 tienkung_dex_robot
├── package.xml  setup.py
├── capabilities.yaml               # 能力契约（含 hand_variants）
├── tienkung_dex_robot/
│   ├── adapter.py                  # create_adapter 工厂（变体 → hand_vendor/enable）
│   ├── body.py                     # 实现 BodyAPI（封装 create_robot）
│   ├── sensor.py                   # 实现 SensorAPI（4 路 RGB-D）
│   ├── safety.py                   # 机型安全层（限幅、急停收敛）
│   ├── joints.py                   # 关节表常量（与 resources/joints.yaml 一致）
│   └── kinematics.py               # 头部相机 FK（cam→base）
└── resources/                      # 随包安装（setup.py data_files）
    ├── joints.yaml                 # 关节 ID/限位事实源
    ├── urdf/tienkung_dex_brainco.urdf
    └── calib/head_gemini336_camera_info.yaml
```

`resources/` 经 `ament_index_python.get_package_share_directory()` 定位，**部署后不依赖开发机绝对路径**。

**适配实现要点**（以 tienkung_dex 为例）：

| 能力 | 适配方式 |
|---|---|
| 上肢 `arm` | 封装 `robot.arm`；关节级 `move_to_joints` + **位姿级 `move_to_pose`（QP + DAQP，D-017）** |
| 上肢 `gripper` | `robot.hand_left/right.set_positions`，0..1 归一化；按 `hand_variants` 支持 brainco / inspire / 拳头假手 |
| 头部 `head` | `robot.head`，2 DOF（yaw / pitch） |
| 下肢 `base` | `robot.walk.set_velocity`（20Hz 速度流），**无位姿导航** |
| `sensor.rgbd` | `robot.cameras`，4 路 RGB-D（depth 单位 mm、color BGR），内参经 `Frame.k` 注入 |

**与 SDK 的关系**：直接封装 `tienkung_dex` 门面（`create_robot(backend=real/sim/mock)`），不重写 ROS 话题层；`mock` 后端用于离机开发与单测。

**必补的安全层**（SDK 不做，集中在 `safety.py`）：

- 关节限幅：按 `joints.py` / `resources/joints.yaml` 的 limits 夹紧；
- 急停：SDK 的 `EstopActiveError` 收敛为 `safety.EstopActive`，禁用 `ZERO_CALIB`；
- 行走前置检查：是否已进话题模式（SDK 未封装，需自订阅 `SbusData` / `rl_state`）。

**手部变体**（同一机型多配置）：

| 变体 | `hand_vendor` | 启用 `hand` | `body.gripper` |
|---|---|---|---|
| `brainco` | `brainco` | 是 | `{type: brainco, dof: 6}` |
| `inspire` | `inspire` | 是 | `{type: inspire, dof: 13}` |
| `fixed`（拳头假手） | — | 否 | **不存在** |

`create_adapter(hand_variant=...)` 按变体推导 `hand_vendor` 与 `enable`；`fixed` 时 `enable` 显式排除 `hand`，`body.gripper` 不可用。

**能力匹配**（装配期）：

```
caps = load(robots/<configs/robot.yaml 的 model>/capabilities.yaml)
      + hand_variants[configs/robot.yaml 的 hand_variant]   # 合并手部变体
for skill in discovered:
    if not compatible(skill.compatible, caps):        # 机型 / 版本
        skip + warn; continue
    if not subset(skill.requires, caps.capabilities): # 能力子集
        skip + warn; continue
    装配
```

v1 只校验「能力名 + `robot_models` + `min_robot_version`」，参数约束（`dof`/`reach`/`type`）的匹配器 v1.1 启用（D-015）。

**新增机型**：新建 `robots/<model>/` 包，实现 `BodyAPI` / `SensorAPI` + 写 `capabilities.yaml`，`configs/robot.yaml` 的 `model` 指过去即可，**不改 `core`**。

---

## 4. 技能模型实现

### 4.1 分类与目录

分类口径是「技能结束后的世界变化」（概要 4.1），落到代码：

| 类别 | 目录 | 判定标准 |
|---|---|---|
| 导航移动类 | `navigation/` | 改变 base 位姿 |
| 操作类 | `manipulation/` | 改变手臂/夹爪或物体位置 |
| 拍照类 | `camera/` | 只产出图像/视频 |
| 语音对话类 | `speech/` | 只涉及音频 I/O |
| 视觉感知类 | `perception/` | 产出结构化世界描述 |
| 场景类 | `scene/` | 编排其他技能（v1.1 后置，D-002） |

- 类别目录**只是分组**，不建 metapackage。
- 「装一个类别」= `colcon build --base-paths skills/<类别>`。

### 4.2 安装契约与技能发现

生命周期：

```
安装（colcon / 离线包）
  → 发现（host 扫描 share/*/skill.yaml）
  → 校验（min_core 版本、requires 能力是否齐备；不满足则跳过并告警，不阻塞其他技能）
  → 装配（导入 entry 构造实例，注册到 registry，由 host 起服务层）
  → 执行（Pipeline 10Hz 驱动动作链）
  → 状态写回（state_store）
```

前四步属**装配期**（host 启动时一次），后两步属**调用期**（每次调用）。

设计说明：

- **发现靠 ament marker**：`skill.yaml` 随包装到 `share/<pkg>/skill.yaml` 并打 index marker，装了就发现、卸了就消失。
- **校验不阻塞**：`min_core` 不满足或能力缺失时跳过该技能并告警，其余技能照常装配——这是「独立安装」的落地保证。
- **不做热加载**：安装/升级/卸载后必须重启 host（见附录 C）。

### 4.3 执行模型（声明式 Step）

技能不直接操作 Pipeline，只产出**步骤列表**；Pipeline 负责驱动、计时、反馈、失败处理。

> 澄清：声明式的是**步骤结构**，不是**执行方式**。运控必须每 tick 持续下发目标（参考实现 QP 就是循环下发 20 次 × 0.04s），因此 `Step.run` 仍是**每 tick 调用一次**的 tick 函数（D-004）。

#### 执行语义

```
total   = sum(step.weight)
done_w  = 0
for step in steps:
    emit(phase=step.name, progress=done_w / total)
    deadline = now + (step.timeout or 实际超时)
    while True:
        if stop_requested:                 # cancel / 看门狗 / 急停
            soft_estop(); return CANCELLED
        r = run_with_budget(step)          # 超过 tick_budget 记违规
        if r.status == "done":   break
        if r.status == "failed":
            if r.retryable and step.retries > 0:
                step.retries -= 1; continue
            if step.on_failure == "skip":
                record_warning(r.code); break
            soft_estop(); return FAILED(r.code)
        if now > deadline: return SYS_TIMEOUT
        sleep(0.1)
    done_w += step.weight
```

设计思路：

- **实际超时**：技能声明的 `skill.timeout`，否则取通道默认（MCP 300s / Action `default_timeout`），见 D-009；单步 `timeout` 缺省继承它。
- **进度由权重累加**：`progress = done_w / total`，技能无需上报进度。
- **`phase` 取自 `Step.name`**：阶段与步骤一一对应，不需要技能维护阶段名。
- **失败策略分档**：`abort` 全链中止 + 停；`skip` 记告警继续。
- **动态追加**：`ctx.append_steps()` 支持运行期插入步骤。

#### 与参考实现的差异

| 项 | 参考实现（`arch/pipeline.py`） | 本设计 |
|---|---|---|
| 步骤表达 | `(callable, "描述串")` 元组 | `Step` 数据类 |
| `phase` 来源 | 复用描述串 | `Step.name` 独立字段 |
| 步骤分类 | 描述串前缀字符串匹配 | `kind` 枚举 |
| 开关过滤 | 按前缀匹配 | 按 `kind` 过滤 |
| 进度 | 无 | 按 `weight` 累加 |
| 单步超时 / 重试 | 无（业务里手写循环） | `timeout` / `retries` + `retryable` |
| 失败策略 | 一律中止 | `on_failure: abort / skip` |
| 动态追加 | `get_extend_actions_list()` | `ctx.append_steps()` |

去掉了「描述串前缀兼作路由键」——它把展示文案和流程控制耦在一起，改一个字就可能改变执行路径。

#### 三条硬约束

1. **单次 `run` 不得超过 `tick_budget`**（默认 0.1s；感知等待类可声明至 2s），否则 Pipeline 记违规并告警。
2. **`run` 必须可重入**：重试时会再次调用，不能依赖「只调用一次」的副作用。
3. **`run` 必须响应取消**：不可有不可打断的阻塞，所有 IO 必须带超时（感知取数 2s 即为此）。

#### 示例

```python
class FetchObjectSkill:
    def build_steps(self, ctx, params) -> list[Step]:
        return [
            Step(name="感知定位", kind="perception", weight=2, tick_budget=2.0,
                 retries=3, run=lambda c: self.locate(c, params["object_name"])),
            Step(name="接近", kind="motion", weight=1, run=self.approach),
            Step(name="抓取", kind="motion", weight=2, retries=2, run=self.grasp),
            Step(name="回收", kind="motion", weight=1, on_failure="skip", run=self.to_hold),
        ]
```

`feedback.phase` 依次为「感知定位 → 接近 → 抓取 → 回收」，`progress` 由权重累加得出，两者都不需要技能自己上报。

### 4.4 能力注入与约束

技能不持有任何控制器、不 `import` 任何 `core` 实现包，全部能力经 `StepCtx` 获取（`StepCtx` 定义见 3.2.2）。这是「可独立安装、可跨机型、可脱离真机测试」三项要求的共同前提。

#### 五条规则

| # | 规则 | 原因 |
|---|---|---|
| 1 | 技能只 `import thinker_skill_sdk` | `core` 重构、Cython 裁剪不影响已安装技能 |
| 2 | `ctx.cfg` 只含本技能配置段 | 参考实现全局 `config` 单例导致跨技能参数污染 |
| 3 | `postcondition` 由 Pipeline 成功后统一写 | 避免「返回成功但状态没写」 |
| 4 | 禁止持有跨 tick 的全局可变状态 | 重试会重入，状态落在闭包或 `ctx.state` |
| 5 | 模型**延迟加载**：首次 `Step.run` 才 `ensure_model()` | 装 N 个技能就有 N 份模型常驻显存，Jetson 撑不住 |

#### 收益：脱离真机测试

```python
# tests/fakes.py
ctx = make_ctx(body=FakeBody(), sensor=FakeSensor(frames=[frame]))
steps = FetchObjectSkill().build_steps(ctx, {"object_name": "bottle"})
run_pipeline(steps, ctx)          # 无 ROS、无真机，毫秒级跑完
```

M0 阶段即可在开发机跑通 MCP → runtime → pipeline → feedback 全链路。

### 4.5 状态管理

组合技能之间靠 `state_store` 传递语义状态，而不是靠调用顺序的巧合。

| 状态 | 写入者 | 读取者 |
|---|---|---|
| `holding` / `holding_object` | `fetch_object` | `place_object`、`handover` |
| `map_ready` | 建图技能 | `navigate_to` |
| `last_detection` | `detect_object` | `fetch_object`（可选加速） |

设计约束：

- **key 白名单**：`state_store` 只接受注册过的 key，防止任意技能污染全局状态。
- **前置校验**：`precondition` 不满足返回可读失败（如「我手上没有东西，请先让我抓取」），而非执行错误动作。
- **后置统一写**：`postcondition` 由 Pipeline 在成功后统一写，避免「返回成功但状态没写」。

### 4.6 身体仲裁与释放

身体（控制器、夹爪、相机）是硬件单例，按三大物理区域仲裁（D-008），不拆到关节：

| 区域 | 标识 | 映射能力（`requires`） |
|---|---|---|
| 头部 | `head` | `head` |
| 上肢 | `upper` | `arm`、`gripper`、`waist` |
| 下肢 | `lower` | `base` |
| 豁免 | — | `sensor.*`（只读，不占区域） |

| 机制 | 说明 |
|---|---|
| 区域互斥 | 同区域串行、跨区域并行；`body_arbiter.acquire(skill, capabilities, ttl)` 成功后才能驱动对应区域 |
| 原子获取 | 需要多区域时一次性全拿或全不拿，避免死锁 |
| 排队 | 未获得时排队，Action goal 保持 pending，`feedback.phase` 显示「等待身体」 |
| 抢占 | 带 `preempt` 的新 goal 软急停旧持有者 → 停稳确认 → 交接，旧 goal 返回 `cancelled` |
| 租约 | 持有必须带 TTL，超时未续约自动释放；续约由 Pipeline 每 tick 自动完成 |
| 只读豁免 | `camera` / `perception` / `speech` 类只读，不参与仲裁，可与其他技能并行 |

能力 → 区域映射配置化（`configs/robot.yaml`），技能仍只声明能力，跨机型承诺不变。

#### 租约生命周期

```
free ──acquire(能力, ttl)──▶ held
held ─┬─ 正常完成/失败/取消 ─▶ 软急停 ─▶ 停稳确认 ─▶ release ─▶ free
      ├─ 抢占(preempt)     ─▶ 软急停旧 ─▶ 停稳确认 ─▶ 交接给新 ─▶ held(新)
      ├─ 紧急抢占          ─▶ 立即停止(不等停稳) ─▶ 交接 ─▶ held(新)
      ├─ 看门狗超时         ─▶ 软急停 ─▶ release ─▶ free（被动兜底）
      └─ TTL 未续约        ─▶ release ─▶ free（被动兜底，防进程崩溃）
```

#### 四条硬约束

1. **释放必须停稳确认**：`stop → is_moving == false → release`，下一个持有者接管时身体必处于静止安全态。
2. **抢占 = 有序交接**：普通抢占走「软急停 → 停稳 → 交接」，任何时刻身体只有一个真正控制者。
3. **续约自动化**：租约 TTL 由 Pipeline 每 tick 自动续，技能作者无感知（避免正常长任务被 TTL 误杀）。
4. **抢占分级**：普通抢占走完整停稳流程；急停/安全事件的紧急抢占立即停止、不等停稳。

### 4.7 取消与安全

#### 4.7.1 取消三路归一

三条取消路径归一为同一条软急停（约束见《接口文档》§6）：

| 路径 | 触发者 | 可靠性 |
|---|---|---|
| Action `cancel` | 编排器主动取消 | 协议保证，任何 ROS2 客户端都会用 |
| MCP `notifications/cancelled` | MCP 客户端主动取消 | 取决于客户端实现 |
| 超时看门狗 | 服务端自身 | 无条件兜底 |

#### 4.7.2 看门狗边界

看门狗**只守护持有身体租约的技能**，不是所有任务的计时器（D-009）：

- 触发条件：持租约时长超过 `实际超时 × 1.2`
- 触发动作：软急停 + 释放租约 + 返回 `SYS_TIMEOUT`
- 不持身体的技能（对话/拍照/感知）不参与看门狗，其超时是技能自己的业务语义

三层兜底：Pipeline 内部超时（业务可控）→ 看门狗（防客户端消失/执行流卡死）→ 租约 TTL（防进程崩溃）。

#### 4.7.3 错误出口与安全机制

技能只产出 `code`，`msg` 由出口按 `configs/errors.yaml` 生成；异常在出口收敛为 `SYS_INTERNAL_ERROR`，不泄漏栈。

| 机制 | 参数化 | 说明 |
|---|---|---|
| 抓取成败判断 | `hand_grasp_success_effort` / `_finger_num` | 六维力 / 指尖力阈值，进技能私有配置 |
| 卡死保护 | 力监控 | 超阈值 → 软急停，返回 `SAFETY_STUCK_DETECTED` |
| 动作失败中止 | — | 任一 `Step` 失败且 `on_failure=abort` → 全链中止 + 停止 |
| 急停 / 暂停 | `CommandNode` | 外部话题触发 |
| 姿态 / 空间约束 | `valid_spatial_range`、腰高 clamp | 目标超出有效空间直接拒绝 |
| 权重缺失降级 | `detector_name` 切换 | 云端不可达时切本地模型或明确报错 |
| 无人值守 | `open_cmd_waiting: false` | 禁止 `input()` 阻塞 |
| 网络暴露面 | `host` 可配置 | MCP 默认绑 `localhost`；对外需显式设 `host` 并前置反代 + 白名单，v1 不做协议层鉴权 |
| 参数白名单 | key 注册表 | `state_store` 只接受注册 key，`ctx.cfg` 只给本技能段 |

---

## 5. 非功能设计

### 5.1 性能

| 指标 | 要求 | 落地手段 |
|---|---|---|
| 动作循环 | 10Hz | Pipeline `sleep(0.1)` + `tick_budget` 约束 |
| 感知发布 | 10Hz | `core/sensor` 10Hz 发布 |
| 感知→运动取数 | ≤ 2s | `sensor.rgbd()` 带超时，感知等待类 `tick_budget=2.0` |
| 启动 | 单技能 < 10s；host 加载 N 技能 < 15s | 装配期一次构建 registry；模型延迟加载不占启动时间 |

### 5.2 并发

同一身体区域串行、跨区域并行；只读技能并行（见 4.6）。并发模型是单 host 内多租约，不是多进程抢占硬件。

### 5.3 可观测

- 结构化日志按技能分文件，自动带 `skill` + `trace_id`。
- `trace_id` 贯穿 MCP → Action → Pipeline，一次调用可串起全链路。

### 5.4 代码保护

| 项 | 做法 |
|---|---|
| 保护范围 | `core` 与 `skills` 的业务 `.py` 用 Cython 编译 |
| 明文目录 | `mcp_interface/` 不参与编译（FastMCP 需反射签名生成 schema） |
| 保护边界 | `skill_sdk` 公共符号是唯一对外契约，内部模块可随意改名 |

### 5.5 兼容性

| 维度 | 策略 |
|---|---|
| ROS | `package.xml` 只按包名依赖、不锁 distro，colcon 按当前 distro 解析（Humble/Jazzy 对等，D-013） |
| Python | 下限 3.10，代码以 3.10 为基线 |
| `fastmcp` | `>=2.12.3,<4` 版本范围，pip 按环境解析；代码只用 2.x/3.x 稳定公共 API |
| 技能契约 | `min_core` 声明最低 SDK 版本，装配期校验 |

### 5.6 可测试性

每个技能必须能仅用 `FakeBody` / `FakeSensor` 在开发机跑通全部步骤（见 4.4），CI 双 distro matrix 各跑一遍。

---

## 附录 A 工作空间与构建

仓库根即工作空间根，用 `Makefile` 封装参数：

```
Thinker-Skill/
├── skills/          # 技能，按类别分组
├── core/            # 共享实现，只写一份
├── python/          # MCP 桥 / 聚合器 / 客户端 SDK
├── configs/         # 跨技能共享配置
├── weights/         # 模型权重（不入库）
├── .colcon/         # build/install/log（gitignore）
├── docs/ tests/ tools/
├── Makefile
```

```bash
make build                            # colcon build --base-paths core robots skills，产物落 .colcon/
make build-skill SKILL=detect_object  # 只构建一个技能
make test                             # 单测，走 Fake 后端，不需要 ROS
```

构建顺序约束：`interfaces → skill_sdk → common / sensor / body → skill_core → skills`。`skill_sdk` 可单独 `pip install -e` 供纯 Python 单测。

边界铁律：colcon 产物只出现在 `.colcon/`；`python/` 与 `core/` 不互相 import。

---

## 附录 B 配置分层

```
configs/
├── main.yaml            # includes 入口，决定生效套件
├── robot.yaml           # 机型：能力→区域映射、限幅、tick_rate
├── camera.yaml          # 相机内外参
├── scene/<site>.yaml    # 场地：导航点、有效空间、阈值
└── errors.yaml          # 错误码 → 文案模板 + retryable
```

加载规则：

1. `main.yaml` 的 `includes` 递归合并，后者覆盖前者（沿用参考实现 `ConfigLoader`）。
2. 技能私有配置（技能包根目录 `configs/skills/<skill>.yaml`）**不参与 include 合并**，装配时单独加载，仅经 `ctx.cfg` 暴露给本技能。
3. 场地差异只改 `scene/`，不改代码。
4. `errors.yaml` 是全局唯一文案来源，技能内不硬编码面向用户的文案。

---

## 附录 C 部署与运维

### C.1 部署分层

机器人只安装三类内容，**不安装开发框架**（`docs/`、`tests/`、`tools/`、源码构建环境）：

| 层 | 内容 | 安装方式 | 更新频率 |
|---|---|---|---|
| 运行时 | `core/`（6 包）+ `robots/<model>` | 一次性 | 低 |
| 技能 | `skills/<类别>/<技能>` | 每技能独立包 | 高 |
| 配置 | `configs/`（`robot.yaml` / `errors.yaml` / `scene/`） | 随运行时 + 现场改 | 中 |

### C.2 机器人目录布局

```
~/thinker/
├── runtime/
│   ├── install/               # core + robots 的 colcon install 树
│   └── VERSION
├── skills/                    # 已装技能，一技能一目录（可多版本）
│   ├── look_down_table/
│   │   ├── 0.1.0/
│   │   └── current -> 0.1.0
│   └── fetch_object_mock/
│       ├── 0.1.0/
│       └── current -> 0.1.0
├── configs/
└── logs/
```

### C.3 装配形态与启动入口

| 形态 | 命令 | 适用 |
|---|---|---|
| standalone | `ros2 run thinker_skill_core thinker_host`（`THINKER_SKILLS_DIR` 指向单技能） | 单技能、故障隔离 |
| composed | `ros2 run thinker_skill_core thinker_host`（`THINKER_SKILLS_DIR` 指向全部） | 组合部署：一个 host 共享控制器与状态 |

`thinker_host` 的部署路径由环境变量决定：

| 变量 | 默认 | 说明 |
|---|---|---|
| `THINKER_CONFIG_DIR` | `configs` | 配置目录 |
| `THINKER_SKILLS_DIR` | `skills` | 技能搜索路径，`:` 分隔多路径 |
| `THINKER_ROBOT_DIR` | **自动解析** | 机型能力声明目录：显式路径存在则用，否则按 `ament_index` 找 `<model>_robot` 的 `share/`（机器人上无源码目录时无需手配） |

技能发现用 `os.walk(followlinks=True)`，因此 `skills/<skill>/current -> <version>` 软链可被扫描；存在 `current` 时**只扫 `current`**，避免多版本共存时重复发现。

### C.4 运行时安装

运行时（core + robots）只在机器人上安装一次：

```bash
# 机器人上源码构建（同架构、同 distro）
rsync -av --exclude='.colcon' --exclude='.git' --exclude='docs' --exclude='tests' \
  dev:~/Thinker-Skill/ ~/thinker/src/
cd ~/thinker/src
source /opt/ros/humble/setup.bash
make build
mv .colcon/install ~/thinker/runtime/install
```

或由同架构构建机产出 `thinker-runtime-<ver>-<distro>-aarch64.tar.gz`，机器人上解到 `~/thinker/runtime/`。

> 关键：不能在开发机（x86_64）构建后拷到 Jetson（aarch64），架构与 distro 必须匹配。

### C.5 技能安装

技能是独立交付单元，装到 `~/thinker/skills/`：

```bash
# 构建机（同架构）只构建该技能
colcon build --packages-select look_down_table --install-base dist/look_down_table/0.1.0
tar czf look_down_table-0.1.0-humble-aarch64.tar.gz -C dist/look_down_table/0.1.0 .

# 机器人
mkdir -p ~/thinker/skills/look_down_table/0.1.0
tar xzf look_down_table-0.1.0-humble-aarch64.tar.gz -C ~/thinker/skills/look_down_table/0.1.0
ln -sfn 0.1.0 ~/thinker/skills/look_down_table/current
```

技能包必须包含：`share/<skill>/skill.yaml`、Python 包、`mcp_interface/`（若暴露 MCP）。

> 走 MCP 的技能需先 `pip install -r mcp_interface/requirements.txt`；只走 ROS Service/Action 的不需要 `fastmcp`。

### C.6 启动服务

```ini
# /etc/systemd/system/thinker-host.service
[Unit]
Description=Thinker-Skill host
After=network.target

[Service]
Environment=THINKER_SKILLS_DIR=/home/liu/thinker/skills
Environment=THINKER_CONFIG_DIR=/home/liu/thinker/configs
ExecStart=/usr/bin/bash -lc "source /opt/ros/humble/setup.bash && \
  source ~/thinker/runtime/install/setup.bash && \
  ros2 run thinker_skill_core thinker_host"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now thinker-host
journalctl -u thinker-host -f
```

### C.7 升级与回滚

| 操作 | 做法 |
|---|---|
| 升级技能 | 解新版本目录 → 校验 → `current` 指向新版本 → 重启 host |
| 回滚技能 | `current` 指回旧版本 → 重启 host（秒级） |
| 卸载技能 | 删除版本目录；若是当前版本，先切走软链再删 |
| 升级运行时 | 安装新 runtime → 重启 host；技能无需重装（`min_core` 满足即可） |

升级准入：

| 场景 | 处理 |
|---|---|
| 目标技能正在执行 | 拒绝升级，返回 `PRECOND_BODY_BUSY` |
| 新版本 `min_core` 高于当前 host | 拒绝安装，提示先升级 host |
| 新版本 `requires` 能力缺失 | 允许安装，装配时跳过并告警 |
| 新旧 `skill.yaml` 的 `name` 不一致 | 拒绝——目录名必须等于技能名 |
| 装完自检失败 | 软链切回旧版本 + 重启 host |

### C.8 与平台注册的关系

| 动作 | 生效范围 |
|---|---|
| 机器人侧安装 | 技能可被 host 装配，ROS Action / Service 可用 |
| 平台侧注册（上传 `metadata.json`） | Agent 的 MCP 工具列表可见 |

两条线独立：只装不注册 → Agent 看不到工具；只注册不装 → 工具在列表但调用失败。上线检查必须两者都过。

### C.9 不做热加载

安装、升级、卸载后都必须重启 host。技能发现在 host 启动期一次性构建 registry，MCP 客户端 `mcp_cfg` 同样需重启。这是当前版本的明确限制。

---

## 附录 D 调试与排查

| 现象 | 排查顺序 |
|---|---|
| 技能没被发现 | `share/<pkg>/skill.yaml` 是否存在 → ament index marker → host 启动日志「跳过/告警」原因 |
| MCP 工具调用 5s 超时 | 聚合器是否在跑 → 端口是否被占 → host 是否装配该技能 |
| 一直返回 `PRECOND_BODY_BUSY` | `body_arbiter` 持有者是谁 → 是否有未释放租约 → 看门狗是否触发过 |
| 有 result 但收不到 `feedback` | `Step.name` 是否为空 → Action server 是否在同一 callback group |
| 改配置不生效 | `ctx.cfg` 加载的是哪个 `scene/` → `main.yaml` 的 `includes` |
| MCP 工具 schema 缺字段 | 函数是否缺类型注解 → 文件是否被 Cython 编译（`mcp_interface.py` 必须明文） |
| 动作执行到一半停住 | 力监控是否触发卡死保护 → 单步 `run` 是否超过 `tick_budget` |
| Action 能调、MCP 看不到工具 | `requirements.txt` 是否装进 venv → 平台侧 `metadata.json` 是否已注册 |

```bash
ros2 service call /thinker/skills ...                        # 列出已装技能
ros2 action send_goal /thinker/skill/fetch_object/action ... # 手动下发
python testbench/print_mcp_tools.py -c <yaml> --schema       # 校验工具 schema
```

---

## 附录 E 验收标准

目标指标见 1.1（G1~G6）。技术验收清单：

- [ ] 每个命令：成功 `succeed` + `code=OK` + 业务字段；失败 `failed` + 枚举 `code` + 中文 `msg` + `retryable`，无异常栈
- [ ] `skill.yaml.errors` 覆盖实际产出的全部 `code`，且都在 `configs/errors.yaml` 有文案
- [ ] 服务未启动时调用：MCP 层 5s 超时返回 failed，不挂死
- [ ] 执行中取消三路都生效：Action `cancel`、MCP `notifications/cancelled`、看门狗超时
- [ ] **接口同构**：同一技能经 MCP 与 Action 调用，`state`/`msg` 语义一致
- [ ] 每个 `Step.name` 唯一，且完整出现在 `feedback.phase` 序列
- [ ] 单步 `run` 超过 `tick_budget` 时 Pipeline 告警
- [ ] import 白名单通过：只出现 `thinker_skill_sdk` 与消息包
- [ ] 无 ROS、无真机用 `FakeBody`/`FakeSensor` 跑通全步骤
- [ ] 编排器经聚合端口调用成功，无需感知技能数量与端口
- [ ] **孤儿任务回收**：MCP 300s 超时断开后，看门狗软急停，身体在 `1.2×timeout` 内释放
- [ ] 同机 ≥2 技能：服务/Action/会话名零冲突；卸载后其余正常
- [ ] `metadata.json` 的 tools、`mcp_interface.py` 函数、`skill.yaml` 的 commands 三者一致（自动化校验）
- [ ] docstring 盲测：仅凭工具描述可正确选择工具并传参
- [ ] `open_cmd_waiting=false` 无人值守跑通
- [ ] 新增类别下技能后，`core` 包数与代码零变化

---

## 附录 F 与参考三包的实现映射

便于熟悉 `kaiwu_agent/{detect,fetch,place}_object` 的读者定位：

| 参考实现 | 本设计 | 说明 |
|---|---|---|
| `mcp_move_box/move_box_service.py` | `core/skill_core/frontends` | 命令白名单 + dispatcher 通用化 |
| `arch/pipeline.py` | `core/skill_core/pipeline.py` | 10Hz 循环改为驱动 `Step` 列表 |
| `manipulation/action_factory.py`（基类） | `core/skill_sdk` + `core/skill_core` | 拆为「类型契约」与「驱动实现」 |
| `grasp_action_factory.py`（1855 行） | `skills/<类别>/<技能>/` | 每条业务链一个技能包 |
| `motion/`、`navigate/`、`tf_nodes.py` | `core/body` | 实现 `BodyAPI`，技能不再持有控制器 |
| `pose_estimate/` | `core/sensor` + 技能包 | 数据通道下沉，算法随技能 |
| `common_utils/` | `core/common` | 一份，不再复制三份 |
| `mcp_srvs/`、`boxdetection_msgs/` | `core/interfaces` | 增加 `Skill.action` |
| `mcp_move_box/configs/` | `configs/` + 技能包私有配置 | 去掉全局单例 |
| 顶层 `mcp_interface/` | 技能目录 `mcp_interface/` + `python/thinker_mcp_bridge` | 桥接收敛，壳只留工具函数 |
| 顶层 `start.sh`（三包互杀） | 技能包 `start.sh` + 部署公共片段 | 会话名带技能名 |
| `DESIGN.md` | `docs/` 架构文档 + 技能包 `DESIGN.md` | 架构级与技能级分离 |

---

## 附录 G 相对参考实现的取舍

### G.1 继承下来的设计

| 参考实现 | 保留原因 |
|---|---|
| 统一信封（一个 service 承载全部命令） | 新增技能不改协议 |
| 四层链路（桥接 → 分发 → 编排 → 执行） | 分层职责清晰 |
| 感知与运控解耦（独立进程 + 话题 + 缓冲） | 感知可独立启停与回放 |
| 配置驱动 + 文案可配 | `return_msgs` → `errors.yaml` |
| 失败不抛异常 | 收敛为 `{state, code, msg}` |

### G.2 明确改掉的

| 参考实现 | 问题 | 本设计 |
|---|---|---|
| 三包逐字节同源 | 一个 bug 改三处 | 平台代码入 `core`，只写一份 |
| 服务名 / 会话名相同 | 同机只能跑一个 | `/thinker/skill/<skill>/…` 命名空间隔离 |
| 只有同步 Service | 无进度、无取消、无抢占 | 增加 `Skill.action` |
| 描述串前缀兼作路由 | 文案与流程耦合 | `Step.kind` 枚举 |
| 全局 `config` 单例 | 跨技能参数污染 | `ctx.cfg` 只给本技能段 |
| `objPose` 等不生效参数 | 误导 LLM | 不暴露 |
| `input()` 阻塞点 | 无法无人值守 | `open_cmd_waiting: false` 为默认 |
