# detect_object

感知技能：开集文本检测 → mask/框 3D 定位 → 输出 base 系位置（米）。

| 项 | 内容 |
|---|---|
| 依赖能力 | `sensor.rgbd`（只读，不占身体租约） |
| 参数 | `object_name`（默认 `bottle`） |
| 步骤 | 取帧 → 检测（GroundingDINO）→ 定位（深度反投影 + 光线延伸） |
| 前置 | 头部姿态由调用方负责（如先调 `look_down_table`），本技能不下发动作 |
| 输出 | `data: {object_name, obj_pose:[x,y,z], label, score, box, stamp}`，base 系、米 |
| 后置状态 | `last_detection`（含 object_name / obj_pose / label / score / box / stamp），供下游技能复用 |
| 私有配置 | `threshold` / `text_threshold`（默认 0.35） |
| 依赖 | `transformers>=4.55`（标准 GroundingDINO 权重，系统 5.0.0 已验证） |

## 算法（对齐参考实现）

1. mask 质心（无 mask 退化用框中心）→ 3×3 中值深度（mm × 1e-3）
2. 针孔反投影：`X=(u-cx)z/fx`、`Y=(v-cy)z/fy`、`Z=z`
3. 沿光线单位向量延伸 0.03 m
4. `cam→base` 变换（由 `sensor.transform_to_base` 提供，使用 **Frame.angles** 即取帧时刻的关节角）
5. 有效空间过滤：`x∈(0.2,0.8) y∈(-0.4,0.4) z∈(0.4,1.3)`

## 失败语义

| code | 场景 |
|---|---|
| `PERCEPTION_OBJECT_NOT_FOUND` | 未检测到目标（可重试） |
| `PERCEPTION_POSE_INVALID` | 深度无效 / 超出有效空间 / cam→base 不可用 |
| `SYS_CONFIG_ERROR` | 缺少相机内参 |

## 已知限制

- `cam→base` 由 `sensor.transform_to_base` 提供：tienkung_dex 用 URDF 正运动学（腰 3 轴 + 头 2 轴）+ `pelvis_height` 平移，非 TF
- 单帧决定结果，无时序融合；取帧时头部若仍在运动，`locate` 读到的角度与画面会不一致
- 权重随技能包安装到 `<skill>/<ver>/weights/`，由 `resources.weights` 声明
