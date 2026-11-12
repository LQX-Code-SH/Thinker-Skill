# detect_object

感知技能：开集文本检测 → mask/框 3D 定位 → 输出 base 系位置（米）。

| 项 | 内容 |
|---|---|
| 依赖能力 | `body.head`、`sensor.rgbd` |
| 参数 | `object_name`（默认 `bottle`）、`head_pitch_deg`（默认 30） |
| 步骤 | 低头 → 取帧 → 检测（GroundingDINO）→ 定位（深度反投影 + 光线延伸） |
| 输出 | `data: {object_name, obj_pose:[x,y,z], label, score}`，base 系、米 |

## 算法（对齐参考实现）

1. mask 质心（无 mask 退化用框中心）→ 3×3 中值深度（mm × 1e-3）
2. 针孔反投影：`X=(u-cx)z/fx`、`Y=(v-cy)z/fy`、`Z=z`
3. 沿光线单位向量延伸 0.03 m
4. `cam→base` 变换（由 `sensor.transform_to_base` 提供）
5. 有效空间过滤：`x∈(0.2,0.8) y∈(-0.4,0.4) z∈(0.4,1.3)`

## 失败语义

| code | 场景 |
|---|---|
| `PERCEPTION_OBJECT_NOT_FOUND` | 未检测到目标（可重试） |
| `PERCEPTION_POSE_INVALID` | 深度无效 / 超出有效空间 / cam→base 不可用 |
| `SYS_CONFIG_ERROR` | 缺少相机内参 |

## 已知限制

- `cam→base` 依赖 `sensor` 实现；tienkung_dex 当前无 TF，需配相机外参后才可用
- 单帧决定结果，无时序融合
