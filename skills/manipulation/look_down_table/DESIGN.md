# look_down_table

低头看桌面：把头部 **pitch** 关节转到指定角度（默认 45°）。

| 项 | 内容 |
|---|---|
| 依赖能力 | `body.head` |
| 参数 | `pitch_deg`（可选，默认 45；范围 -15°~58°） |
| 步骤 | 低头看桌面（幂等下发目标角） |
| 后置状态 | 无 |
| 私有配置 | `max_pitch_deg`（真机软上限，默认 58）、`tol_deg`（到位容差，默认 3°）、`settle_ticks`（连续稳定 tick 数，默认 3） |

安全：角度按 SDK `joints.yaml` 的 head_pitch 限位 `[-15°, 58°]` 夹紧，超出自动截断。

真机实测（2026-11-12）：头部低头目标 50°/58° 均停在 **0.829 rad ≈ 47.5°**，即机械稳态上限低于 SDK 表；同时存在约 2.5° 稳态误差。故用 `max_pitch_deg` 做软限幅、`tol_deg` 放宽到位判据，避免误判 `SYS_TIMEOUT`。返回 `data` 同时给出 `requested_pitch_deg`（原始请求）与 `pitch_deg`（限幅后目标）。
