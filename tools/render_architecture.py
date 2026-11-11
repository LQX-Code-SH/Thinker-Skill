#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 Thinker-Skill 系统架构图（docs/assets/系统架构图.png）。

用法:
    python3 tools/render_architecture.py
"""

import os

from PIL import Image, ImageDraw, ImageFont

W, H = 1800, 1120
FONT_REG = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT_BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
SC = 2

# ---------- 配色 ----------
INK = "#1F2933"
SUB = "#5B6773"
LINE = "#C9D3DF"
BLUE_D = "#2F5D94"
BLUE = "#3F7FC1"
BLUE_L = "#E8F1FA"
BLUE_BG = "#F4F8FD"
GREEN = "#3E8E5A"
GREEN_BG = "#EDF7F0"
ORANGE = "#C97B1E"
ORANGE_BG = "#FDF3E4"
VIOLET = "#6B5BB5"
VIOLET_BG = "#F2F0FB"
GRAY_BG = "#F2F4F7"

_fonts = {}


def font(size, bold=False):
    size = int(round(size))
    key = (size, bold)
    if key not in _fonts:
        _fonts[key] = ImageFont.truetype(FONT_BOLD if bold else FONT_REG, size, index=SC)
    return _fonts[key]


def txt(d, xy, s, size, color=INK, bold=False, anchor="la"):
    d.text(xy, s, font=font(size, bold), fill=color, anchor=anchor)


def tw(s, size, bold=False):
    return font(size, bold).getlength(s)


def card(d, box, fill="#FFFFFF", outline=LINE, r=8, width=1):
    d.rounded_rectangle(box, radius=r, fill=fill, outline=outline, width=width)


def band(d, y, h, num, label, fill, chip=BLUE_D):
    """左侧编号色块 + 内容底板"""
    d.rounded_rectangle((56, y, 1196, y + h), radius=10, fill=fill)
    d.rounded_rectangle((56, y, 196, y + h), radius=10, fill=chip)
    txt(d, (126, y + h / 2 - 14), num, 22, "#FFFFFF", True, "mm")
    size = 12.5
    while size > 8 and tw(label, size) > 116:
        size -= 0.5
    txt(d, (126, y + h / 2 + 12), label, size, "#E8EEF6", False, "mm")


def arrow_down(d, x, y1, y2, color=BLUE):
    m = (y1 + y2) / 2
    d.line((x, y1, x, y2), fill=color, width=2)
    d.polygon(((x - 5, y1 + 4), (x + 5, y1 + 4), (x, y1 - 1)), fill=color)
    d.polygon(((x - 5, y2 - 4), (x + 5, y2 - 4), (x, y2 + 1)), fill=color)
    return m


def chevron_step(d, box, text, fill, outline):
    x0, y0, x1, y1 = box
    d.rounded_rectangle(box, radius=5, fill=fill, outline=outline, width=1)
    txt(d, ((x0 + x1) / 2, (y0 + y1) / 2), text, 13, INK, True, "mm")


def two_line_card(d, box, title, desc, title_color=INK, fill="#FFFFFF", outline=LINE,
                  tsize=14, dsize=11, dy=6):
    card(d, box, fill, outline)
    cx = (box[0] + box[2]) / 2
    cy = (box[1] + box[3]) / 2
    txt(d, (cx, cy - dy), title, tsize, title_color, True, "mm")
    txt(d, (cx, cy + dy + 4), desc, dsize, SUB, False, "mm")


d = ImageDraw.Draw(Image.new("RGB", (W, H)))


def main():
    img = Image.new("RGB", (W, H), "#FFFFFF")
    d = ImageDraw.Draw(img)

    # ---------- 标题 ----------
    txt(d, (W / 2, 44), "Thinker-Skill 系统架构图", 34, INK, True, "mm")
    txt(d, (W / 2, 82),
        "机器人技能封装库 · 按类别分组 · 每个技能可独立安装与组合 · 编排在外部主机完成，部署后在本体执行", 15, SUB, False, "mm")

    # ---------- 0. 应用层 ----------
    y, h = 128, 78
    band(d, y, h, "0", "应用层", GRAY_BG, "#5B6773")
    items = [("LLM Agent（大脑）", "按工具名与描述调用"),
             ("工作流编排", "按流程编排 MCP 或 Action"),
             ("现场调试", "ros2 service / action call")]
    cw = (990 - 40) / 3
    for i, (t, s) in enumerate(items):
        x = 210 + i * (cw + 20)
        two_line_card(d, (x, y + 14, x + cw, y + h - 14), t, s)

    # ---------- 1. 服务层 ----------
    y, h = 222, 148
    band(d, y, h, "1", "服务层", BLUE_BG, BLUE_D)
    txt(d, (210, y + 12), "两条通用接口（MCP / Action）+ 一条调试通道 —— 按语义需求选择，不绑定调用方", 12, BLUE_D, True)
    items = [("MCP Server", "streamable-http · 聚合 8000 / 技能 8022+", "工具 docstring 即 LLM prompt"),
             ("ROS Action Server", "Skill.action", "goal → feedback → cancel / 抢占"),
             ("ROS Service", "ControlCommand", "调试与旧链路兼容通道")]
    for i, (t, s1, s2) in enumerate(items):
        x = 210 + i * (cw + 20)
        box = (x, y + 32, x + cw, y + 92)
        card(d, box, "#FFFFFF", "#B9CFE6")
        cx = (box[0] + box[2]) / 2
        txt(d, (cx, y + 50), t, 15, BLUE_D, True, "mm")
        txt(d, (cx, y + 70), s1, 11, SUB, False, "mm")
        txt(d, (cx, y + 84), s2, 11, SUB, False, "mm")
    box = (210, y + 100, 1184, y + h - 10)
    card(d, box, "#FFFFFF", "#B9CFE6")
    txt(d, (228, (box[1] + box[3]) / 2), "统一信封协议", 13, BLUE_D, True, "lm")
    txt(d, (350, (box[1] + box[3]) / 2),
        "请求  { command, params }      →      响应  { state, code, msg, retryable, data }", 13, INK, False, "lm")
    txt(d, (980, (box[1] + box[3]) / 2), "新技能只加命令名，不改协议", 11, ORANGE, True, "lm")

    # ---------- 2. 执行层 ----------
    y, h = 386, 216
    band(d, y, h, "2", "执行层", "#EEF4FB", BLUE_D)
    txt(d, (210, y + 10), "core/skill_core", 13, BLUE_D, True)
    txt(d, (330, y + 10), "技能发现 · 装配 · 执行 · 状态，全部技能共用一套", 12, SUB)

    # 左：组件
    box = (206, y + 30, 660, y + h - 12)
    card(d, box, "#FFFFFF", "#B9CFE6")
    txt(d, (box[0] + 14, box[1] + 12), "执行层组件", 13, BLUE_D, True)
    comps = [("技能发现", "扫描 share/*/skill.yaml，装了就出现"),
             ("身体仲裁", "body_arbiter：头/上肢/下肢区域互斥 / 排队 / 抢占"),
             ("状态存储", "state_store：holding · map_ready"),
             ("注册表", "registry：skill_name → factory"),
             ("Pipeline", "声明式 steps · 10Hz · 软急停 · 失败中止")]
    row_h = (box[3] - box[1] - 40) / len(comps)
    for i, (n, s) in enumerate(comps):
        ry = box[1] + 34 + i * row_h
        d.ellipse((box[0] + 14, ry + 5, box[0] + 22, ry + 13), fill=BLUE)
        txt(d, (box[0] + 30, ry), n, 13, INK, True)
        txt(d, (box[0] + 30 + tw(n, 13, True) + 10, ry + 1), s, 11, SUB)

    # 右：执行链
    box = (672, y + 30, 1184, y + h - 12)
    card(d, box, "#FFFFFF", "#B9CFE6")
    txt(d, (box[0] + 14, box[1] + 12), "单次技能调用执行链", 13, BLUE_D, True)
    steps = [("发现", "读取 skill.yaml 并注册"),
             ("校验", "min_core 版本 / requires 能力；不满足则跳过，不阻塞其他技能"),
             ("装配", "构造 factory，注册到 registry"),
             ("执行", "Pipeline 驱动动作链，逐阶段回传 feedback"),
             ("写回", "state_store 写入 holding 等前置状态")]
    sy = box[1] + 38
    sh = (box[3] - sy - 10) / len(steps)
    for i, (n, s) in enumerate(steps):
        cy = sy + i * sh
        chevron_step(d, (box[0] + 14, cy + 2, box[0] + 90, cy + sh - 6), n, BLUE_L, "#B9CFE6")
        txt(d, (box[0] + 102, cy + 3), s, 11, SUB)
        if i < len(steps) - 1:
            d.line((box[0] + 52, cy + sh - 4, box[0] + 52, cy + sh + 1), fill=BLUE, width=2)

    # ---------- 3. 技能层 ----------
    y, h = 618, 176
    band(d, y, h, "3", "技能层", GREEN_BG, GREEN)
    txt(d, (210, y + 10), "skills/<类别>/<技能>/", 13, GREEN, True)
    txt(d, (378, y + 10), "每个技能目录 = 独立可安装的 ament 包（skill.yaml + package.xml + DESIGN.md + mcp_interface）",
        12, SUB)
    cats = [("导航移动类", "navigate_to · follow_person · return_home"),
            ("操作类", "fetch_object · place_object · handover"),
            ("视觉感知类", "detect_object · locate_person"),
            ("拍照类", "take_photo · record_video"),
            ("语音对话类", "speak · ask_and_confirm"),
            ("场景类", "serve_drink · v1.1 后置")]
    cw3 = (990 - 40) / 3
    for i, (t, s) in enumerate(cats):
        r, c = divmod(i, 3)
        x = 210 + c * (cw3 + 20)
        yy = y + 34 + r * 66
        two_line_card(d, (x, yy, x + cw3, yy + 58), t, s, GREEN, "#FFFFFF", "#BFDCC8", 14, 10, 7)

    # ---------- 4. 基础能力层 ----------
    y, h = 810, 122
    band(d, y, h, "4", "基础能力层", "#F7F5EC", "#8A7A3B")
    txt(d, (210, y + 10), "core/", 13, "#8A7A3B", True)
    txt(d, (258, y + 10), "共享实现 —— 只写一份；技能只依赖 core/skill_sdk，其余由执行层装配", 12, SUB)
    caps = [("core/skill_sdk", "Step · StepCtx · Protocol", "技能作者唯一依赖"),
            ("core/body", "控制器 · QP · moveit", "腰/头/手爪/导航/tf"),
            ("core/sensor", "相机 · RGB-D · TF", "仅数据通道，算法在技能"),
            ("core/common", "DataBuffer · Transform", "TTS · ConfigLoader"),
            ("core/interfaces", "ControlCommand.srv", "Skill.action · msg")]
    cw4 = (990 - 40) / 5
    for i, (t, s1, s2) in enumerate(caps):
        x = 210 + i * (cw4 + 10)
        box = (x, y + 32, x + cw4, y + h - 12)
        card(d, box, "#FFFFFF", "#DCD3B4")
        cx = (box[0] + box[2]) / 2
        txt(d, (cx, y + 50), t, 11, "#8A7A3B", True, "mm")
        txt(d, (cx, y + 70), s1, 10, INK, False, "mm")
        txt(d, (cx, y + 86), s2, 9.5, SUB, False, "mm")

    # ---------- 5. 机器人本体层 ----------
    y, h = 948, 88
    band(d, y, h, "5", "机器人本体层", GRAY_BG, "#5B6773")
    hw = [("机器人本体", "天轶 2.0"), ("RGB-D 相机", "Orbbec"), ("激光雷达", "—"),
          ("麦克风阵列", "语音 I/O"), ("机械臂 · 夹爪", "双臂 + 六维力"), ("底盘 · 导航", "移动平台")]
    cw6 = (990 - 50) / 6
    for i, (t, s) in enumerate(hw):
        x = 210 + i * (cw6 + 10)
        two_line_card(d, (x, y + 16, x + cw6, y + h - 16), t, s, INK, "#FFFFFF", LINE, 12, 10, 7)

    # ---------- 层间箭头 ----------
    for y1, y2 in ((206, 222), (370, 386), (602, 618), (794, 810), (932, 948)):
        arrow_down(d, 626, y1 + 2, y2 - 2)

    # ---------- 右侧：系统特性 ----------
    px0, px1 = 1216, 1744
    box = (px0, 128, px1, 470)
    card(d, box, "#FFFFFF", LINE)
    txt(d, ((px0 + px1) / 2, 152), "系统特性", 17, INK, True, "mm")
    d.line((px0 + 20, 168, px1 - 20, 168), fill="#E3E9F0", width=1)
    feats = [("独立安装", "一个技能 = 一个 ament 包，装/卸不动 core"),
             ("可组合", "装 N 个技能，服务/Action 名零冲突"),
             ("区域仲裁", "同区域串行 · 跨区域并行 · 只读豁免"),
             ("长任务可控", "阶段反馈 · 可取消 · 可抢占"),
             ("失败可判", "code 给程序 · msg 给 LLM 播报"),
             ("双接口一致", "MCP 与 Action 共用同一执行内核"),
             ("契约一致", "metadata / mcp_interface / skill.yaml 三方对齐")]
    fy = 182
    for n, s in feats:
        d.ellipse((px0 + 20, fy + 6, px0 + 28, fy + 14), fill=BLUE)
        txt(d, (px0 + 38, fy), n, 13, INK, True)
        txt(d, (px0 + 38, fy + 20), s, 11, SUB)
        fy += 42

    # ---------- 右侧：闭环 ----------
    box = (px0, 486, px1, 742)
    card(d, box, "#FFFFFF", LINE)
    txt(d, ((px0 + px1) / 2, 510), "技能闭环", 17, INK, True, "mm")
    nodes = {"感知": (px0 + 60, 540), "决策": (px0 + 290, 540),
             "执行": (px0 + 290, 638), "反馈": (px0 + 60, 638)}
    for n, (nx, ny) in nodes.items():
        card(d, (nx, ny, nx + 152, ny + 52), BLUE_L, "#B9CFE6")
        txt(d, (nx + 76, ny + 26), n, 14, BLUE_D, True, "mm")
    ar = ORANGE
    d.line((px0 + 216, 566, px0 + 286, 566), fill=ar, width=2)
    d.polygon(((px0 + 286, 566), (px0 + 277, 561), (px0 + 277, 571)), fill=ar)
    d.line((px0 + 366, 596, px0 + 366, 634), fill=ar, width=2)
    d.polygon(((px0 + 366, 634), (px0 + 361, 625), (px0 + 371, 625)), fill=ar)
    d.line((px0 + 286, 664, px0 + 216, 664), fill=ar, width=2)
    d.polygon(((px0 + 216, 664), (px0 + 225, 659), (px0 + 225, 669)), fill=ar)
    d.line((px0 + 136, 634, px0 + 136, 596), fill=ar, width=2)
    d.polygon(((px0 + 136, 596), (px0 + 131, 605), (px0 + 141, 605)), fill=ar)
    txt(d, ((px0 + px1) / 2, 710), "感知取数 → 决策 → 执行 → 状态回写", 11, SUB, False, "mm")

    # ---------- 右侧：图例 ----------
    box = (px0, 758, px1, 1036)
    card(d, box, "#FFFFFF", LINE)
    txt(d, ((px0 + px1) / 2, 782), "图例", 17, INK, True, "mm")
    d.line((px0 + 20, 798, px1 - 20, 798), fill="#E3E9F0", width=1)
    ly = 816
    legend = [(BLUE, "MCP 调用（任意 MCP 客户端 → 服务层）"),
              (ORANGE, "Action goal / feedback / cancel"),
              ("#8A7A3B", "ROS topic / service"),
              (LINE, "包依赖（package.xml exec_depend）")]
    for color, label in legend:
        d.line((px0 + 24, ly + 6, px0 + 60, ly + 6), fill=color, width=3)
        txt(d, (px0 + 70, ly - 2), label, 11, SUB)
        ly += 26
    txt(d, (px0 + 24, ly + 8), "层次约束", 12, INK, True)
    notes = ["服务层不写业务逻辑", "技能只 import skill_sdk，不互相 import",
             "运控技能必须经身体仲裁", "core 只放通用能力，包数不随技能增长"]
    ny = ly + 28
    for n in notes:
        txt(d, (px0 + 24, ny), "· " + n, 11, SUB)
        ny += 20

    # ---------- 页脚 ----------
    txt(d, (W / 2, 1074), "Thinker-Skill v0.1 · 由 kaiwu_agent 三包设计演进：保留四层链路与统一信封，去掉逐字节复制",
        12, "#8A96A3", False, "mm")

    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "docs", "assets", "系统架构图.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    img.save(out, "PNG")
    print("saved:", out)


if __name__ == "__main__":
    main()
