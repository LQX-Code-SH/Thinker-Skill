#!/usr/bin/env bash
# 打包单个技能为离线包：<skill>-<ver>-<distro>-<arch>.tar.gz
# 用法：tools/pack_skill.sh <skill> [输出目录]
# 前置：runtime 已构建并 source（技能依赖 thinker_skill_sdk）
set -euo pipefail

SKILL="${1:?用法: tools/pack_skill.sh <skill> [输出目录]}"
OUT_DIR="${2:-dist}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

SKILL_DIR="$(find "$ROOT/skills" -maxdepth 3 -type d -name "$SKILL" | head -1)"
if [ -z "$SKILL_DIR" ]; then
  echo "未找到技能: $SKILL" >&2
  exit 1
fi

VERSION="$(python3 -c "import yaml;print(yaml.safe_load(open('$SKILL_DIR/skill/skill.yaml'))['version'])")"
DISTRO="${ROS_DISTRO:-humble}"
ARCH="$(uname -m)"

WORK="$ROOT/.pack/$SKILL"
rm -rf "$WORK"

echo "构建 $SKILL ..."
colcon --log-base "$WORK/log" build --packages-select "$SKILL" \
  --base-paths "$ROOT/skills" \
  --build-base "$WORK/build" \
  --install-base "$WORK/install"

mkdir -p "$OUT_DIR"
STAGE="$OUT_DIR/$SKILL-$VERSION-$DISTRO-$ARCH"
rm -rf "$STAGE"
mkdir -p "$STAGE"
cp -a "$WORK/install" "$STAGE/install"
[ -d "$SKILL_DIR/mcp_interface" ] && cp -a "$SKILL_DIR/mcp_interface" "$STAGE/mcp_interface"
[ -f "$SKILL_DIR/DESIGN.md" ] && cp "$SKILL_DIR/DESIGN.md" "$STAGE/"

tar czf "$STAGE.tar.gz" -C "$OUT_DIR" "$(basename "$STAGE")"
rm -rf "$STAGE"

echo "打包完成: $OUT_DIR/$SKILL-$VERSION-$DISTRO-$ARCH.tar.gz"
