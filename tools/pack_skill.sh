#!/usr/bin/env bash
# 打包单个技能为离线包：<skill>-<ver>-<distro>-<arch>.tar.gz
# 用法：tools/pack_skill.sh <skill> [输出目录]
# 前置：runtime 已构建并 source（技能依赖 thinker_skill_sdk）
# 权重：skill.yaml 的 resources.weights 列出的目录会一并打进包（部署后位于 <ver>/weights/）
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

# 按 skill.yaml 的 resources.weights 打包权重
WEIGHT_LIST="$(mktemp)"
trap 'rm -f "$WEIGHT_LIST"' EXIT
python3 - "$SKILL_DIR/skill/skill.yaml" > "$WEIGHT_LIST" <<'PY'
import sys

import yaml

data = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
for path in (data.get("resources") or {}).get("weights") or []:
    print(path)
PY
while IFS= read -r rel; do
  [ -n "$rel" ] || continue
  src="$ROOT/$rel"
  if [ ! -d "$src" ]; then
    echo "缺少权重目录: $src" >&2
    exit 1
  fi
  echo "打包权重: $rel ($(du -sh "$src" | cut -f1))"
  mkdir -p "$STAGE/$(dirname "$rel")"
  cp -a "$src" "$STAGE/$rel"
done < "$WEIGHT_LIST"

# 清理构建/下载缓存，避免打进包
find "$STAGE" -type d \( -name __pycache__ -o -name .cache \) -prune -exec rm -rf {} +

tar czf "$STAGE.tar.gz" -C "$OUT_DIR" "$(basename "$STAGE")"
rm -rf "$STAGE"

echo "打包完成: $OUT_DIR/$SKILL-$VERSION-$DISTRO-$ARCH.tar.gz ($(du -sh "$OUT_DIR/$SKILL-$VERSION-$DISTRO-$ARCH.tar.gz" | cut -f1))"
