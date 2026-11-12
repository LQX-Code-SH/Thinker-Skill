#!/usr/bin/env bash
# 安装技能离线包到 <根>/skills/<skill>/<version>/，并切换 current 软链
# 用法：tools/install_skill.sh <pkg.tar.gz> [安装根目录，默认 ~/thinker]
set -euo pipefail

TARBALL="${1:?用法: tools/install_skill.sh <pkg.tar.gz> [安装根目录]}"
ROOT_DIR="${2:-$HOME/thinker}"

BASE="$(basename "$TARBALL" .tar.gz)"
SKILL="$(echo "$BASE" | cut -d- -f1)"
VERSION="$(echo "$BASE" | cut -d- -f2)"

DEST="$ROOT_DIR/skills/$SKILL/$VERSION"
rm -rf "$DEST"
mkdir -p "$DEST"
tar xzf "$TARBALL" -C "$DEST" --strip-components=1

ln -sfn "$VERSION" "$ROOT_DIR/skills/$SKILL/current"

echo "安装完成: $SKILL $VERSION"
echo "  目录: $DEST"
echo "  软链: $ROOT_DIR/skills/$SKILL/current -> $VERSION"
echo "提示: 重启 thinker-host 生效"
