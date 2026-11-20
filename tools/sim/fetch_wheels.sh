#!/usr/bin/env bash
# 用 curl 把技能镜像需要的 wheel 预下载到本地目录（默认 tools/sim/wheels，
# 构建时由 Dockerfile 指到 BuildKit cache mount 的 /wheels）。
#
# 为什么不用 pip/uv 直接下：
#   pip 只走 HTTP/1.1 且没有断点续传（900 MB 的 torch 断一次就整包重下）；
#   uv 在长构建里会挂在慢 CDN 节点（实测 0.7 MB/s，同源 curl 有 12–25 MB/s）。
#   curl 走 HTTP/2 + -C - 续传 + --retry，稳且可续。
#
# 用法：
#   make skill-build                       # 构建内自动调用（缓存到 /wheels）
#   WHEELHOUSE=/tmp/w tools/sim/fetch_wheels.sh   # 手动预下载备用
set -euo pipefail

IDX="${WHEEL_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"
BASE="${IDX%/simple}"
OUT="${WHEELHOUSE:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/wheels}"
mkdir -p "$OUT"

# 大件（torch + CUDA 运行库）写死版本；纯 Python 小依赖不写版本，取最新稳定版。
PKGS=(
  "torch==2.9.0"
  "torchvision==0.24.0"
  "triton==3.5.0"
  "nvidia-cuda-nvrtc-cu12==12.8.93"
  "nvidia-cuda-runtime-cu12==12.8.90"
  "nvidia-cuda-cupti-cu12==12.8.90"
  "nvidia-cudnn-cu12==9.10.2.21"
  "nvidia-cublas-cu12==12.8.4.1"
  "nvidia-cufft-cu12==11.3.3.83"
  "nvidia-curand-cu12==10.3.9.90"
  "nvidia-cusolver-cu12==11.7.3.90"
  "nvidia-cusparse-cu12==12.5.8.93"
  "nvidia-cusparselt-cu12==0.7.1"
  "nvidia-nccl-cu12==2.27.5"
  "nvidia-nvshmem-cu12==3.3.20"
  "nvidia-nvtx-cu12==12.8.90"
  "nvidia-nvjitlink-cu12==12.8.93"
  "nvidia-cufile-cu12==1.13.1.3"
)
# 只预下载"大件"。纯 Python 小依赖（sympy/mpmath/pillow/gdown/jinja2 …）
# 不钉版本：它们之间有约束关系（如 sympy 1.14 要求 mpmath<1.4），
# 钉死会互相冲突，交给 pip 在安装时解析，几 MB 而已。

# 从索引页里挑 wheel 的绝对下载地址；ver 为空时取最新版本
pick_url() {
  local page="$1" raw="$2" ver="$3" listing best versions
  listing="$(printf '%s' "$page" \
    | tr '<>' '\n\n' \
    | grep -ioE "href=\"[^\"]*${raw}-[^\"]*\.whl[^\"]*\"" \
    | grep -viE "aarch64|armv7l|arm64|macosx|win_|win32|musllinux" \
    | grep -iE "manylinux[^\"]*x86_64|py3-none-any" \
    | sed 's/^href="//I; s/"$//; s/#.*$//' \
    | sort -u)"
  [ -n "$listing" ] || return 1

  if [ -z "$ver" ]; then
    versions="$(printf '%s\n' "$listing" \
      | sed -E "s#^.*/${raw}-([^-]+)-.*#\1#" \
      | grep -E '^[0-9]+(\.[0-9]+)*$' | sort -V -u)"
    ver="$(printf '%s\n' "$versions" | tail -1)"
    [ -n "$ver" ] || return 1
    echo "（$raw 取最新稳定版 $ver）" >&2
  fi

  # 同一版本可能有多个平台/ABI 轮子：cp312 优先，其次 abi3，最后 py3-none-any
  best="$(printf '%s\n' "$listing" | grep -iE "/${raw}-${ver}-cp312-cp312" | head -1)"
  [ -n "$best" ] || best="$(printf '%s\n' "$listing" | grep -iE "/${raw}-${ver}-" | grep -i 'abi3' | head -1)"
  [ -n "$best" ] || best="$(printf '%s\n' "$listing" | grep -iE "/${raw}-${ver}-py3-none-any" | head -1)"
  [ -n "$best" ] || best="$(printf '%s\n' "$listing" | grep -iE "/${raw}-${ver}-" | head -1)"
  [ -n "$best" ] || return 1
  printf '%s' "$best" | sed "s#^\.\./\.\./#${BASE}/#"
}

fail=0
for spec in "${PKGS[@]}"; do
  if [[ "$spec" == *"=="* ]]; then
    name="${spec%%==*}"
    ver="${spec##*==}"
  else
    name="$spec"
    ver=""
  fi
  raw="$(printf '%s' "$name" | tr '[:upper:]' '[:lower:]' | tr '-' '_')"

  if [ -n "$ver" ]; then
    existing="$(ls "$OUT/${raw}-${ver}-"*.whl 2>/dev/null | head -1 || true)"
    if [ -n "$existing" ]; then
      echo "跳过（已存在）: $(basename "$existing")"
      continue
    fi
  fi

  page="$(curl -sfL "$IDX/${name}/" || true)"
  if [ -z "$page" ]; then
    echo "取索引失败: ${name}" >&2
    fail=1
    continue
  fi
  url="$(pick_url "$page" "$raw" "$ver" || true)"
  if [ -z "$url" ]; then
    echo "找不到 ${name}${ver:+==$ver} 的 x86_64 wheel" >&2
    fail=1
    continue
  fi

  out="$OUT/${url##*/}"
  [ -f "$out" ] && { echo "跳过（已存在）: ${url##*/}"; continue; }
  echo "下载 ${url##*/}"
  curl -fL --retry 10 --retry-delay 2 --retry-all-errors -C - -o "$out" "$url"
done

# 生成 requirements.txt：只列 PKGS 里的"大件"（按实际下载到的 wheel 反推版本）。
# 注意不能扫目录里所有 *.whl —— 缓存里可能有上一轮的旧小包（如 mpmath 1.4.1），
# 把它们写进 requirements 会和 sympy 1.14（要求 mpmath<1.4）冲突。
REQ="$OUT/requirements.txt"
: > "$REQ"
for spec in "${PKGS[@]}"; do
  [[ "$spec" == *"=="* ]] || continue
  name="${spec%%==*}"
  ver="${spec##*==}"
  raw="$(printf '%s' "$name" | tr '[:upper:]' '[:lower:]' | tr '-' '_')"
  f="$(ls "$OUT/${raw}-${ver}-"*.whl 2>/dev/null | head -1 || true)"
  [ -n "$f" ] || continue
  echo "${raw}==${ver}" >> "$REQ"
done
sort -o "$REQ" "$REQ"

# 清掉早期脚本版本留在缓存里的小包（它们会干扰 pip 的版本解析）
for n in sympy mpmath networkx jinja2 markupsafe filelock fsspec typing_extensions; do
  rm -f "$OUT/${n}-"*.whl
done

echo "--- 结果 ---"
ls -lh "$OUT" 2>/dev/null | tail -n +2 | awk '{print $5, $9}'
du -sh "$OUT"
echo "requirements: $REQ ($(wc -l < "$REQ") 条)"
exit "$fail"
