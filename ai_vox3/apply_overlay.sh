#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# AI-VOX3 桌面小跟班 —— 公共仓改动覆盖脚本
#
# 背景：本作品对 openvela 公共仓（nuttx / apps）做了大量底层修复
#       （I2S 全双工时钟、GDMA 完成队列、IOB 池、ES8311、ST7789、WiFi adapter 等）。
#       按大赛规定，公共仓改动需 fork + PR 由组委会 review 合入，不能直接 push；
#       因此这些改动在本仓以快照形式提供：ai_vox3/{nuttx,apps,packages_ai_agent}。
#       repo sync 只映射了板级目录（vendor/openvela/boards/...），
#       内核侧与应用层补丁需要执行本脚本覆盖，否则编译出的固件缺少语音链路修复。
#
# 用法（在 openvela 工作区根目录，即含 nuttx/ apps/ packages/ 的那一层）：
#     bash contest2026_393_jizhipai/ai_vox3/apply_overlay.sh
#
# 也可显式指定工作区根目录：
#     bash .../apply_overlay.sh /path/to/openvela
# ---------------------------------------------------------------------------
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_ROOT="$(cd "$HERE/../.." && pwd)"
ROOT="${1:-$DEFAULT_ROOT}"

die() { echo "ERROR: $*" >&2; exit 1; }

[ -d "$ROOT/nuttx" ]    || die "找不到 $ROOT/nuttx —— 请在 openvela 工作区根目录执行，或把根目录作为第一个参数传入"
[ -d "$ROOT/packages" ] || die "找不到 $ROOT/packages"
[ -d "$ROOT/apps" ]     || die "找不到 $ROOT/apps"

copy_tree() {  # $1=src $2=dst  $3..=额外 rsync 排除项
  local src="$1" dst="$2"; shift 2
  [ -d "$src" ] || die "快照目录缺失：$src"
  mkdir -p "$dst"
  if command -v rsync >/dev/null 2>&1; then
    local ex=(); local e
    for e in "$@"; do ex+=(--exclude "$e"); done
    rsync -a "${ex[@]}" "$src"/ "$dst"/
  else
    ( cd "$src" && tar -cf - . ) | ( cd "$dst" && tar -xf - )
  fi
}

echo "== AI-VOX3 overlay =="
echo "   快照源: $HERE"
echo "   工作区: $ROOT"

n_before=$(find "$HERE/nuttx" -type f 2>/dev/null | wc -l)
copy_tree "$HERE/nuttx" "$ROOT/nuttx" ".config"
echo "   nuttx   : 覆盖完成（快照 $n_before 个文件；.config 保留工作区现有配置）"

copy_tree "$HERE/apps" "$ROOT/apps"
echo "   apps    : 覆盖完成（netinit / wapi）"

copy_tree "$HERE/packages_ai_agent" "$ROOT/packages/ai_agent" ".built" ".depend"
echo "   ai_agent: 覆盖完成"

echo
echo "下一步："
echo "   ./build.sh vendor/openvela/boards/contest2026_393_esp32s3_eye/configs/ai_agent -j8"
echo "   产物 nuttx/nuttx.bin，烧录写入 0x0（不要动 0x9000 的 NVS）"
