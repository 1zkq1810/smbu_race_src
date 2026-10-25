#!/usr/bin/env bash
# build_all.sh —— 超抗赛栈一键构建（只编译，不改任何源码）
# 用法： bash <本文件路径>
# 路径不写死：从脚本自身位置往上退四层得到工作区根
#   tools -> rmul2027_sentry_task -> smbu_race_src -> src -> 工作区根
set -e

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
echo "工作区根：$WS"
if [ ! -d "$WS/src" ]; then
  echo "[!] 推导出的路径里没有 src/ 目录，说明脚本放错位置或工作区结构不同，已停止。"
  exit 1
fi
cd "$WS"

colcon build \
  --symlink-install \
  --parallel-workers 1 \
  --packages-skip joint_state_publisher joint_state_publisher_gui \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo
echo "构建结束。使用前请执行： source $WS/install/setup.bash"
