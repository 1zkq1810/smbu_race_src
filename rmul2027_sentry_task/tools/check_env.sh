#!/usr/bin/env bash
# check_env.sh —— 工作区现状体检（只读：不创建、不修改、不删除任何文件）
# 用法： bash <本文件路径>
# 路径不写死：从脚本自身位置往上退四层得到工作区根
#   tools -> rmul2027_sentry_task -> smbu_race_src -> src -> 工作区根

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC=$WS/src/smbu_race_src

echo "======== 1. 时间 / 机器 / 路径推导 ========"
echo "工作区根 WS=$WS"
echo "脚本所在目录 SELF_DIR=$SELF_DIR"
if [ ! -d "$SRC" ]; then
  echo "[!] $SRC 不存在，说明 WS 推导错了，下面几段都会是空的，先停下修脚本。"
fi
date '+%Y-%m-%d %H:%M:%S %Z'
echo "user=$(whoami)  host=$(hostname)"

echo
echo "======== 2. ROS 环境 ========"
echo "ROS_DISTRO=${ROS_DISTRO:-<未设置>}"
echo "AMENT_PREFIX_PATH 逐段（顺序=优先级，靠前优先）："
echo "${AMENT_PREFIX_PATH:-<空>}" | tr ':' '\n' | sed 's/^/  /'

echo
echo "======== 3. 仓库状态 ========"
if [ -d "$SRC/.git" ]; then
  git -C "$SRC" log -1 --format='  本地 HEAD: %h  %s  (%ci)'
  printf '  远端 main: '
  git -C "$SRC" rev-parse origin/main 2>/dev/null || echo "<取不到>"
  echo "  未提交改动条数: $(git -C "$SRC" status --short | wc -l)"
else
  echo "  [!] $SRC 不是 git 仓库"
fi

echo
echo "======== 4. 包数 ========"
echo "  src 下 package.xml 个数: $(find "$WS/src" -name package.xml | wc -l)"
echo "  colcon 识别到的包个数:   $(colcon list --base-paths "$WS/src" 2>/dev/null | wc -l)"

echo
echo "======== 5. install/ 里实际存在的条目 ========"
ls -1 "$WS/install" 2>/dev/null | grep -v -E '^(COLCON_IGNORE|.colcon_install_layout|setup\.|local_setup\.|_local_setup)' | sed 's/^/  /'

echo
echo "======== 6. 关键包由谁提供 ========"
for p in point_lio small_gicp_relocalization pb_nav2_plugins pb_omni_pid_pursuit_controller standard_robot_pp_ros2 pb_rm_interfaces rmul2027_sentry_task joint_state_publisher; do
  printf '  %-32s ' "$p"
  ros2 pkg prefix "$p" 2>/dev/null || echo "未找到"
done

echo
echo "======== 7. 同名包遮蔽检查 ========"
if ros2 pkg prefix joint_state_publisher >/dev/null 2>&1; then
  P=$(ros2 pkg prefix joint_state_publisher)
  case "$P" in
    "$WS"*) echo "  [!] 被工作区遮蔽: $P" ;;
    *)      echo "  [ok] 来自系统: $P" ;;
  esac
else
  echo "  [!] 找不到（系统也没装）"
fi

echo
echo "======== 8. 实车资产（地图 / 先验点云）========"
D1="$WS/src/smbu_race_src/pb2027_sentry_nav/pb2025_nav_bringup/map/reality"
D2="$WS/src/smbu_race_src/pb2027_sentry_nav/pb2025_nav_bringup/pcd/reality"
echo "  map/reality 内容:"
ls -1 "$D1" 2>/dev/null | sed 's/^/    /' || echo "    目录不存在"
echo "  pcd/reality 内容:"
ls -1 "$D2" 2>/dev/null | sed 's/^/    /' || echo "    目录不存在"

echo
echo "======== 9. 一键构建脚本位置 ========"
if [ -f "$SELF_DIR/build_all.sh" ]; then echo "  [ok] 同目录 build_all.sh 存在: $SELF_DIR/build_all.sh"; else echo "  [!] 同目录没有 build_all.sh"; fi
if [ -f "$WS/build_all.sh" ]; then echo "  旧位置仍在（可留可删）: $WS/build_all.sh"; else echo "  旧位置 $WS/build_all.sh 已不存在"; fi

echo
echo "======== 体检结束（本脚本无任何写操作）========"
