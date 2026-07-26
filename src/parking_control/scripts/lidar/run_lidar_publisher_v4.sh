#!/usr/bin/env bash
# Isaac Sim PC용: 최신 v4 천장 LiDAR 2대를 ROS 2 PointCloud2로 발행한다.
set -u

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../../.." && pwd)"
CAPTURE_SCRIPT="$SCRIPT_DIR/capture_lidar.py"
V4_STAGE="$REPO_ROOT/isaacpjt/Isaac_envo/parking/parking_environment_v4.usd"
ISAAC_REL="${ISAAC_SIM_ROOT:-$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release}"

if [[ ! -f "$V4_STAGE" ]]; then
    echo "최신 v4 USD를 찾을 수 없습니다: $V4_STAGE" >&2
    exit 1
fi
if [[ ! -x "$ISAAC_REL/python.sh" ]]; then
    echo "Isaac Sim python.sh를 찾을 수 없습니다: $ISAAC_REL/python.sh" >&2
    exit 1
fi

# 시스템 ROS(Python 3.10)가 Isaac Python 3.11에 섞이지 않게 정리한다.
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
export PYTHONUTF8=1
export PYTHONIOENCODING=utf-8
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-122}"
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$ISAAC_REL/exts/isaacsim.ros2.bridge/humble/lib"

headless_args=(--headless)
capture_args=()
for arg in "$@"; do
    if [[ "$arg" == "--gui" ]]; then
        headless_args=()
    else
        capture_args+=("$arg")
    fi
done

exec "$ISAAC_REL/python.sh" "$CAPTURE_SCRIPT" \
    --stage "$V4_STAGE" --with-v4-robots --with-v4-pedestrians --live --ros2 \
    "${headless_args[@]}" "${capture_args[@]}"
