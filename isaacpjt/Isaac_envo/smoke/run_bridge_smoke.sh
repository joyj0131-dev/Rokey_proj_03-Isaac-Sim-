#!/bin/bash
# R2 브리지 스모크 런처 (터미널 B, 시스템 ROS 2 Humble).
# Isaac 러너(--bridge)와 같은 도메인/화이트리스트를 쓴다. run_probe_a_detector.sh
# 와 동일한 DDS 환경 설정 패턴.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# /opt/ros 의 setup.bash 는 미설정 변수를 참조하므로 nounset(set -u)을 켠 채
# 소싱하면 그 줄에서 스크립트가 죽는다. 소싱 동안만 nounset 을 끈다.
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-126}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$HOME/.ros/fastdds_whitelist.xml}"

exec python3 "$SCRIPT_DIR/bridge_smoke.py" "$@"
