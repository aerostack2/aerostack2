#!/bin/bash
# =============================================================================
# drone_bringup.sh
# 드론 부팅 시 as2 필수 노드를 자동 실행하는 셸 스크립트.
#
# 사용법:
#   ./drone_bringup.sh [옵션]
#
# 옵션:
#   -n, --namespace   <name>   드론 네임스페이스 (기본: config 파일 값)
#   -c, --config      <path>   drone_bringup.yaml 경로 (기본: 패키지 기본값)
#   -s, --sim                  시뮬레이션 시간 사용 (use_sim_time:=true)
#   -l, --log-level   <level>  로그 레벨: debug|info|warn|error (기본: info)
#   -h, --help                 이 도움말 출력
#
# 예시:
#   # 기본 실행 (drone_bringup.yaml 기본값 사용)
#   ./drone_bringup.sh
#
#   # 드론 네임스페이스 지정
#   ./drone_bringup.sh -n drone1
#
#   # 시뮬레이션 모드
#   ./drone_bringup.sh -n drone0 --sim
#
#   # 커스텀 설정 파일
#   ./drone_bringup.sh -c /path/to/my_config.yaml
# =============================================================================

set -e

# ---------------------------------------------------------------------------
# 기본값
# ---------------------------------------------------------------------------
NAMESPACE=""
CONFIG=""
USE_SIM_TIME="false"
LOG_LEVEL="info"
ROS_DISTRO="${ROS_DISTRO:-humble}"
WS_DIR="${AS2_WS:-$HOME/ros2_ws}"

# ---------------------------------------------------------------------------
# 인자 파싱
# ---------------------------------------------------------------------------
while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--namespace)
            NAMESPACE="$2"; shift 2 ;;
        -c|--config)
            CONFIG="$2"; shift 2 ;;
        -s|--sim)
            USE_SIM_TIME="true"; shift ;;
        -l|--log-level)
            LOG_LEVEL="$2"; shift 2 ;;
        -h|--help)
            head -n 30 "$0" | grep '^#' | sed 's/^# \?//'
            exit 0 ;;
        *)
            echo "[ERROR] 알 수 없는 옵션: $1" >&2
            exit 1 ;;
    esac
done

# ---------------------------------------------------------------------------
# 환경 소스
# ---------------------------------------------------------------------------
echo "[Bringup] ROS2 환경 소스: /opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${WS_DIR}/install/setup.bash" ]]; then
    echo "[Bringup] 워크스페이스 소스: ${WS_DIR}/install/setup.bash"
    # shellcheck source=/dev/null
    source "${WS_DIR}/install/setup.bash"
else
    echo "[WARN] 워크스페이스 setup.bash 없음: ${WS_DIR}/install/setup.bash"
    echo "[WARN] AS2_WS 환경변수로 워크스페이스 경로를 지정하세요."
fi

# ---------------------------------------------------------------------------
# 런치 인자 구성
# ---------------------------------------------------------------------------
LAUNCH_ARGS=()

if [[ -n "$NAMESPACE" ]]; then
    LAUNCH_ARGS+=("namespace:=${NAMESPACE}")
fi

if [[ -n "$CONFIG" ]]; then
    if [[ ! -f "$CONFIG" ]]; then
        echo "[ERROR] 설정 파일을 찾을 수 없음: $CONFIG" >&2
        exit 1
    fi
    LAUNCH_ARGS+=("config:=${CONFIG}")
fi

LAUNCH_ARGS+=("use_sim_time:=${USE_SIM_TIME}")
LAUNCH_ARGS+=("log_level:=${LOG_LEVEL}")

# ---------------------------------------------------------------------------
# 실행
# ---------------------------------------------------------------------------
echo ""
echo "=================================================="
echo " AeroStack2 Drone Bringup"
echo " namespace  : ${NAMESPACE:-'(config 파일 값 사용)'}"
echo " config     : ${CONFIG:-'(패키지 기본값 사용)'}"
echo " sim time   : ${USE_SIM_TIME}"
echo " log level  : ${LOG_LEVEL}"
echo "=================================================="
echo ""

exec ros2 launch as2_drone_bringup drone_bringup.launch.py "${LAUNCH_ARGS[@]}"
