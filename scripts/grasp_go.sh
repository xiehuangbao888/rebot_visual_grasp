#!/usr/bin/env bash
# Start grasp_execute and trigger /grasp_execute once poses are ready.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${ROOT}/install/setup.bash"
elif [[ -f "${HOME}/rebotarm_ros2/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${HOME}/rebotarm_ros2/install/setup.bash"
fi

WAIT_POSES="${WAIT_POSES:-1}"
POSE_TIMEOUT_S="${POSE_TIMEOUT_S:-60}"
EXTRA_LAUNCH_ARGS=("$@")

echo "[grasp_go] starting grasp_execute..."
ros2 launch rebot_visual_grasp grasp_execute.launch.py "${EXTRA_LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

cleanup() {
  if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[grasp_go] stopping grasp_execute (pid=${LAUNCH_PID})"
    kill "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[grasp_go] waiting for /grasp_execute ..."
for _ in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -qx '/grasp_execute'; then
    break
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "[grasp_go] grasp_execute exited early" >&2
    exit 1
  fi
  sleep 0.5
done
if ! ros2 service list 2>/dev/null | grep -qx '/grasp_execute'; then
  echo "[grasp_go] timeout waiting for /grasp_execute" >&2
  exit 1
fi

if [[ "${WAIT_POSES}" == "1" ]]; then
  echo "[grasp_go] waiting for /grasp_pose /pregrasp_pose /place_pose (timeout ${POSE_TIMEOUT_S}s)..."
  deadline=$((SECONDS + POSE_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    ok=1
    for topic in /grasp_pose /pregrasp_pose /place_pose; do
      if ! timeout 1 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
        ok=0
        break
      fi
    done
    if (( ok == 1 )); then
      echo "[grasp_go] poses ready"
      break
    fi
    sleep 0.5
  done
  if (( SECONDS >= deadline )); then
    echo "[grasp_go] timeout waiting for poses; make sure grasp_yolo is publishing" >&2
    exit 1
  fi
fi

echo "[grasp_go] calling /grasp_execute ..."
ros2 service call /grasp_execute std_srvs/srv/Trigger {}
echo "[grasp_go] done (Ctrl+C to stop grasp_execute)"
wait "${LAUNCH_PID}"
