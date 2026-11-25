#!/usr/bin/env bash
# Run from any directory after sourcing ROS 2 Humble and the simulator workspace.
set -eo pipefail
DEMO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$DEMO_DIR/../.." && pwd)"
HEURISTIC="${1:-euclidean}"
MODE="${2:-demo}"
if [[ "$HEURISTIC" != euclidean && "$HEURISTIC" != manhattan ]] || [[ "$MODE" != demo && "$MODE" != --test ]]; then
  echo 'Usage: launch_demo.sh [euclidean|manhattan] [--test]' >&2; exit 1
fi
PYTHON="${MTE544_PYTHON:-python3}"
for command in ros2 gzserver setsid "$PYTHON"; do
  command -v "$command" >/dev/null || { echo "Missing $command. Source your ROS/simulator environment first." >&2; exit 1; }
done
"$PYTHON" -c 'import rclpy, numpy, matplotlib, sklearn, yaml'
if [[ "$MODE" == demo ]]; then
  command -v rviz2 >/dev/null
  [[ -n "${DISPLAY:-}" ]] || { echo 'Run in a graphical desktop terminal, or use --test.' >&2; exit 1; }
fi
TB3_SHARE="$(ros2 pkg prefix --share turtlebot3_gazebo)"
export TURTLEBOT3_MODEL=burger ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${MTE544_SIM_DOMAIN:-155}"
export MTE544_GAZEBO_PORT="${MTE544_GAZEBO_PORT:-11355}"
export GAZEBO_MASTER_URI="http://127.0.0.1:$MTE544_GAZEBO_PORT" GAZEBO_IP=127.0.0.1 GAZEBO_MODEL_DATABASE_URI=''
export GAZEBO_MODEL_PATH="$TB3_SHARE/models:${GAZEBO_MODEL_PATH:-}"
"$PYTHON" - <<'PY'
import socket, os
port = int(os.environ["MTE544_GAZEBO_PORT"])
with socket.socket() as s:
    if s.connect_ex(('127.0.0.1',port)) == 0:
        raise SystemExit(f'Demo Gazebo port {port} is in use. Close the earlier demo first.')
PY
mkdir -p "$DEMO_DIR/runs"
RUN_DIR="$(mktemp -d "$DEMO_DIR/runs/${HEURISTIC}-XXXXXX")"
cp "$REPO_DIR/room.pgm" "$REPO_DIR/room.yaml" "$RUN_DIR/"
cd "$RUN_DIR"
PIDS=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  for attempt in {1..30}; do
    alive=0
    for pid in "${PIDS[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=1; done
    [[ "$alive" == 0 ]] && break
    sleep 0.1
  done
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  wait || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() { local log="$1"; shift; setsid "$@" > "$log" 2>&1 & PIDS+=("$!"); }
start gazebo.log gzserver "$DEMO_DIR/arena.world" -slibgazebo_ros_init.so -slibgazebo_ros_factory.so
ros2 run gazebo_ros spawn_entity.py -entity lab4_burger -file "$TB3_SHARE/models/turtlebot3_burger/model.sdf" -x 0 -y 0 -z 0.01 > spawn.log 2>&1
start robot-state.log ros2 launch turtlebot3_gazebo robot_state_publisher.launch.py use_sim_time:=true
start map.log "$PYTHON" -u "$REPO_DIR/mapPublisher.py"
echo "Heuristic: $HEURISTIC. Logs/results: $RUN_DIR"
if [[ "$MODE" == --test ]]; then
  "$PYTHON" -u "$DEMO_DIR/test_navigation.py" --heuristic "$HEURISTIC" 2>&1 | tee navigation.log
else
  start decisions.log "$PYTHON" -u "$DEMO_DIR/run_decisions.py" --heuristic "$HEURISTIC"
  start rviz.log rviz2 -d "$DEMO_DIR/demo.rviz" --ros-args -p use_sim_time:=true
  echo 'In RViz use 2D Goal Pose: (2, 0), then (-1, 0). Ctrl+C stops the demo.'
  wait -n "${PIDS[@]}"
fi
