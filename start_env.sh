#!/usr/bin/env bash
# Start the whole UR test environment in one tmux session.
# Usage: ./start_env.sh [mock|ursim|real] [--build]
#   mock  : ur_robot_driver mock hardware
#   ursim : URSim in docker (ur_client_library), driver connects to it
#   real  : physical UR5e, set ROBOT_IP=<ip>
# Env overrides: ROBOT_IP, PORT (Arduino serial port), WS (workspace)
# Stop everything: tmux kill-session -t ur
set -euo pipefail

MODE="${1:-mock}"
BUILD="${2:-}"
WS="${WS:-$HOME/robotics_project}"
PORT="${PORT:-/dev/ttyACM0}"
SESSION=ur

case "$MODE" in
  mock)  ROBOT_IP="yyy.yyy.yyy.yyy"; DRIVER_EXTRA="use_mock_hardware:=true" ;;
  ursim) ROBOT_IP="${ROBOT_IP:-127.0.0.1}"; DRIVER_EXTRA="" ;;
  real)  : "${ROBOT_IP:?set ROBOT_IP=<robot ip> for real mode}"; DRIVER_EXTRA="" ;;
  *) echo "mode must be mock|ursim|real"; exit 1 ;;
esac

command -v tmux >/dev/null || { echo "install tmux: sudo apt install tmux"; exit 1; }

if [[ "$BUILD" == "--build" ]]; then
  (cd "$WS" && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install)
fi

# Prefix run in every pane (discovery server env is required in WSL)
SRC="source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && export ROS_DISCOVERY_SERVER=127.0.0.1:11811"

tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -f fast-discovery-server 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n discovery
tmux set-option -t "$SESSION" mouse on

# win <name> <delay_s> <command>
win() {
  tmux new-window -t "$SESSION:" -n "$1"
  tmux send-keys -t "$SESSION:$1" "$SRC; sleep $2; $3" C-m
}

# 1. Fast DDS discovery server
tmux send-keys -t "$SESSION:discovery" \
  "source /opt/ros/jazzy/setup.bash; fastdds discovery --server-id 0" C-m

# 2. URSim (only in ursim mode)
if [[ "$MODE" == "ursim" ]]; then
  win ursim 0 "docker start -ai ursim"
fi

# 3. UR driver
win driver 3 "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=$ROBOT_IP $DRIVER_EXTRA launch_rviz:=false"

# 4. MoveIt + RViz (waits for the driver)
win moveit 15 "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true"

# 5. Arduino bridge + stop_on_force (top), live sensor echo (bottom)
win bridge 3 "ros2 run ur_app_demo arduino_bridge --ros-args -p port:=$PORT"
win stop 3 "ros2 run ur_app_demo stop_on_force --ros-args -p threshold:=${THRESHOLD:-512}"
ECHO_PANE=$(tmux split-window -t "$SESSION:bridge" -v -P -F '#{pane_id}')
tmux send-keys -t "$ECHO_PANE" "$SRC; sleep 6; export ROS_SUPER_CLIENT=true; ros2 topic echo /arduino_force std_msgs/msg/Int32 --no-daemon" C-m

# 6. Free shell with the demo command pre-typed (not executed)
tmux new-window -t "$SESSION:" -n shell
tmux send-keys -t "$SESSION:shell" "$SRC; export ROS_SUPER_CLIENT=true" C-m
tmux send-keys -t "$SESSION:shell" "ros2 run ur_app_demo waypoints"

tmux select-window -t "$SESSION:shell"
tmux attach -t "$SESSION"