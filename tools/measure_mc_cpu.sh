#!/bin/bash
# Measure manual_composition CPU% over a short run (grip hw params, no motion).
# Runs the binary via the passwordless sudoers entry (SETENV NOPASSWD).
# Usage: bash tools/measure_mc_cpu.sh <label> [run_seconds]
LABEL="${1:-run}"
RUN_S="${2:-30}"
WS=/home/vmx/skill23_ws
BIN=$WS/install/studica_control/lib/studica_control/manual_composition
PARAMS="${PARAMS_OVERRIDE:-$WS/install/skill23_studio/share/skill23_studio/config/hw_params/hw_params_grip.yaml}"

source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export ROS_LOG_DIR=/tmp/roslog_mc_test
mkdir -p "$ROS_LOG_DIR"
chmod 777 "$ROS_LOG_DIR" 2>/dev/null

if pgrep -f 'manual_composition --ros-args' >/dev/null; then
  echo "ERROR: manual_composition already running — stop the stack first"
  exit 1
fi

sudo -E -n "$BIN" --ros-args -r __node:=control_server --params-file "$PARAMS" \
  > /tmp/mc_${LABEL}.log 2>&1 &
SUDO_PID=$!
echo "sudo pid=$SUDO_PID label=$LABEL"
sleep 10   # warmup: HAL init, AHRS calibration chatter

PID=""
for p in $(pgrep -f 'manual_composition --ros-args'); do
  [ "$(cat /proc/$p/comm 2>/dev/null)" = "manual_composit" ] && PID=$p
done
if [ -z "$PID" ]; then
  echo "ERROR: process not found after warmup"; tail -20 /tmp/mc_${LABEL}.log; exit 1
fi
echo "measuring pid=$PID"

read_cpu() { awk '{print $14+$15}' /proc/$1/stat 2>/dev/null; }
HZ=$(getconf CLK_TCK)
T0=$(read_cpu $PID)
sleep "$RUN_S"
T1=$(read_cpu $PID)
if [ -z "${T1:-}" ]; then
  echo "ERROR: process died during measurement"; tail -20 /tmp/mc_${LABEL}.log; exit 1
fi

CPU=$(echo "scale=1; ($T1 - $T0) * 100 / $HZ / $RUN_S" | bc)
echo "RESULT label=$LABEL cpu=${CPU}% over ${RUN_S}s (ticks $T0 -> $T1)"

sudo -n pkill -INT -f 'manual_composition --ros-args'
sleep 3
sudo -n pkill -9 -f 'manual_composition --ros-args' 2>/dev/null
echo "--- last log lines ---"
tail -8 /tmp/mc_${LABEL}.log
