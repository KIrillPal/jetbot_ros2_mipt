#!/usr/bin/env bash
# Start the whole teleoperation stack, and stop every part of it on exit.
# Success lines are only "OK  <stage>". Failures print ERROR plus the reason.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export DISPLAY="${DISPLAY:-:0}"

LOG_DIR=/tmp/jetbot-teleop
mkdir -p "$LOG_DIR"
LOCK_FILE="$LOG_DIR/entrypoint.pid"

WE_STARTED_CONTAINER=0
CAMERA_PID=""
DRIVER_EXEC_PID=""
DRIVER_INNER_PID=""
KEYBOARD_EXEC_PID=""
KEYBOARD_INNER_PID=""
CLEANED=0

ok() {
  printf '\033[32mOK\033[0m  %s\n' "$1"
}

fail() {
  local title=$1
  shift || true
  printf '\033[31mERROR\033[0m  %s\n' "$title" >&2
  local line
  for line in "$@"; do
    [[ -n $line ]] || continue
    printf '       %s\n' "$line" >&2
  done
  exit 1
}

log_lines() {
  local file=$1
  [[ -f $file ]] || return 0
  tail -n 30 "$file"
}

compose() {
  docker compose "$@"
}

container_running() {
  compose ps --status running --format '{{.Service}}' 2>/dev/null | grep -qx 'jetbot_educational'
}

exec_in() {
  compose exec --privileged -T jetbot_educational bash -lc "$1"
}

# Bracket trick so pgrep does not match the shell that invoked it.
proc_in_container() {
  local pattern=$1
  exec_in "pgrep -f '$pattern'" >/dev/null 2>&1
}

stop_inner() {
  local pid=$1
  [[ -n $pid ]] || return 0
  exec_in "kill -INT $pid" >/dev/null 2>&1 || true
  local i
  for i in 1 2 3 4 5 6 7 8; do
    exec_in "kill -0 $pid" >/dev/null 2>&1 || return 0
    sleep 0.25
  done
  exec_in "kill -TERM $pid" >/dev/null 2>&1 || true
  sleep 0.5
  exec_in "kill -KILL $pid" >/dev/null 2>&1 || true
}

stop_host() {
  local pid=$1
  [[ -n $pid ]] || return 0
  kill -INT "$pid" >/dev/null 2>&1 || true
  local i
  for i in 1 2 3 4 5 6 7 8; do
    kill -0 "$pid" >/dev/null 2>&1 || return 0
    sleep 0.25
  done
  kill -KILL "$pid" >/dev/null 2>&1 || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ $CLEANED -eq 1 ]]; then
    exit "$status"
  fi
  CLEANED=1
  set +e
  stop_host "$CAMERA_PID"
  stop_inner "$KEYBOARD_INNER_PID"
  stop_host "$KEYBOARD_EXEC_PID"
  stop_inner "$DRIVER_INNER_PID"
  stop_host "$DRIVER_EXEC_PID"
  if [[ $WE_STARTED_CONTAINER -eq 1 ]]; then
    compose stop jetbot_educational >/dev/null 2>&1
  fi
  if [[ -f $LOCK_FILE ]] && [[ "$(cat "$LOCK_FILE" 2>/dev/null)" == "$$" ]]; then
    rm -f "$LOCK_FILE"
  fi
  exit "$status"
}

device_holders() {
  local dev=$1
  if command -v fuser >/dev/null 2>&1; then
    fuser -v "$dev" 2>&1 || true
  elif command -v lsof >/dev/null 2>&1; then
    lsof "$dev" 2>&1 || true
  fi
}

check_device_free() {
  local dev=$1
  local name=$2
  local err
  if [[ ! -e $dev ]]; then
    fail "sensor ${name} is missing" \
      "${dev} does not exist." \
      "On the host, run: sudo ./general_scripts/create_udev_rules.sh"
  fi
  if ! err="$(python3 -c "f=open('${dev}','rb+',buffering=0); f.close()" 2>&1)"; then
    local holders
    holders="$(device_holders "$dev")"
    fail "sensor ${name} is busy" \
      "${dev} cannot be opened: ${err}" \
      "${holders:-no holder process could be identified (try: sudo fuser -v ${dev})}" \
      "Stop whatever holds it, including a second teleop stack, pid_tune, or the old jetbot container."
  fi
}

check_not_already_up() {
  local -a reasons=()
  if [[ -f $LOCK_FILE ]]; then
    local old
    old="$(cat "$LOCK_FILE" 2>/dev/null || true)"
    if [[ -n $old ]] && kill -0 "$old" 2>/dev/null; then
      reasons+=("entrypoint is already running as pid ${old}")
    else
      rm -f "$LOCK_FILE"
    fi
  fi
  if pgrep -f 'general_scripts/camera_serve[r].py' >/dev/null 2>&1; then
    reasons+=("camera server is already running on the host (port 8080)")
  fi
  if docker ps --format '{{.Names}}' 2>/dev/null | grep -qx 'jetbot_ros2_mipt-jetbot-1'; then
    reasons+=("old container jetbot_ros2_mipt-jetbot-1 is running and can hold /dev/ttyMOTOR")
  fi
  if container_running; then
    if proc_in_container 'activate_all_driver[s].launch.py'; then
      reasons+=("activate_all_drivers is already running in the container")
    fi
    if proc_in_container 'lib/jetbot_bringup/keyboar[d]_teleop'; then
      reasons+=("keyboard_teleop is already running in the container (port 8081)")
    fi
  fi
  if [[ ${#reasons[@]} -gt 0 ]]; then
    fail "teleoperation pipeline is already up" \
      "${reasons[@]}" \
      "Stop that entrypoint with Ctrl+C. If it was started by hand, stop camera_server.py, keyboard_teleop, and activate_all_drivers."
  fi
}

check_ports_free() {
  local port=$1
  local what=$2
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${port}\$"; then
    local who
    who="$(ss -ltnp 2>/dev/null | grep -E ":${port}\\b" || true)"
    fail "${what} port ${port} is already in use" \
      "${who:-a process is listening on ${port}}" \
      "Free the port before starting teleoperation."
  fi
}

wait_for_pidfile() {
  local name=$1
  local i pid
  for i in $(seq 1 40); do
    pid="$(exec_in "cat /tmp/jetbot-teleop-${name}.pid" 2>/dev/null || true)"
    pid="${pid//$'\r'/}"
    if [[ $pid =~ ^[0-9]+$ ]]; then
      printf '%s' "$pid"
      return 0
    fi
    sleep 0.25
  done
  return 1
}

start_docker() {
  if ! docker info >/dev/null 2>"$LOG_DIR/docker.err"; then
    mapfile -t lines < <(log_lines "$LOG_DIR/docker.err")
    fail "docker is not available" \
      "The docker daemon did not answer." \
      "${lines[@]}"
  fi
  if container_running; then
    ok "docker container"
    return
  fi
  if ! compose up -d jetbot_educational >"$LOG_DIR/compose-up.log" 2>&1; then
    mapfile -t lines < <(log_lines "$LOG_DIR/compose-up.log")
    mapfile -t more < <(compose logs --tail 30 jetbot_educational 2>&1 || true)
    fail "docker container did not start" "${lines[@]}" "${more[@]}"
  fi
  WE_STARTED_CONTAINER=1
  local i
  for i in $(seq 1 30); do
    if container_running; then
      ok "docker container"
      return
    fi
    sleep 0.5
  done
  mapfile -t lines < <(compose ps -a 2>&1 || true)
  mapfile -t more < <(compose logs --tail 40 jetbot_educational 2>&1 || true)
  fail "docker container did not stay up" \
    "jetbot_educational exited or never became running." \
    "${lines[@]}" \
    "${more[@]}"
}

start_drivers() {
  : >"$LOG_DIR/drivers.log"
  compose exec --privileged -T jetbot_educational bash -lc \
    'source /opt/ros/humble/install/setup.bash
     source /home/app/ros2_ws/install/setup.bash
     echo $$ >/tmp/jetbot-teleop-drivers.pid
     exec ros2 launch jetbot_bringup activate_all_drivers.launch.py' \
    >"$LOG_DIR/drivers.log" 2>&1 &
  DRIVER_EXEC_PID=$!
  DRIVER_INNER_PID="$(wait_for_pidfile drivers || true)"
  if [[ -z $DRIVER_INNER_PID ]]; then
    mapfile -t lines < <(log_lines "$LOG_DIR/drivers.log")
    fail "motor and lidar drivers failed to start" "${lines[@]}"
  fi
}

# Spawners exit on purpose once the controller is active. The launch log says
# "process has finished cleanly", but diffbot_base_controller keeps running
# inside ros2_control_node. That line is success, not shutdown.
# ros2 launch colors the controller name, so the bytes are
# "activated <esc>[1mdiffbot_base_controller<esc>[0m", not plain text.
plain_driver_log() {
  sed -E 's/\x1B\[[0-9;]*[A-Za-z]//g' "$LOG_DIR/drivers.log"
}

controllers_are_active() {
  local text
  text="$(plain_driver_log)"
  grep -q 'Configured and activated joint_state_broadcaster' <<<"$text" \
    && grep -q 'Configured and activated diffbot_base_controller' <<<"$text" \
    && grep -q 'Successfully activated!' <<<"$text"
}

wait_for_controllers() {
  local deadline=$((SECONDS + 50))
  while (( SECONDS < deadline )); do
    if ! kill -0 "$DRIVER_EXEC_PID" 2>/dev/null; then
      mapfile -t lines < <(log_lines "$LOG_DIR/drivers.log")
      fail "motor and lidar drivers exited" "${lines[@]}"
    fi
    if grep -Eq 'OpenFailed|Device or resource busy|LibSerial::' "$LOG_DIR/drivers.log"; then
      mapfile -t lines < <(grep -E 'OpenFailed|Device or resource busy|LibSerial::|ttyMOTOR|ttyLIDAR' "$LOG_DIR/drivers.log" | tail -n 20)
      fail "a driver could not open its sensor" \
        "ros2_control failed while opening the motor serial port." \
        "${lines[@]}"
    fi
    if controllers_are_active; then
      ok "motor drivers"
      return
    fi
    sleep 1
  done
  mapfile -t lines < <(log_lines "$LOG_DIR/drivers.log")
  fail "motor drivers did not become active" \
    "The launch log never reported that diffbot_base_controller was activated." \
    "A spawner line 'process has finished cleanly' is normal and is not a failure." \
    "${lines[@]}"
}

wait_for_lidar() {
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if proc_in_container 'rplida[r]_node'; then
      if grep -Eq 'cannot retrieve RPLidar health|process has died' "$LOG_DIR/drivers.log"; then
        break
      fi
      ok "lidar"
      return
    fi
    if grep -Eq 'cannot retrieve RPLidar health|ttyLIDAR|process has died' "$LOG_DIR/drivers.log"; then
      break
    fi
    sleep 1
  done
  mapfile -t lines < <(grep -E 'rplidar|RPLidar|ttyLIDAR|died|Error' "$LOG_DIR/drivers.log" | tail -n 20)
  fail "lidar sensor failed" \
    "rplidar_node is not running. /dev/ttyLIDAR may be busy, unplugged, or the lidar did not answer." \
    "${lines[@]}"
}

start_keyboard() {
  : >"$LOG_DIR/keyboard.log"
  compose exec --privileged -T jetbot_educational bash -lc \
    'source /opt/ros/humble/install/setup.bash
     source /home/app/ros2_ws/install/setup.bash
     echo $$ >/tmp/jetbot-teleop-keyboard.pid
     exec ros2 run jetbot_bringup keyboard_teleop --ros-args -p linear:=0.12 -p angular:=1.0' \
    >"$LOG_DIR/keyboard.log" 2>&1 &
  KEYBOARD_EXEC_PID=$!
  KEYBOARD_INNER_PID="$(wait_for_pidfile keyboard || true)"
  if [[ -z $KEYBOARD_INNER_PID ]]; then
    mapfile -t lines < <(log_lines "$LOG_DIR/keyboard.log")
    fail "keyboard teleop failed to start" "${lines[@]}"
  fi
  local i
  for i in $(seq 1 40); do
    if ! kill -0 "$KEYBOARD_EXEC_PID" 2>/dev/null; then
      mapfile -t lines < <(log_lines "$LOG_DIR/keyboard.log")
      fail "keyboard teleop exited" "${lines[@]}"
    fi
    if grep -q 'Address already in use' "$LOG_DIR/keyboard.log"; then
      fail "keyboard teleop port 8081 is already in use" \
        "Another keyboard_teleop is listening." \
        "$(log_lines "$LOG_DIR/keyboard.log")"
    fi
    if ss -ltn | awk '{print $4}' | grep -Eq '(:8081)$'; then
      ok "keyboard teleop"
      return
    fi
    sleep 0.25
  done
  mapfile -t lines < <(log_lines "$LOG_DIR/keyboard.log")
  fail "keyboard teleop did not open port 8081" "${lines[@]}"
}

start_camera() {
  : >"$LOG_DIR/camera.log"
  setsid python3 "$ROOT/general_scripts/camera_server.py" >"$LOG_DIR/camera.log" 2>&1 </dev/null &
  CAMERA_PID=$!
  local i
  for i in $(seq 1 40); do
    if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
      mapfile -t lines < <(log_lines "$LOG_DIR/camera.log")
      fail "camera server exited" \
        "The CSI camera may be busy (another nvargus or camera_server process) or the pipeline failed." \
        "${lines[@]}"
    fi
    if grep -Eq 'Address already in use|nvargus|Failed to|Error' "$LOG_DIR/camera.log"; then
      if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
        mapfile -t lines < <(log_lines "$LOG_DIR/camera.log")
        fail "camera server failed" "${lines[@]}"
      fi
    fi
    if curl -sf -o /dev/null --max-time 1 "http://127.0.0.1:8080/teleop"; then
      ok "camera and web page"
      return
    fi
    sleep 0.5
  done
  mapfile -t lines < <(log_lines "$LOG_DIR/camera.log")
  fail "camera server did not serve the teleop page" "${lines[@]}"
}

supervise() {
  while true; do
    if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
      mapfile -t lines < <(log_lines "$LOG_DIR/camera.log")
      fail "camera server stopped" "${lines[@]}"
    fi
    if ! kill -0 "$DRIVER_EXEC_PID" 2>/dev/null; then
      mapfile -t lines < <(log_lines "$LOG_DIR/drivers.log")
      fail "motor and lidar drivers stopped" "${lines[@]}"
    fi
    if ! kill -0 "$KEYBOARD_EXEC_PID" 2>/dev/null; then
      mapfile -t lines < <(log_lines "$LOG_DIR/keyboard.log")
      fail "keyboard teleop stopped" "${lines[@]}"
    fi
    sleep 2
  done
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

check_not_already_up
echo "$$" >"$LOCK_FILE"
check_device_free /dev/ttyMOTOR motor
ok "motor port"
check_device_free /dev/ttyLIDAR lidar
ok "lidar port"
if pgrep -f 'nvarguscamera[s]rc|camera_x11_vie[w].sh|start_pi_camera_strea[m].sh' >/dev/null 2>&1; then
  mapfile -t holders < <(pgrep -af 'nvarguscamerasrc|camera_x11_view.sh|start_pi_camera_stream.sh' || true)
  fail "camera is busy" \
    "Another process already has the CSI camera." \
    "${holders[@]}"
fi
ok "camera is free"
check_ports_free 8080 "camera"
check_ports_free 8081 "keyboard teleop"
start_docker
start_drivers
wait_for_controllers
wait_for_lidar
start_keyboard
start_camera

ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
ip="${ip:-127.0.0.1}"
ok "teleoperation  http://${ip}:8080/teleop"
supervise
