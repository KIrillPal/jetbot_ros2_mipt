#!/usr/bin/env bash
set -euo pipefail
LAB3_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
case "$LAB3_DIR/" in /home/jetbot/IntelligentRobotics/*) echo 'Use a directory outside IntelligentRobotics.' >&2; exit 2;; esac
LAB3_NAME=jetbot_lab3_fusion
LAB3_ACTION=${1:-start}
if docker inspect "$LAB3_NAME" >/dev/null 2>&1; then
  [[ $(docker inspect -f '{{index .Config.Labels "org.mipt.lab3"}}' "$LAB3_NAME") == odometry-fusion ]] || exit 2
  case "$LAB3_ACTION" in
    start) docker start "$LAB3_NAME";;
    stop) docker stop "$LAB3_NAME";;
    restart) docker restart "$LAB3_NAME";;
    status) docker inspect -f '{{.State.Status}}' "$LAB3_NAME";;
    *) echo 'Usage: run_on_robot.sh start|stop|restart|status' >&2; exit 2;;
  esac
  exit
fi
[[ $LAB3_ACTION == start ]] || exit 2
test -x "$LAB3_DIR/ws/install/robot_localization/lib/robot_localization/ekf_node"
docker run -d --name "$LAB3_NAME" --label org.mipt.lab3=odometry-fusion \
  --network host --read-only --cap-drop ALL --security-opt no-new-privileges:true \
  --user 1000:1000 --cpus=1.0 --memory=512m --memory-swap=768m \
  --tmpfs /tmp:rw,nosuid,nodev,size=64m,mode=1777 \
  --mount "type=bind,source=$LAB3_DIR,target=/lab3,readonly" \
  -e ROS_DOMAIN_ID=2 -e ROS_LOCALHOST_ONLY=0 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOG_DIR=/tmp/lab3-ros-log \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/lab3/fastdds_udp.xml \
  -e FASTDDS_DEFAULT_PROFILES_FILE=/lab3/fastdds_udp.xml \
  jetbot_lab3_fusion:20261102 bash -c \
  'set -e; source /opt/ros/humble/install/setup.bash; source /lab3/ws/install/setup.bash; exec python3 -B /lab3/run_stack.py'
