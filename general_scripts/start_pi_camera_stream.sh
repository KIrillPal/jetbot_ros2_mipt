#!/usr/bin/env bash

set -euo pipefail

width=${JETBOT_CAMERA_WIDTH:-1280}
height=${JETBOT_CAMERA_HEIGHT:-720}
framerate=${JETBOT_CAMERA_FRAMERATE:-30}
bitrate=${JETBOT_CAMERA_BITRATE:-3000000}
port=${JETBOT_CAMERA_PORT:-5004}

echo "Streaming IMX219 camera ${width}x${height}@${framerate} to udp://127.0.0.1:${port}"

exec gst-launch-1.0 -e \
  nvarguscamerasrc sensor-id=0 \
  '!' "video/x-raw(memory:NVMM),width=${width},height=${height},format=NV12,framerate=${framerate}/1" \
  '!' nvv4l2h264enc \
      insert-sps-pps=true \
      iframeinterval=15 \
      idrinterval=15 \
      bitrate="${bitrate}" \
  '!' h264parse config-interval=-1 \
  '!' mpegtsmux alignment=7 \
  '!' udpsink host=127.0.0.1 port="${port}" sync=false async=false
