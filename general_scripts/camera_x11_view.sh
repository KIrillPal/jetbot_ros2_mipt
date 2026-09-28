#!/usr/bin/env bash

set -euo pipefail

width=640
height=360
fps=5
sensor_id=0

usage() {
  cat <<'EOF'
Usage: camera_x11_view.sh [OPTIONS]

Show the Jetson CSI camera in an X11 window forwarded over SSH.

Options:
  --width PIXELS    Preview width (default: 640)
  --height PIXELS   Preview height (default: 360)
  --fps RATE        Forwarded preview frame rate (default: 5)
  --sensor-id ID    nvarguscamerasrc sensor ID (default: 0)
  -h, --help        Show this help

Connect from the client with trusted X11 forwarding:
  ssh -Y jetbot@JETSON_IP

Then run this script as the regular user, not with sudo.
EOF
}

require_positive_integer() {
  local name=$1
  local value=$2

  if [[ ! $value =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: ${name} must be a positive integer; got '${value}'." >&2
    exit 2
  fi
}

while (($#)); do
  case "$1" in
    --width)
      [[ $# -ge 2 ]] || { echo "Error: --width needs a value." >&2; exit 2; }
      width=$2
      shift 2
      ;;
    --height)
      [[ $# -ge 2 ]] || { echo "Error: --height needs a value." >&2; exit 2; }
      height=$2
      shift 2
      ;;
    --fps)
      [[ $# -ge 2 ]] || { echo "Error: --fps needs a value." >&2; exit 2; }
      fps=$2
      shift 2
      ;;
    --sensor-id)
      [[ $# -ge 2 ]] || { echo "Error: --sensor-id needs a value." >&2; exit 2; }
      sensor_id=$2
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Error: unknown option '$1'." >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_positive_integer width "$width"
require_positive_integer height "$height"
require_positive_integer fps "$fps"
if [[ ! $sensor_id =~ ^[0-9]+$ ]]; then
  echo "Error: sensor-id must be a non-negative integer; got '${sensor_id}'." >&2
  exit 2
fi

if [[ -z ${DISPLAY:-} ]]; then
  cat >&2 <<'EOF'
Error: DISPLAY is empty, so this is not an X11-forwarded SSH session.

Reconnect from the client with:
  ssh -Y jetbot@JETSON_IP

On Windows, first start an X server such as VcXsrv or Xming.
EOF
  exit 1
fi

for command in gst-launch-1.0 xauth; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "Error: required command '$command' is not installed." >&2
    exit 1
  fi
done

for plugin in nvarguscamerasrc nvvidconv videorate videoconvert queue ximagesink; do
  if ! gst-inspect-1.0 "$plugin" >/dev/null 2>&1; then
    echo "Error: required GStreamer plugin '$plugin' is unavailable." >&2
    exit 1
  fi
done

if ! xauth list "$DISPLAY" 2>/dev/null | grep -q .; then
  echo "Warning: no Xauthority cookie was found for DISPLAY=${DISPLAY}." >&2
  echo "If the window fails to open, reconnect using 'ssh -Y'." >&2
fi

echo "Opening camera ${sensor_id} at ${width}x${height}, ${fps} FPS."
echo "Forwarding window to DISPLAY=${DISPLAY}; press Ctrl+C to stop."

exec gst-launch-1.0 -e \
  nvarguscamerasrc sensor-id="$sensor_id" \
  '!' 'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1' \
  '!' nvvidconv \
  '!' "video/x-raw,width=${width},height=${height},format=BGRx" \
  '!' videorate drop-only=true \
  '!' "video/x-raw,framerate=${fps}/1" \
  '!' queue max-size-buffers=1 leaky=downstream \
  '!' videoconvert \
  '!' ximagesink sync=false
