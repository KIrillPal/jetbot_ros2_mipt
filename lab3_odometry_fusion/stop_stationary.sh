#!/usr/bin/env bash
# Stop only the temporary commissioning processes recorded by this session.
set -euo pipefail
LAB3_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
LAB3_CONTAINER=${LAB3_DRIVER_CONTAINER:-jetbot_ros2_mipt_rviz-jetbot_educational-1}
bash "$LAB3_DIR/run_on_robot.sh" stop
docker exec -i "$LAB3_CONTAINER" python3 - <<'PY'
import json,os,signal,time
path='/tmp/lab3-stationary-owned.json'
if not os.path.exists(path):
    raise SystemExit('No ownership record; refusing to stop any hardware process')
with open(path) as stream: records=json.load(stream)
for record in records:
    pid=record['pid']
    try:
        with open('/proc/%d/stat'%pid) as stream: start=stream.read().rsplit(')',1)[1].split()[19]
        with open('/proc/%d/cmdline'%pid,'rb') as stream: command=stream.read().replace(b'\0',b' ').decode()
    except FileNotFoundError:
        continue
    if start!=record['start'] or record['marker'] not in command:
        raise SystemExit('Process identity changed; refusing to signal pid %d'%pid)
    os.kill(pid,signal.SIGINT)
    print('Sent SIGINT to owned process',pid)
    end=time.monotonic()+15
    while time.monotonic()<end and os.path.exists('/proc/%d'%pid): time.sleep(.2)
    if os.path.exists('/proc/%d'%pid):
        raise SystemExit('Process still exists; inspect its log before starting another driver stack')
print('Temporary sensors/RF2O stopped. Start your usual teleop stack, then RF2O, then fusion.')
PY
