# JetBot ROS 2 (MIPT educational)

ROS 2 Humble stack for the MIPT JetBot: a differential-drive robot on a Jetson Nano with a serial motor board, an RPLidar, and an IMX219 CSI camera. You drive it from a browser page that shows the live camera and takes W/A/S/D from your keyboard. The same stack also supports a gamepad, SLAM mapping with `slam_toolbox`, and Nav2 configs.

- [Architecture](#architecture)
- [Teleoperation](#teleoperation)
- [Where the code lives](#where-the-code-lives)
- [Topics, frames, and ports](#topics-frames-and-ports)
- [Other workflows](#other-workflows)
- [Setup: devices, Docker, build](#setup-devices-docker-build)
- [PID tuning](#pid-tuning)
- [Troubleshooting](#troubleshooting)
- [Known gaps](#known-gaps)

---

## Architecture

The system runs in two places on the Jetson:

- **Host.** The camera server (`general_scripts/camera_server.py`). It must run on the host because the CSI camera goes through NVIDIA's Argus service, which lives outside Docker.
- **Container** `jetbot_ros2_mipt-jetbot_educational-1`. All ROS 2 nodes. It uses host networking, so the host and container talk over `127.0.0.1`, and `ROS_DOMAIN_ID=2`.

### Data flow

```text
                        Browser  (http://<jetson-ip>:8080/teleop)
                          │   ▲
     W/A/S/D, Shift, Esc  │   │  MJPEG /video_feed (~20 fps)
                          ▼   │
  ┌─────────── HOST ─────────────────────────────────────────────────────────┐
  │  camera_server.py  :8080                                                 │
  │    ├─ renders web/templates/teleop.html                                  │
  │    ├─ IMX219 ─► nvarguscamerasrc ─► nvvidconv ─► nvjpegenc ─► latest JPEG │
  │    └─ proxies /api/keyboard, /api/car/*, /api/tracking, /api/status      │
  └──────────────────────────────┬───────────────────────────────────────────┘
                                 │ HTTP 127.0.0.1:8081  (X-Teleop-Client: <cookie id>)
  ┌─────────── CONTAINER ────────▼───────────────────────────────────────────┐
  │  keyboard_teleop  (ROS node, single-client lock)                         │
  │        │ /cmd_vel_joy   geometry_msgs/Twist @ 10 Hz                      │
  │        ▼                                                                 │
  │  twist_mux   navigation=/cmd_vel (50) · joystick=/cmd_vel_joy (90)       │
  │              robot_steering=/cmd_vel_robot_steering (100)                │
  │        │ /diffbot_base_controller/cmd_vel_unstamped                      │
  │        ▼                                                                 │
  │  controller_manager (ros2_control_node, 60 Hz)                           │
  │    ├─ diffbot_base_controller ─► /diffbot_base_controller/odom           │
  │    │                             TF odom → base_footprint                │
  │    ├─ joint_state_broadcaster ─► /joint_states ─► robot_state_publisher  │
  │    └─ DiffDriveJetbotHardware plugin ─► /dev/ttyMOTOR @115200 ─► motors  │
  │                                                                          │
  │  rplidar_node  /dev/ttyLIDAR ─► /scan (frame lidar_link)                 │
  │        └─► slam_toolbox (optional) ─► /map, TF map → odom                │
  └──────────────────────────────────────────────────────────────────────────┘
```

### Design choices

- **Camera latency.** Capture, scaling, and JPEG encoding all stay in GPU memory; Python only forwards finished JPEGs. The `appsink` holds one buffer and drops old frames, and each browser gets the newest frame. A slow viewer skips frames; it never slows the camera. Measured: sensor at 60 fps, page at about 20 fps, about 20 KB per frame.
- **Single driver.** `keyboard_teleop` gives the robot to the first browser that sends a key. Other browsers get HTTP `409 busy` until the owner clicks **Disable Control** or presses Esc, or until the owner's page stops polling `/api/status` for more than 1 s (for example, the tab is closed).
- **Dead-man behavior.** `keyboard_teleop` keeps publishing at 10 Hz. It sends zero velocity when no key is held or when the owner goes stale. `twist_mux` drops a source after 0.5 s of silence, and `diffbot_base_controller` stops on its own after 0.5 s without commands (`cmd_vel_timeout`).
- **Priority.** Any input goes through `twist_mux`. A publisher on `/cmd_vel_robot_steering` (priority 100) overrides the keyboard and joystick (90), and those override Nav2 (50).

---

## Teleoperation

The motors move as soon as you press a key. Put the robot on the floor with space around it, or lift the wheels off the ground, and keep a way to cut power within reach.

You need four terminals: one on the host and three inside the container. Commands marked **host** run on the Jetson itself. Commands marked **container** run after `docker exec`.

### 0. Host: devices and container (once per boot)

```bash
ls -la /dev/ttyMOTOR /dev/ttyLIDAR
# expect: /dev/ttyMOTOR -> ttyTHS1, /dev/ttyLIDAR -> ttyUSB0

cd ~/IntelligentRobotics/jetbot_ros2_mipt
docker compose up -d
docker ps --filter name=jetbot_ros2_mipt-jetbot_educational-1
```

The container starts idle; nothing runs until you launch it. Do not start the old `jetbot_ros2_mipt-jetbot-1` container. It is the previous namespaced stack and will fight over the serial ports.

### 1. Terminal A (container): base drivers

```bash
docker exec -it --privileged jetbot_ros2_mipt-jetbot_educational-1 bash
source /home/app/ros2_ws/install/setup.bash
echo $ROS_DOMAIN_ID    # must print 2

# Preflight: the motor port must be free BEFORE launch
python3 -c "f=open('/dev/ttyMOTOR','rb+', buffering=0); print('open ok'); f.close()"

ros2 launch jetbot_bringup activate_all_drivers.launch.py
```

Leave it running. This starts `twist_mux`, `rplidar_node`, `ros2_control_node`, `robot_state_publisher`, the controller spawners, and a mesh HTTP server. It does **not** start a joystick or the keyboard node.

### 2. Terminal B (container): check the base is alive

```bash
docker exec -it --privileged jetbot_ros2_mipt-jetbot_educational-1 bash
source /home/app/ros2_ws/install/setup.bash
ros2 control list_controllers -c /controller_manager
```

Both `joint_state_broadcaster` and `diffbot_base_controller` must be **active**. From now on `/dev/ttyMOTOR` is held by `ros2_control_node`, so the preflight `open` test above will report "busy". That is expected at this stage.

### 3. Terminal C (container): keyboard teleop node

```bash
docker exec -it --privileged jetbot_ros2_mipt-jetbot_educational-1 bash
source /home/app/ros2_ws/install/setup.bash
ros2 run jetbot_bringup keyboard_teleop --ros-args -p linear:=0.12 -p angular:=1.0
```

The node logs `one web client for /api/keyboard on :8081` and listens for the camera server.

| Parameter | Default | Meaning |
|---|---|---|
| `linear` | `0.08` | forward/back speed, m/s |
| `angular` | `0.8` | turn rate, rad/s. The controller caps this at 1.0 |
| `turbo_scale` | `1.5` | forward speed multiplier while Shift is held |
| `listen_port` | `8081` | HTTP port the camera server proxies to |
| `client_timeout` | `1.0` | seconds without a status poll before the owner is dropped |

Do not run `teleop_twist_joy` at the same time. It also publishes on `/cmd_vel_joy` and will cancel the browser's commands.

### 4. Terminal D (host): camera and web page

```bash
cd ~/IntelligentRobotics/jetbot_ros2_mipt
python3 general_scripts/camera_server.py
# teleop page http://0.0.0.0:8080/teleop  camera 640x360 @ 20 fps
```

Optional flags: `--width`, `--height`, `--fps` (default 20), `--quality` (JPEG, default 70), `--port` (default 8080), `--teleop-port` (default 8081). Run it as your normal user, not with `sudo` and not inside Docker.

### 5. Browser: drive

1. Find the robot's IP with `hostname -I` on the Jetson, and open `http://<jetson-ip>:8080/teleop` from a desktop browser. On a narrow (mobile) screen the page is view-only.
2. Click **Enable Control**. The page captures the mouse pointer, and a red dot appears on the video.
3. Drive:

   | Key | Action |
   |---|---|
   | `W` / `S` | forward / back |
   | `A` / `D` | turn left / right |
   | `Shift` + `W` | turbo forward |
   | `Esc` | release control and stop |

4. Press Esc or click **Disable Control** when you're done, so another browser can take over.

The **Reset Head**, **Add Face**, **Capture Image** buttons and mouse look come from the shared frontend. This robot has no pan/tilt head, so they do nothing.

### Check the chain while driving

```bash
# container
ros2 topic echo /cmd_vel_joy                                   # what the page is asking for
ros2 topic echo /diffbot_base_controller/cmd_vel_unstamped     # what reaches the controller
```

```bash
# host
curl -s http://127.0.0.1:8080/stats     # capture_fps, stream_fps, clients, jpeg_bytes
```

### Stop

Stop in reverse order with Ctrl+C: camera server (D), `keyboard_teleop` (C), then drivers (A). `keyboard_teleop` sends a final zero Twist on exit.

### After editing `keyboard_teleop.py`

`jetbot_bringup` runs from a copy in `build/`, so edits under `src/` take effect only after a rebuild:

```bash
# container
cd ~/ros2_ws && colcon build --packages-select jetbot_bringup && source install/setup.bash
```

`camera_server.py` and the HTML templates are read directly from the repo, so you only need to restart the server.

---

## Where the code lives

### Repository layout

```text
jetbot_ros2_mipt/
├── Dockerfile                    # app image: twist_mux, joystick, teleop_twist_joy, xacro, libserial
├── docker-compose.yml            # service jetbot_educational, devices, ROS_DOMAIN_ID=2, mounts ./src
├── docker_configs/               # base image (ROS 2 Humble, Nav2, slam_toolbox, ros2_control)
├── general_scripts/              # host-side tools (NOT mounted into the container)
│   ├── camera_server.py          # web teleop + camera server
│   ├── web/templates/            # teleop.html, base.html
│   ├── camera_x11_view.sh        # camera over ssh -X
│   ├── start_pi_camera_stream.sh # camera → H.264 over UDP for pi_camera
│   └── create_udev_rules.sh      # /dev/ttyMOTOR, /dev/ttyLIDAR symlinks
└── src/                          # ROS 2 workspace (mounted at /home/app/ros2_ws/src)
    ├── jetbot_bringup/           # launch files, configs, maps, keyboard_teleop
    ├── diffdrive_jetbot/         # ros2_control hardware plugin, URDF, controllers
    ├── pi_camera_ros/            # camera → /camera/image/compressed (nested repo)
    └── v4l2_camera/              # upstream V4L2 driver (nested repo)
```

### Node map

Upstream nodes come from the Docker image; the "Code" column points to what this repo controls: launch file, config, or source.

| Node / process | Runs in | Started by | Code in this repo |
|---|---|---|---|
| `keyboard_teleop` | container | `ros2 run jetbot_bringup keyboard_teleop` | `src/jetbot_bringup/jetbot_bringup/keyboard_teleop.py`; entry point in `src/jetbot_bringup/setup.py` |
| camera and teleop web server (not a ROS node) | host | `python3 general_scripts/camera_server.py` | `general_scripts/camera_server.py`; page in `general_scripts/web/templates/teleop.html` and `base.html` |
| `twist_mux` | container | `activate_all_drivers.launch.py` | launch: `src/jetbot_bringup/launch/activate_all_drivers.launch.py`; config: `src/jetbot_bringup/config/twist_mux.yaml`; upstream source cloned in `Dockerfile` |
| `rplidar_node` | container | `rplidar.launch.py`, included by `activate_all_drivers` | launch: `src/jetbot_bringup/launch/rplidar.launch.py`; upstream `rplidar_ros` from the base image |
| `controller_manager` (`ros2_control_node`) | container | `diffbot.launch.py`, included by `activate_all_drivers` | launch: `src/diffdrive_jetbot/bringup/launch/diffbot.launch.py`; config: `src/diffdrive_jetbot/bringup/config/diffbot_controllers.yaml` |
| `DiffDriveJetbotHardware` (plugin inside `controller_manager`) | container | loaded from the URDF `<ros2_control>` tag | `src/diffdrive_jetbot/hardware/diffbot_system.cpp`, `hardware/include/diffdrive_jetbot/diffbot_system.hpp`; serial protocol: `four_ch_motor_drive_comms.hpp`; encoder math: `wheel.hpp`; plugin export: `diffdrive_jetbot.xml`; serial and motor params: `description/ros2_control/diffbot.ros2_control.xacro` |
| `diffbot_base_controller`, `joint_state_broadcaster` | container | spawners in `diffbot.launch.py` | parameters in `diffbot_controllers.yaml`; upstream `ros2_controllers` |
| `robot_state_publisher` | container | `diffbot.launch.py` | URDF: `src/diffdrive_jetbot/description/urdf/diffbot.urdf.xacro`, `diffbot_description.urdf.xacro`, `diffbot.materials.xacro`; meshes in `description/meshes/` |
| `mesh_http_server` (`python3 -m http.server`) | container | `diffbot.launch.py` | serves `src/diffdrive_jetbot/description/` on port 8000 so RViz can load meshes |
| `joy_node`, `teleop_twist_joy_node` | container | manual `ros2 run` (see [Gamepad](#gamepad)) | config: `src/jetbot_bringup/config/joy.yaml`; upstream sources cloned in `Dockerfile` |
| `slam_toolbox` | container | manual `ros2 launch slam_toolbox ...` | config: `src/jetbot_bringup/config/slam_toolbox_online_async.yaml`; saved maps: `src/jetbot_bringup/maps/` |
| Nav2 | container | not wired to a launch file here | configs only: `src/jetbot_bringup/config/nav2_params.yaml`, `nav2_default_params.yaml` |
| `pi_image_compressed_publisher` | container | `ros2 run pi_camera publish_compressed` | `src/pi_camera_ros/pi_camera/publish_compressed.py`; also `publish_raw.py`, `view_compressed.py`; feed from `general_scripts/start_pi_camera_stream.sh` |
| `v4l2_camera_node` | container | `ros2 run v4l2_camera v4l2_camera_node` | `src/v4l2_camera/src/v4l2_camera_node.cpp`, `v4l2_camera.cpp`. The IMX219 exposes raw Bayer on `/dev/video0`, so this is not the useful camera path on this robot |
| RViz model view | container | `ros2 launch diffdrive_jetbot view_robot.launch.py` | `src/diffdrive_jetbot/description/launch/view_robot.launch.py`; configs in `description/rviz/` |
| PID tuning and motor test tools (not ROS) | container | manual build | `src/diffdrive_jetbot/tune_scripts/pid_tune.cpp`, `test_motor_comms.cpp`, `plots.py` |

---

## Topics, frames, and ports

The educational stack uses **no namespace**, so all topics are global.

| Topic | Type | Publisher → subscriber |
|---|---|---|
| `/cmd_vel_joy` | `geometry_msgs/Twist` | `keyboard_teleop` or `teleop_twist_joy` → `twist_mux` |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → `twist_mux` |
| `/cmd_vel_robot_steering` | `geometry_msgs/Twist` | `rqt_robot_steering` → `twist_mux` |
| `/diffbot_base_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` | `twist_mux` → `diffbot_base_controller` |
| `/diffbot_base_controller/odom` | `nav_msgs/Odometry` | `diffbot_base_controller` |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` → `robot_state_publisher` |
| `/scan` | `sensor_msgs/LaserScan` | `rplidar_node` → `slam_toolbox` / Nav2 |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` |
| `/joy` | `sensor_msgs/Joy` | `joy_node` → `teleop_twist_joy_node` |
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | `pi_image_compressed_publisher` |

**TF tree:** `map → odom` (slam_toolbox) `→ base_footprint` (diffbot_base_controller) `→ base_link → {left_wheel_link, right_wheel_link, caster_ball_link, lidar_link, camera_link}` (robot_state_publisher).

| Port | Process | Where |
|---|---|---|
| 8080 | camera and teleop web server | host |
| 8081 | `keyboard_teleop` HTTP API | container (host network) |
| 8000 | mesh HTTP server for RViz | container |
| 5004/udp | `start_pi_camera_stream.sh` → `pi_camera` | localhost |

---

## Other workflows

### Gamepad

The gamepad is not started by `activate_all_drivers`. Stop `keyboard_teleop` first, then run each of these in its own container terminal:

```bash
ros2 run joy_linux joy_linux_node --ros-args -r __node:=joy_node -p dev:=/dev/input/js0
```

```bash
ros2 run teleop_twist_joy teleop_node --ros-args \
  -r __node:=teleop_twist_joy_node \
  --params-file /home/app/ros2_ws/src/jetbot_bringup/config/joy.yaml \
  -r cmd_vel:=cmd_vel_joy
```

Hold **button 6 (L1)** while moving the stick. This is a dead-man switch; without it the robot does not move. Button 7 (R1) is turbo.

### SLAM mapping

With the drivers running and some way to drive (the web page or the gamepad):

```bash
# container
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/app/ros2_ws/src/jetbot_bringup/config/slam_toolbox_online_async.yaml

ros2 topic hz /map
ros2 run nav2_map_server map_saver_cli -f /home/app/ros2_ws/src/jetbot_bringup/maps/my_map \
  --ros-args -r map:=/map
```

The map is written as `my_map.pgm` and `my_map.yaml` into `src/jetbot_bringup/maps/` on the host, because `src/` is bind-mounted.

### Direct motor test (bypasses keyboard and joystick)

```bash
# container — the robot WILL move
ros2 topic pub -r 10 /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist \
  "{linear: {x: 0.08}, angular: {z: 0.0}}"
# Ctrl+C, then stop explicitly:
ros2 topic pub --once /diffbot_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{}"
```

If this moves the wheels but the web page does not, the problem is above `twist_mux`. If this does not move them either, check controllers, power, and the motor board.

### Camera without the web page

Only one process can own the CSI camera. Stop `camera_server.py` before using any of these.

- **X11 over SSH:** `ssh -Y jetbot@<jetson-ip>`, then `./general_scripts/camera_x11_view.sh --fps 5`. On Windows, use MobaXterm or VcXsrv as the X server.
- **ROS image topic:** run `./general_scripts/start_pi_camera_stream.sh` on the host, then `ros2 run pi_camera publish_compressed` in the container.

---

## Setup: devices, Docker, build

### Device names (once per robot)

```bash
# host
cd general_scripts
sudo ./create_udev_rules.sh
ls -la /dev/tty{MOTOR,LIDAR}
```

`99-usb-jetbot-lidar-motor.rules` maps the motor board to `/dev/ttyMOTOR` (a USB CH340 `1a86:7522`, or the Jetson UART `ttyTHS1`) and the RPLidar CP210x (`10c4:ea60`) to `/dev/ttyLIDAR`. On this robot `ttyMOTOR → ttyTHS1` and `ttyLIDAR → ttyUSB0`.

### Docker images

| Image | File | Contents |
|---|---|---|
| base `lupasic/jetbot_ros_humble_base:1.0` | `docker_configs/Dockerfile_base` | ROS 2 Humble, Nav2, slam_toolbox, ros2_control, RViz, rqt |
| app `jetbot_full_ros2` | `Dockerfile` | `twist_mux`, `joystick_drivers`, `teleop_twist_joy`, `rqt_robot_steering`, `topic_tools`, `xacro`, `libserial` |

Building the base image for arm64 on a PC:

```bash
cd docker_configs
./build_base_image_on_amd.sh tar jetbot_ros2_base.tar   # or: push <tag>
scp jetbot_ros2_base.tar jetbot@<jetson-ip>:~
# on the Jetson
docker load < jetbot_ros2_base.tar
```

Building the app image on the Jetson:

```bash
docker build -t jetbot_full_ros2 .
docker compose up -d
```

`docker compose restart` does not pick up edits to `docker-compose.yml`. After changing it, run `docker compose up -d --force-recreate`.

### Building the workspace

```bash
# container
cd ~/ros2_ws
colcon build --parallel-workers 2
source install/setup.bash
```

On the Nano, keep `--parallel-workers` low to avoid running out of memory.

---

## PID tuning

The motor PID gains are stored in the motor board's flash, not in ROS parameters or the URDF. `pid_tune` is the only way to change them.

```bash
# container
cd ~/ros2_ws/src/diffdrive_jetbot/tune_scripts
g++ -I../hardware/include/diffdrive_jetbot -o pid_tune pid_tune.cpp -lserial -lpthread
./pid_tune
```

Stop `activate_all_drivers` first, because `pid_tune` needs `/dev/ttyMOTOR` to itself.

The tool asks for the ultimate gain (Ku) and period (Tu) to seed Ziegler–Nichols gains, then runs a loop:

- `[t]` tests the current gains. Reboot the motor board when prompted; it prints a score, and lower is better.
- `[m]` lets you enter Kp, Ki, and Kd manually.
- `[s]` saves the best gains to the board and exits.

It writes a `.csv` of raw data and a `.txt` report; plot them with `plots.py`.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `LibSerial::OpenFailed` or `Device or resource busy` when launching drivers | Another process holds `/dev/ttyMOTOR`: a second launch, the old container, or `pid_tune` | Stop it; run the `open ok` preflight; use only one `activate_all_drivers` |
| `list_controllers` hangs or finds no service | `ros2_control_node` is not running or crashed | Read Terminal A's log; the cause is usually the serial port |
| `rplidar_node` dies with `80008002` | The lidar is not ready or not spinning | Replug the lidar USB and relaunch; the base still works without it |
| Page says nothing and keys do nothing | `keyboard_teleop` is not running, so the page's API calls return 503 | Start Terminal C |
| Keys work in one browser but not another | Single-client lock (HTTP 409) | Press Esc in the first browser, or wait 1 s after closing it |
| `/cmd_vel_joy` moves, the controller topic stays at zero | A higher-priority `twist_mux` source is publishing | `ros2 topic info /cmd_vel_robot_steering`; stop that publisher |
| Keyboard commands keep being cancelled | `teleop_twist_joy` is also publishing on `/cmd_vel_joy` | Run only one of them |
| Gamepad `/joy` is live but the robot does not move | Button 6 is not held | Hold L1 |
| Camera server fails to start | Another process owns the camera, or it is running in Docker or with `sudo` | Stop `camera_x11_view.sh` or `start_pi_camera_stream.sh`; run as the normal user on the host |
| Nothing is visible between host and container | Different `ROS_DOMAIN_ID` | Use `2` everywhere |

Useful checks:

```bash
ros2 node list | sort
ros2 control list_hardware_interfaces
ros2 topic info -v /cmd_vel_joy
ros2 run tf2_tools view_frames
```

---

## Known gaps

- `src/pi_camera_ros` and `src/v4l2_camera` are nested git repos with no `.gitmodules` file. A fresh clone gets empty folders; clone them by hand (`v4l2_camera` comes from `gitlab.com/boldhearts/ros2_v4l2_camera`, branch `humble`).
- `diffbot.launch.py` defaults `robot_ip` to `192.168.1.152` for mesh URLs. If RViz on another machine shows no meshes, pass `robot_ip:=<jetson-ip>`.
- There is no `mapping.launch.py`, `nav2_bringup.launch.py`, or `localization.launch.py` in this branch; they were removed for the educational build. Use `slam_toolbox`'s own launch file for mapping. Nav2 has params but no launch file here.
- The web API has no authentication. Anyone on the network who can reach port 8080 can take control when it is free.
