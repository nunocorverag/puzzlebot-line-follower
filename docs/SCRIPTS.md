# Scripts & Tools Reference

Catalog of every helper script and standalone tool, what it does, and when to
run it. The laptop is the dev machine; the Jetson (`puzzlebot@10.10.0.100`) is
the runtime target.

## Shared conventions

All `scripts/*.sh` honor these environment overrides (with defaults):

```bash
JETSON_USER=puzzlebot
JETSON_HOST=10.10.0.100
REMOTE_WS=/home/puzzlebot/ros2_ws        # remote workspace
# remote package = ${REMOTE_WS}/src/puzzlebot_ros
```

Override inline, e.g. `JETSON_HOST=10.10.0.101 scripts/run_line_follower_jetson.sh`.

The shared boilerplate (SSH/sourcing, laptop-IP autodetection, camera freeing,
H264 receiver) lives in **`scripts/lib/common.sh`**; the `run_*_jetson.sh`
scripts source it so they all behave the same.

### Preview / streaming: the `STREAM` variable

Every camera tool honours one variable, `STREAM`:

| `STREAM` | What you get |
| --- | --- |
| `h264` (default) | Hardware H264/RTP to the laptop; the receiver opens automatically. Lowest bandwidth, best over WiFi. |
| `local` | `cv2.imshow` window forwarded over `ssh -X` (needs a DISPLAY). |
| `none` | Headless — no preview at all. |

```bash
scripts/run_sign_detector_jetson.sh                 # H264 (default)
STREAM=local scripts/run_sign_detector_jetson.sh    # ssh -X window
STREAM=none  scripts/run_recorder_jetson.sh         # headless capture
```

If the H264 encoder can't open it falls back to a local window automatically.
The line calibrator defaults to an H264 dashboard; use `STREAM=local` only when
you need real OpenCV trackbars over `ssh -X`. **Every camera tool opens the CSI
camera itself — there is no separate "start the camera first" step.** The CSI is
single-owner, so run one at a time; the scripts free it before starting.

---

## Daily line-follower workflow

| Script | What it does | When to run |
| --- | --- | --- |
| `sync_to_jetson.sh` (canonical) | `rsync` the repo to the Jetson package dir (excludes build/datasets), then `chmod +x` the scripts. | Before every build/run; most run-scripts call it for you. |
| `build_on_jetson.sh` (canonical) | `colcon build --packages-select puzzlebot_ros` on the Jetson. | After syncing changed Python that ROS needs installed (entry points/launch). |
| `run_line_follower_jetson.sh` | Runs `ros2 run puzzlebot_ros line_follower` (the autonomous racer). MJPEG at `http://10.10.0.100:8080`. Override node with `NODE=`. | Real autonomous run. Wheels-up first. |
| `run_demo_tmux.sh` | tmux session: sync/build + micro-ROS agent + line follower + topic monitor. | Full demo orchestration. |
| `stop_demo.sh` (canonical) | Kills tmux, kills `line_follower`/`autonomous_racer`, publishes a zero `/cmd_vel` burst, stops micro-ROS agent. | **Emergency stop / clean shutdown.** Keep it in a ready terminal. |
| `kill_all_jetson.sh` | `pkill` of recorder/camera/agent processes on the Jetson. | Quick cleanup of stray processes. |
| `start_all_jetson.sh` | `nohup` launch of camera + recorder in background with logs in `/tmp`. | Headless data-collection sessions. |

## Calibration & perception tuning

| Script / Tool | What it does | When to run |
| --- | --- | --- |
| `run_line_calibrator_jetson.sh` | Syncs, then runs `tools/line_vision_calibrator.py --gstreamer`. Default `STREAM=h264` shows a fast dashboard (overlay + Otsu mask + state panel). Use `STREAM=local` for the old OpenCV trackbars. | Tune intersection/mask params live. **Primary perception playground.** |
| `tools/line_vision_calibrator.py` | The calibrator itself. Live trackbars, mask/overlay/state windows, saves labeled samples. Also runs offline on saved images: `--image path.jpg`. | Live on Jetson or offline tuning on the laptop. |
| `set_calibrator_param.sh` | Writes `PARAM=VALUE` (or `label X`) into the calibrator command file the tool watches. | Adjust parameters while the H264 calibrator is running, e.g. `scripts/set_calibrator_param.sh min_dash_count 6`. |
| `run_illumination_calibrator_jetson.sh` + `tools/illumination_calibrator.py` | Auto-guided flat-field capture over H264; averages good white-surface frames, writes `config/illumination_flatfield.npz`, pulls it back. | **Illumination calibration** — see [CALIBRATION_ILLUMINATION.md](CALIBRATION_ILLUMINATION.md). |
| `run_focus_assist_jetson.sh` + `tools/focus_assist.py` | Live sharpness meter over H264 with peak-hold; twist the lens to maximize. | **Set focus first** (before camera calibration). |
| `run_checkerboard_capture_jetson.sh` + `tools/calib_capture_checkerboard.py` | Auto-guided checkerboard capture over H264 (pose-diversity hints); pulls images to `calibration_images/`. | **Camera calibration, step 1** — see [CALIBRATION_CHECKERBOARD.md](CALIBRATION_CHECKERBOARD.md). |
| `run_calibrate_camera_jetson.sh` + `tools/calibrate_camera.py` | Computes intrinsics on the Jetson (drops outliers, reports RMS), pulls back `config/camera_params.npz` + preview. | **Camera calibration, step 2.** |
| `pull_calibration_dataset.sh` | `rsync` the Jetson `debug_dataset/` down to `debug_dataset/jetson/`. | After a labeling session, to tune offline. |

## Robot motion (use with care)

> **The wheels only move if the micro-ROS motor bridge is running.** The follower
> / teleop / jog all just *publish* `/cmd_vel`; the thing that drives the motors
> is `micro_ros_agent` (serial `/dev/ttyUSB0`, started by `~/start_robot.sh` →
> `ros2 launch puzzlebot_ros micro_ros_agent.launch.py`). It also brings up the
> encoder topics (`/VelocityEncL`, `/VelocityEncR`, `/robot_vel`). If
> `ros2 topic info /cmd_vel` shows **Subscription count: 0**, the bridge is not
> running and nothing will move. It does **not** touch the camera, so it runs
> alongside the follower (unlike `start_all_jetson.sh`, whose recorder grabs the
> single-owner CSI and conflicts).
>
> **Motor deadband ≈ 0.08–0.10 m/s.** 0.05 m/s does not overcome static friction;
> use ≥0.10 for jogs and creeps. If logic/encoders work but wheels don't move at
> all, suspect motor **power/battery**, not software.
>
> **Only one `/cmd_vel` publisher at a time.** Stop the follower before teleop/jog
> (`ssh puzzlebot@10.10.0.100 'pkill -f line_follower'`) or they fight.

| Script | What it does | When to run |
| --- | --- | --- |
| `run_motor_agent_jetson.sh` | Starts the micro-ROS agent (the `/cmd_vel` → motors bridge) via `~/start_robot.sh`. Foreground; Ctrl-C stops. | **Run this first** for any motion (teleop/jog/follower). Safe alongside the follower. |
| `run_teleop_wasd_jetson.sh` + `tools/teleop_wasd.py` | Robust real-time WASD teleop. **Hold-to-go** (release a key → stops after `HOLD_TIMEOUT`≈0.4 s; `HOLD_TIMEOUT=0` = sticky), one tap = real speed, depth-1 QoS + 50 Hz republish, zeroes on exit/SSH drop. `w/s`=fwd/back, `a/d`=**steer** (gentle, so `w`+`a` is a real curve, not a one-wheel pivot), `q/e`=**pivot in place** (strong). `z` straighten, `x` stop-linear, space STOP, `-`/`=` scale, Ctrl-C quit. Tune `V_MAX`/`STEER_W`/`PIVOT_W`. | Manual driving with combined moves. |
| `run_teleop_wasd_combo.sh` + `tools/teleop_wasd_gui.py` + `tools/cmd_vel_udp_bridge.py` | **True-combo** WASD: reads the **laptop** keyboard with real key state (pygame window) so holding `w`+`a` together is a genuine curve, and sends velocity over **UDP** to a Jetson bridge that republishes `/cmd_vel` (no ROS on the laptop; bridge has a 0.3 s watchdog). A terminal/SSH teleop physically *cannot* do simultaneous key holds — this can. Needs `python3-pygame` on the laptop (`sudo apt install python3-pygame`). Keys: `w/s` `a/d` `q/e`, space stop, `-`/`=` speed, ESC quit. | Manual driving when you need real simultaneous combos. |
| `jog_forward_jetson.sh SPEED DURATION` | Publishes `/cmd_vel linear.x=SPEED` for `DURATION` s, then zero. Use ≥0.10 (deadband). e.g. `0.10 1.5`. | Move forward for visual ROI tests **without** the autonomous node. Never alongside `line_follower`. |
| `teleop_jetson.sh` | `teleop_twist_keyboard` over SSH (keys `i/,/j/l`, `k`=stop). | Manual driving (single-axis). |
| `set_drive_jetson.sh on\|off` | Toggles the follower's motion master switch via `/drive_enable` (Bool). The follower starts with driving **disabled**. | Enable/halt the follower's motion during testing without killing it. |

### Testing the follower's approach-and-center

The follower loads the tuned detection params the calibrator saves via
`save_calib` (`config/intersection_params.json`); without it, built-in defaults.
On detecting an intersection it enters `APPROACH_CENTER` — creeps forward
(`approach_speed`, default 0.10, **above the deadband**) while centering on the
fitted entry line, until the zebra reaches `approach_target_entry_y_pct` (82) and
is centered, then stops and waits for a `/intersection_decision`.

```bash
# 0. Tune the entry band in the calibrator, then persist it:
scripts/set_calibrator_param.sh save_calib 1     # -> config/intersection_params.json
scripts/build_on_jetson.sh                       # node code changes need a rebuild

# 1. Motor bridge (own terminal):
scripts/run_motor_agent_jetson.sh
# 2. Follower in test mode (no traffic light needed):
IGNORE_TRAFFIC_LIGHT=1 scripts/run_line_follower_jetson.sh
# 3. When wheels are clear, allow motion (and halt anytime):
scripts/set_drive_jetson.sh on
scripts/set_drive_jetson.sh off
```

## Camera & data capture

| Script / Tool | What it does | When to run |
| --- | --- | --- |
| `run_camera_jetson.sh` | `ros2 launch puzzlebot_ros camera_jetson.launch.py` (publishes `/video_source/raw`). | Bring up the CSI camera **topic** for other ROS nodes. The tools no longer need it — they open the camera directly. |
| `run_recorder_jetson.sh` + `tools/recorder.py` | Periodically saves frames to `dataset/`. | Collect a raw image dataset. |
| `run_teleop_recorder_jetson.sh` + `tools/teleop_recorder.py` | Drive + record simultaneously. | Build a driving dataset. |
| `run_sign_detector_jetson.sh` + `tools/sign_detector.py` | YOLO (`config/best.pt`) traffic-sign / light detection with live preview. Opens the camera itself; honours `STREAM`. Auto-installs `ultralytics`. | Validate the trained YOLO model on the Jetson. |

## Camera preview / streaming

The follower can stream the annotated frame two ways:

There are two different things you can view; pick by intent:

| What you see | Script | Notes |
| --- | --- | --- |
| **Raw camera, no overlays** | `scripts/run_camera_h264_jetson.sh` | Pure GStreamer (no ROS, no line follower). Just the camera. Lowest latency. |
| **Line follower's annotated view** (ROI boxes, anchors, steering line) | `scripts/run_line_follower_h264.sh` | Runs the autonomous racer and streams the frame it draws on. For debugging perception. |

> The CSI camera allows only ONE process at a time. Run either the follower or
> the raw-camera preview, never both. Stop a running follower with
> `scripts/stop_demo.sh` before switching.

Transport options for the follower stream:

| Mode | How | Notes |
| --- | --- | --- |
| MJPEG (default) | Open `http://10.10.0.100:8080` in a browser. | Per-frame JPEG. Tune with `stream_fps`, `stream_quality`, `stream_max_width` ROS params. Event-driven (each frame sent once). |
| H264/RTP over UDP | Hardware-encoded (`nvv4l2h264enc`), much lower bandwidth. | Opt-in. Inter-frame compression; best for laggy WiFi. |

H264 workflow (one command, IP auto-detected):

```bash
# Starts follower in H264 mode AND opens the receiver window. Ctrl+C stops both.
scripts/run_line_follower_h264.sh
```

The laptop IP is auto-detected from the route to the Jetson. On the `RoboNet`
connection (ipv4.method=shared) the laptop is always `10.10.0.1`, so you never
set it. Manual/split equivalent:

```bash
STREAM=h264 scripts/run_line_follower_jetson.sh        # H264_HOST auto-detected
scripts/view_h264_stream.sh                            # on the laptop (manual receiver)
```

Tuning ROS params (also work via the run-script env or `--ros-args`):
`stream_fps` (15), `stream_quality` (60), `stream_max_width` (0=full),
`h264_bitrate` (2000000 bits/s), `h264_port` (5000). If the H264 writer fails to
open it falls back to MJPEG automatically.

## Camera & illumination calibration (recommended)

The current `config/camera_params.npz` was calibrated from a **different
camera's** image set (`activity_2_07`, rms≈2.23). Recalibrate this CSI camera in
its mounted pose, then redo the illumination flat-field. Both are auto-guided and
previewed over H264:

```bash
# 1. Camera intrinsics (checkerboard)
scripts/run_checkerboard_capture_jetson.sh   # auto-guided capture
scripts/run_calibrate_camera_jetson.sh       # compute -> config/camera_params.npz
git add config/camera_params.npz && git commit -m "Recalibrate intrinsics"

# 2. Illumination flat-field (white banner) — AFTER step 1
scripts/run_illumination_calibrator_jetson.sh
git add config/illumination_flatfield.npz && git commit -m "Recalibrate flat-field"
```

Full step-by-step with sampling tips:
- [CALIBRATION_CHECKERBOARD.md](CALIBRATION_CHECKERBOARD.md)
- [CALIBRATION_ILLUMINATION.md](CALIBRATION_ILLUMINATION.md)
- [PERCEPTION_TUNING.md](PERCEPTION_TUNING.md)
