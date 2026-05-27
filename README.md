# Puzzlebot Line Follower

ROS2 Humble package copied from the working Jetson workspace and set up for the
laptop/Jetson workflow used in the Manchester projects.

## Layout

- `puzzlebot_ros/line_follower.py`: current autonomous racer node. It reads the
  Jetson CSI camera, follows the line, handles traffic-light state, publishes
  `/cmd_vel`, and exposes MJPEG on `http://10.10.0.100:8080`.
- `puzzlebot_ros/line_detector.py`: simpler dual-ROI line follower.
- `launch/`: original Puzzlebot launch files from the Jetson package.
- `scripts/`: sync, build, run, tmux demo, and stop helpers.

## Jetson Defaults

The scripts default to:

```bash
JETSON_USER=puzzlebot
JETSON_HOST=10.10.0.100
REMOTE_WS=/home/puzzlebot/ros2_ws
```

Override any of them inline if needed:

```bash
JETSON_HOST=10.10.0.101 scripts/run_demo_tmux.sh
```

## Common Flow

From this repo on the laptop:

```bash
scripts/sync_to_jetson.sh
scripts/build_on_jetson.sh
scripts/run_line_follower_jetson.sh
```

For the full tmux workflow:

```bash
scripts/run_demo_tmux.sh
```

That opens panes for sync/build, micro-ROS agent, line follower, and topic
monitoring. The camera preview stream is available at:

```text
http://10.10.0.100:8080
```

Stop safely with:

```bash
scripts/stop_demo.sh
```
