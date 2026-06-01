# Archived code

These files are **not part of the active line-follower / traffic-light stack**.
They come from earlier TE3002B course modules (square trajectory, waypoint
following, Kalman localization, visual MPC servoing, ArUco, Gazebo sims) and were
moved here on 2026-05-31 to keep the runtime package focused.

Nothing here is built or installed: the nodes were removed from `setup.py`
`console_scripts`, and the launch files are no longer picked up by the
`launch/*.launch.py` glob.

## nodes/

| File | Original purpose |
| --- | --- |
| `line_detector.py` | Older dual-ROI line follower. **Superseded by `puzzlebot_ros/line_follower.py`.** Note: it registered the same ROS node name `line_follower`, so running both at once would clash. |
| `vision_node.py` | Visual-servoing perception publishing `/ex`, `/area`, `/object_detected`. Feeds `mpc_node.py`. |
| `mpc_node.py` | Visual MPC controller consuming `vision_node.py`. |
| `trajectory_generator.py` | Generates `/waypoint` poses (circle/period). |
| `pid_waypoint_follower.py` | PID follower for `/waypoint` using `/odom`. |
| `pid_square_controller.py` | Closed-loop PID for a 2x2 m square path. |
| `odom_node.py` | Simple differential-drive odometry. |
| `trafficlight_waypoint.py` | Combined Kalman + waypoint + traffic-light activity node. |

## launch/

ROS2 / Gazebo launch files from other modules: `gazebo_box`, `gazebo_aruco`,
`gazebo_empty`, `aruco_jetson`, `goto_kalman`, `marker_publisher`.

## How to restore one

```bash
git mv archive/nodes/<file>.py puzzlebot_ros/
# then re-add its entry_point in setup.py
```
