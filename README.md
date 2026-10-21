# mocap_distance_viz

Shows the travelled distance of OptiTrack rigid bodies tracked by `mocap_optitrack`.
For each body it computes the cumulative path length and the displacement from the start point.
The results are shown in the terminal running the node and, optionally, in RViz.

| Branch | ROS |
|---|---|
| `main` | ROS 1 Noetic (also builds on Melodic) |
| `humble` | ROS 2 Humble |

## Interface

### Input

Topics published by [mocap_optitrack](https://github.com/ros-drivers/mocap_optitrack) (`master` branch).

| `msg_type` | Topic | Type |
|---|---|---|
| `pose` (default) | `<name>/pose` | `geometry_msgs/PoseStamped` |
| `odom` | `<name>/Odom` | `nav_msgs/Odometry` |

### Output

| Topic / Service | Type | Description |
|---|---|---|
| `<name>/distance` | `std_msgs/Float64` | Cumulative path length [m] |
| `<name>/displacement` | `std_msgs/Float64` | Distance from the start point [m] |
| `<name>/path` | `nav_msgs/Path` | Travelled path (latched) |
| `/mocap_distance_viz/markers` | `visualization_msgs/MarkerArray` | RViz markers |
| `/mocap_distance_viz/reset` | `std_srvs/Trigger` | Reset all measurements |

### Terminal output

In a terminal, the node shows a table that is updated in place:

| Column | Description |
|---|---|
| `BODY` | Rigid body name, with its RViz marker color |
| `PATH` | Cumulative path length [m] |
| `DISP` | Distance from the start point [m] |
| `SPEED` | Average speed over the last second [m/s] |
| `POSITION` | Current position [m] (hidden in narrow terminals) |
| `STATUS` | `tracking`, `lost` (no data for `stale_timeout`), or `waiting` |

When the output is redirected to a file or pipe, the node logs a one-line summary instead.

## Distance computation

- A step is added only once the body has moved at least `min_step` from the last counted point, so mocap jitter is not accumulated.
- A step larger than `max_step` is treated as a tracking loss and is not counted.
- With `planar: true`, the z axis is ignored.

## Parameters

Set in `config/distance_viz.yaml`.

| Name | Default | Description |
|---|---|---|
| `rigid_bodies` | `["Robot_1"]` | Rigid body names (topic prefixes) |
| `msg_type` | `pose` | `pose` or `odom` |
| `topic_suffix` | `""` | Topic suffix; empty uses `pose` or `Odom` |
| `planar` | `false` | Ignore the z axis |
| `min_step` | `0.005` | Minimum counted step [m] |
| `max_step` | `0.5` | Maximum counted step [m]; ≤ 0 disables the check |
| `max_path_length` | `20000` | Poses kept in the path; ≤ 0 means unlimited |
| `publish_rate` | `10.0` | Distance and marker publish rate [Hz] |
| `path_publish_rate` | `2.0` | Path publish rate [Hz] |
| `display` | `auto` | Terminal output: `auto`, `live`, `log` or `off` |
| `display_rate` | `5.0` | Terminal refresh rate [Hz] |
| `log_period` | `1.0` | Summary period in `log` mode [s] |
| `stale_timeout` | `0.5` | Time without data before `lost` [s] |
| `text_height` | `0.12` | RViz text size [m] |
| `text_offset_z` | `0.35` | RViz text height above the body [m] |

## Usage

```bash
cd ~/catkin_ws/src
git clone https://github.com/Leeseunghun03/mocap_distance_viz.git
cd ~/catkin_ws && catkin_make && source devel/setup.bash

roslaunch mocap_optitrack mocap.launch
roslaunch mocap_distance_viz distance_viz.launch
```

| Launch argument | Default | Description |
|---|---|---|
| `config` | `config/distance_viz.yaml` | Parameter file |
| `rviz` | `false` | Start RViz |
| `fake` | `false` | Publish simulated mocap data |

```bash
rosservice call /mocap_distance_viz/reset
```

Start the nodes with `rosrun` or `roslaunch`. The files in `scripts/` are not executable on their own.

### Multiple rigid bodies

Set `rigid_bodies` to the topic prefixes used in `mocap.yaml`, e.g. `["Robot_1", "Robot_2"]`.
The RViz config shows the path of `Robot_1` only. For other bodies, add a Path display for `/<name>/path`.

## Tests

```bash
catkin_make run_tests && catkin_test_results build/test_results
```
