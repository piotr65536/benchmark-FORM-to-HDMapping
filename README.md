## Hint

Please change branch to [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-FORM-to-HDMapping/tree/Bunker-DVI-Dataset-reg-1) for quick experiment.

## Example Dataset:

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

# benchmark-FORM-to-HDMapping

Runs the [FORM](https://github.com/rpl-cmu/form) LiDAR odometry algorithm on a ROS bag
file and converts the output to an [HDMapping](https://github.com/MapsHD/HDMapping) session.

FORM (Fixed-Lag Odometry with Reparative Mapping) is a real-time LiDAR odometry system
from Carnegie Mellon University's Robotics Perception Lab (2025).

## Prerequisites

- Docker
- A ROS bag containing a `/velodyne_points` topic (`sensor_msgs/PointCloud2`)
  — ideally with a `ring` field (Velodyne driver standard)

## Step 1 — Clone with submodules

```bash
git clone https://github.com/MapsHD/benchmark-FORM-to-HDMapping.git --recursive
cd benchmark-FORM-to-HDMapping
```

> **Note**: Internet access is required at `docker build` time because FORM fetches
> the `tsl::robin_map` dependency via CMake FetchContent.

## Step 2 — Build the Docker image

```bash
docker build -t form_noetic .
```

This installs:
- Ubuntu 20.04 + ROS Noetic
- GTSAM 4.2 (from the borglab PPA)
- Intel TBB, Eigen3, PCL
- FORM C++ library (compiled from submodule)
- catkin workspace with `form_ros_node` and `form-to-hdmapping`

The build takes several minutes on first run.

## Step 3 — Run the pipeline

```bash
chmod +x docker_session_run-ros1-form.sh
./docker_session_run-ros1-form.sh /path/to/input.bag /path/to/output/dir
```

Or with no arguments to use a GUI file selector (requires `zenity`):

```bash
./docker_session_run-ros1-form.sh
```

**What happens:**

The script opens a Docker container with a tmux session containing four panes:

| Pane | Role |
|------|------|
| 0 | `roscore` |
| 1 | `form_ros_node` — reads `/velodyne_points`, publishes `/form/odometry` + `/form/registered_cloud` |
| 2 | `rosbag record` — captures the two published topics |
| 3 | `rosbag play` — plays your input bag with simulated clock |

After playback completes, recording is stopped and a second Docker run converts
`recorded-form.bag` into the HDMapping session format.

## Step 4 — Open in HDMapping

Output files appear in `<output_dir>/output_hdmapping-form/`:

```
lio_initial_poses.reg
poses.reg
scan_lio_0.laz
scan_lio_1.laz
...
session.json
trajectory_lio_0.csv
trajectory_lio_1.csv
...
```

Open `session.json` with the
[multi_view_tls_registration_step_2](https://github.com/MapsHD/HDMapping) application.

## Notes on point cloud format

The `form_ros_node` handles three input cloud types automatically:

| Cloud type | Handling |
|---|---|
| Organized + `ring` field | Ring field used as row index (best accuracy, Velodyne standard) |
| Organized, no `ring` field | Row index = `point_index / width` |
| Unorganized (`height == 1`) | Elevation-angle bucketing assigns synthetic rings |

Adjust `num_rows` and `num_columns` in [src/form-ros-node/launch/form.launch](src/form-ros-node/launch/form.launch)
to match your sensor (default: 64 rows × 1024 columns for Velodyne HDL-64E).

## Sensor geometry presets

| Sensor | num_rows | num_columns |
|--------|----------|-------------|
| Velodyne HDL-64E | 64 | 1024 |
| Velodyne VLP-16 | 16 | 1800 |
| Ouster OS1-64 | 64 | 1024 |
| Ouster OS0-128 | 128 | 1024 |

## Contact

januszbedkowski@gmail.com
