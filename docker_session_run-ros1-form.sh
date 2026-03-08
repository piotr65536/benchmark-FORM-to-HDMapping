#!/bin/bash
# Run FORM LiDAR odometry on a rosbag and convert output to HDMapping session format.
#
# Usage:
#   ./docker_session_run-ros1-form.sh <input.bag> <output_dir>
#
# Or with no arguments to use a GUI file selector (requires zenity).

set -e

IMAGE_NAME='form_noetic'
TMUX_SESSION='ros1_form'

DATASET_CONTAINER_PATH='/ros_ws/dataset/input.bag'
BAG_OUTPUT_CONTAINER='/ros_ws/recordings'

RECORDED_BAG_NAME='recorded-form.bag'
HDMAPPING_OUT_NAME='output_hdmapping'

usage() {
    echo "Usage:"
    echo "  $0 <input.bag> <output_dir>"
    echo
    echo "  input.bag   — ROS bag with /velodyne_points (sensor_msgs/PointCloud2)"
    echo "  output_dir  — directory where HDMapping session files will be written"
    exit 1
}

echo "=== FORM → HDMapping pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    usage
fi

if [[ $# -eq 2 ]]; then
    DATASET_HOST_PATH="$1"
    BAG_OUTPUT_HOST="$2"
elif [[ $# -eq 0 ]]; then
    command -v zenity >/dev/null 2>&1 || { echo "Error: zenity not available. Provide paths as arguments."; exit 1; }
    DATASET_HOST_PATH=$(zenity --file-selection --title="Select input BAG file")
    BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
    usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
    echo "Error: no file or directory selected."
    exit 1
fi

if [[ ! -f "$DATASET_HOST_PATH" ]]; then
    echo "Error: BAG file does not exist: $DATASET_HOST_PATH"
    exit 1
fi

mkdir -p "$BAG_OUTPUT_HOST"
DATASET_HOST_PATH=$(realpath "$DATASET_HOST_PATH")
BAG_OUTPUT_HOST=$(realpath "$BAG_OUTPUT_HOST")

echo "Input bag : $DATASET_HOST_PATH"
echo "Output dir: $BAG_OUTPUT_HOST"

xhost +local:docker >/dev/null 2>&1 || true

# ── Phase 1: run FORM + record output topics ──────────────────────────────────
docker run -it --rm \
    --network host \
    -e DISPLAY="$DISPLAY" \
    -e ROS_HOME=/tmp/.ros \
    -u 1000:1000 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
    -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
    "$IMAGE_NAME" \
    /bin/bash -c "
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash

tmux new-session -d -s $TMUX_SESSION

# ---- Pane 0: roscore ----
tmux send-keys -t $TMUX_SESSION '
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
roscore
' C-m

# ---- Pane 1: FORM ROS node ----
tmux split-window -v -t $TMUX_SESSION
tmux send-keys -t $TMUX_SESSION '
sleep 3
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
roslaunch form_ros_node form.launch use_sim_time:=true
' C-m

# ---- Pane 2: rosbag record ----
tmux split-window -v -t $TMUX_SESSION
tmux send-keys -t $TMUX_SESSION '
sleep 5
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo \"[record] starting\"
rosbag record /form/registered_cloud /form/odometry -O $BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME
echo \"[record] done\"
' C-m

# ---- Pane 3: rosbag play ----
tmux split-window -h -t $TMUX_SESSION
tmux send-keys -t $TMUX_SESSION '
sleep 7
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo \"[play] starting\"
rosbag play $DATASET_CONTAINER_PATH --clock
echo \"[play] done\"
tmux wait-for -S PLAY_DONE
' C-m

# ---- Control window: wait for play, then clean up ----
tmux new-window -t $TMUX_SESSION -n control
tmux send-keys -t ${TMUX_SESSION}:control '
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo \"[control] waiting for bag playback to finish\"
tmux wait-for PLAY_DONE
echo \"[control] stopping rosbag record\"
tmux send-keys -t ${TMUX_SESSION}:0.2 C-c
sleep 5
echo \"[control] killing ROS nodes\"
rosnode kill -a || true
sleep 3
pkill -f roslaunch || true
pkill -f roscore   || true
' C-m

tmux attach -t $TMUX_SESSION
"

# ── Phase 2: convert recorded bag to HDMapping session ────────────────────────
echo "=== Converting recorded bag to HDMapping session ==="

docker run -it --rm \
    --network host \
    -e ROS_HOME=/tmp/.ros \
    -u 1000:1000 \
    -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
    "$IMAGE_NAME" \
    /bin/bash -c "
set -e
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
rosrun form-to-hdmapping listener \
    \"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME\" \
    \"$BAG_OUTPUT_CONTAINER/${HDMAPPING_OUT_NAME}-form\"
"

echo "=== DONE ==="
echo "HDMapping session written to: $BAG_OUTPUT_HOST/${HDMAPPING_OUT_NAME}-form/"
