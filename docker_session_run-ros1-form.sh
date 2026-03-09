#!/bin/bash

IMAGE_NAME='form_noetic'
TMUX_SESSION='ros1_form'

DATASET_CONTAINER_PATH='/ros_ws/dataset/input.bag'
BAG_OUTPUT_CONTAINER='/ros_ws/recordings'

RECORDED_BAG_NAME="recorded-form.bag"
HDMAPPING_OUT_NAME="output_hdmapping"

LIDAR_TOPIC=/livox/pointcloud
LIDAR_NUM_ROWS=6 
LIDAR_NUM_COLS=4000

usage() {
  echo "Usage:"
  echo "  $0 <input.bag> <output_dir>"
  echo
  echo "If no arguments are provided, a GUI file selector will be used."
  echo
  echo "Environment variables:"
  echo "  LIDAR_NUM_ROWS  - number of LiDAR rows    (default: 64)"
  echo "  LIDAR_NUM_COLS  - number of LiDAR columns  (default: 1024)"
  echo "  LIDAR_TOPIC     - input PointCloud2 topic   (default: /velodyne_points)"
  exit 1
}

echo "=== FORM rosbag pipeline ==="

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
fi

if [[ $# -eq 2 ]]; then
  DATASET_HOST_PATH="$1"
  BAG_OUTPUT_HOST="$2"
elif [[ $# -eq 0 ]]; then
  command -v zenity >/dev/null || {
    echo "Error: zenity is not available"
    exit 1
  }
  DATASET_HOST_PATH=$(zenity --file-selection --title="Select BAG file")
  BAG_OUTPUT_HOST=$(zenity --file-selection --directory --title="Select output directory")
else
  usage
fi

if [[ -z "$DATASET_HOST_PATH" || -z "$BAG_OUTPUT_HOST" ]]; then
  echo "Error: no file or directory selected"
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
echo "LiDAR     : $LIDAR_NUM_ROWS rows x $LIDAR_NUM_COLS cols"
echo "Topic     : $LIDAR_TOPIC"

xhost +local:docker >/dev/null

# ── Pre-flight: inspect the input bag ─────────────────────────────────────────
echo ""
echo "=== Inspecting input bag ==="
docker run --rm \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  "$IMAGE_NAME" \
  /bin/bash -c "
    source /opt/ros/noetic/setup.bash
    echo '--- Topics in bag ---'
    rosbag info '$DATASET_CONTAINER_PATH' --yaml | grep -E '^\s*- topic:|^\s*type:|^\s*count:'
    echo ''
    echo '--- Full bag info ---'
    rosbag info '$DATASET_CONTAINER_PATH'
  "
echo "=== End bag info ==="
echo ""
echo "If the topic name above does not match '$LIDAR_TOPIC', re-run with:"
echo "  LIDAR_TOPIC=/your/topic $0 ..."
echo ""

# ── Phase 1: run FORM + record output topics ──────────────────────────────────
docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DATASET_HOST_PATH":"$DATASET_CONTAINER_PATH":ro \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c '

    tmux new-session -d -s '"$TMUX_SESSION"'

    # ---------- PANE 0: roscore ----------
    tmux send-keys -t '"$TMUX_SESSION"' '\''
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
roscore
'\'' C-m

    # ---------- PANE 1: FORM ROS node ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 5
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
roslaunch form_ros_node form.launch use_sim_time:=true input_topic:='"$LIDAR_TOPIC"' num_rows:='"$LIDAR_NUM_ROWS"' num_columns:='"$LIDAR_NUM_COLS"'
'\'' C-m

    # ---------- PANE 2: rosbag record ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 2
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[record] start"
rosbag record /form/registered_cloud /form/odometry -O '"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME"'
echo "[record] exit"
'\'' C-m

    # ---------- PANE 3: rosbag play ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 7
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[play] start"
rosbag play '"$DATASET_CONTAINER_PATH"' --clock; tmux wait-for -S BAG_DONE;
echo "[play] done"
'\'' C-m

    # ---------- PANE 4: RViz ----------
    tmux split-window -v -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 6
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[rviz] launching with FORM config"
rviz -d /ros_ws/src/form-ros-node/rviz/form.rviz
echo "[rviz] exit"
'\'' C-m

    # ---------- PANE 5: diagnostics ----------
    tmux split-window -h -t '"$TMUX_SESSION"'
    tmux send-keys -t '"$TMUX_SESSION"' '\''sleep 8
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "=== ROS DIAGNOSTICS ==="
echo ""
echo "--- Active topics ---"
rostopic list
echo ""
echo "--- Checking input topic: '"$LIDAR_TOPIC"' ---"
timeout 5 rostopic hz '"$LIDAR_TOPIC"' 2>&1 &
echo ""
echo "--- Checking FORM output: /form/odometry ---"
timeout 5 rostopic hz /form/odometry 2>&1 &
echo ""
echo "--- Checking FORM output: /form/registered_cloud ---"
timeout 5 rostopic hz /form/registered_cloud 2>&1 &
wait
echo ""
echo "=== If input topic shows messages but FORM output shows nothing ==="
echo "=== then sensor geometry (rows/cols) is likely wrong.           ==="
echo ""
echo "--- Node list ---"
rosnode list
echo ""
echo "--- form_ros_node info ---"
rosnode info /form_ros_node 2>/dev/null || echo "(node not found)"
echo ""
echo "[diag] done — you can type ROS1 commands here, e.g.:"
echo "  rostopic list"
echo "  rostopic echo /form/odometry"
echo "  rostopic hz /velodyne_points"
echo "  rosnode info /form_ros_node"
'\'' C-m

    # ---------- Control window ----------
    tmux new-window -t '"$TMUX_SESSION"' -n control '\''
source /opt/ros/noetic/setup.bash
source /ros_ws/devel/setup.bash
echo "[control] waiting for play end"
tmux wait-for BAG_DONE
echo "[control] stop record"
tmux send-keys -t '"$TMUX_SESSION"'.2 C-c

sleep 5
rosnode kill -a

sleep 7
pkill -f ros || true
'\''

    tmux attach -t '"$TMUX_SESSION"'
  '

# ── Phase 2: convert recorded bag to HDMapping session ────────────────────────
echo "=== Converting recorded bag to HDMapping session ==="

docker run -it --rm \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e ROS_HOME=/tmp/.ros \
  -u 1000:1000 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$BAG_OUTPUT_HOST":"$BAG_OUTPUT_CONTAINER" \
  "$IMAGE_NAME" \
  /bin/bash -c "
    set -e
    source /opt/ros/noetic/setup.bash
    source /ros_ws/devel/setup.bash
    rosrun form-to-hdmapping listener \
      \"$BAG_OUTPUT_CONTAINER/$RECORDED_BAG_NAME\" \
      \"$BAG_OUTPUT_CONTAINER/$HDMAPPING_OUT_NAME-form\"
  "

echo "=== DONE ==="
