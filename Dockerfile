FROM ubuntu:20.04

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ── Base tools ────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    build-essential \
    git \
    apt-transport-https \
    ca-certificates \
    wget \
    libeigen3-dev \
    libtbb-dev \
    libpcl-dev \
    nlohmann-json3-dev \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# ── CMake >= 3.24 (required for FetchContent FIND_PACKAGE_ARGS used by FORM) ──
# Ubuntu 20.04 ships CMake 3.16 which is too old.
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
    | gpg --dearmor - > /usr/share/keyrings/kitware-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main" \
    > /etc/apt/sources.list.d/kitware.list && \
    apt-get update && apt-get install -y --no-install-recommends cmake \
    && rm -rf /var/lib/apt/lists/*

# ── GTSAM 4.2 from borglab PPA ───────────────────────────────────────────────
# ABI-compatible with Ubuntu 20.04 Eigen3 and TBB packages installed above.
RUN add-apt-repository ppa:borglab/gtsam-release-4.2 && \
    apt-get update && apt-get install -y --no-install-recommends \
    libgtsam-dev \
    libgtsam-unstable-dev \
    && rm -rf /var/lib/apt/lists/*

# ── ROS Noetic ────────────────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros1.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-catkin-tools \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# ── Build FORM C++ library ────────────────────────────────────────────────────
# FORM has no CMake install() target, so we build in-source and reference the
# build directory directly from the catkin packages.
# Note: tsl::robin_map is fetched via CMake FetchContent — internet access
# is required during docker build.
WORKDIR /opt/form-src
COPY ./src/form .

RUN cmake -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DFORM_BUILD_TESTS=OFF \
      -DFORM_BUILD_PYTHON=OFF \
    && cmake --build build --parallel $(nproc)

# ── Build catkin workspace ────────────────────────────────────────────────────
WORKDIR /ros_ws

COPY ./src/form-ros-node     ./src/form-ros-node
COPY ./src/form-to-hdmapping ./src/form-to-hdmapping

# CMAKE_POLICY_VERSION_MINIMUM is needed because the new CMake (3.30+) rejects
# catkin's toplevel.cmake which uses cmake_minimum_required(VERSION 2.8.12).
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# ── Non-root user ─────────────────────────────────────────────────────────────
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID -s /bin/bash ros

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /ros_ws/devel/setup.bash"   >> /root/.bashrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/ros/.bashrc && \
    echo "source /ros_ws/devel/setup.bash"   >> /home/ros/.bashrc

CMD ["bash"]
