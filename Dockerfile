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
    cmake \
    libeigen3-dev \
    libtbb-dev \
    libpcl-dev \
    nlohmann-json3-dev \
    tmux \
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

RUN source /opt/ros/noetic/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release

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
