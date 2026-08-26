FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# Toolchain for ArduPilot SITL, the micro-ROS agent and the DDS IDL generator
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-dev \
    python3-vcstool \
    build-essential \
    cmake \
    pkg-config \
    default-jdk \
    libzbar0 \
    && rm -rf /var/lib/apt/lists/*

# ArduPilot build deps + MAVProxy (sitl_dds_udp.launch.py also starts MAVProxy)
RUN pip3 install --no-cache-dir \
    empy==3.3.4 pexpect pymavlink dronecan future lxml MAVProxy

# Hydrone stack runtime deps.
#
# numpy is PINNED BELOW 2.0 on purpose. ROS Humble's cv_bridge ships a compiled
# boost extension linked against numpy 1.x; installing numpy 2.x over it does
# not fail at install time, and `import cv_bridge` only prints
#   AttributeError: _ARRAY_API not found
# before carrying on — then the first imgmsg_to_cv2() call SEGFAULTS the node.
# That takes down visual_odometry_node, which with GPS disabled is what feeds
# the EKF its position. Do not relax this pin without checking cv_bridge again:
#   python3 -c 'from cv_bridge import CvBridge; CvBridge()'
# The landing-pad nodes sidestep cv_bridge entirely (hydrone_vision/
# image_convert.py), but vision_node and visual_odometry_node still use it.
RUN pip3 install --no-cache-dir mediapipe pyzbar opencv-python "numpy<2"

# Runtime deps of biguasim that its setup.py doesn't declare (its code
# imports torch/roma/matplotlib). CPU-only torch: the CUDA wheels add
# multiple GB and UE5 does the physics; swap the index-url if you need GPU torch.
RUN pip3 install --no-cache-dir torch==2.7.1 --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --no-cache-dir roma==1.5.3 matplotlib

# UE5 runtime: Vulkan loader + mesa drivers, and an unprivileged user —
# Unreal refuses to start as root, so the entrypoint drops to this user.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libvulkan1 mesa-vulkan-drivers vulkan-tools \
    && rm -rf /var/lib/apt/lists/* \
    && useradd -m -s /bin/bash -u 1000 hydrone

# MAVROS: bridges ArduPilot MAVLink <-> /mavros/* topics that the autonomy
# controller_node speaks (arming, mode, setpoints). Also provides mavros_msgs,
# a runtime import of controller_node. install_geographiclib_datasets pulls the
# geoid the global-position plugin needs (non-fatal if it can't download).
RUN apt-get update && apt-get install -y \
      ros-humble-mavros ros-humble-mavros-msgs ros-humble-mavros-extras \
      geographiclib-tools \
    && (geographiclib-get-geoids egm96-5 || true) \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────────────────────────
# LAYER ORDER MATTERS BELOW THIS LINE.
#
# Everything above is environment: ~1 GB of pip/apt that depends on nothing in
# this repo. It is deliberately placed ahead of `COPY src/` so that editing a
# node does NOT invalidate it — Docker rebuilds every layer below a changed
# one, and torch alone is an 800 MB re-download.
#
# Keep the project `COPY src/` + colcon build LAST. If you add a new pip/apt
# dependency, put it above this line, not below.
# ─────────────────────────────────────────────────────────────────────────────

WORKDIR /ws

# 1. Pinned source dependencies (this layer re-runs only when deps.repos changes)
COPY deps.repos ./
RUN vcs import --recursive . < deps.repos

# 2. IDL generator required by ArduPilot's DDS build
ENV MICROXRCEDDSGEN_DIR=/ws/tools/Micro-XRCE-DDS-Gen
RUN cd "$MICROXRCEDDSGEN_DIR" && ./gradlew assemble -x submodulesUpdate
ENV PATH="$MICROXRCEDDSGEN_DIR/scripts:$PATH"

# 3. Third-party ROS packages (ArduPilot SITL is the slow one — cached
#    independently of project code changes). Sequential executor: building
#    micro_ros_agent and ardupilot_sitl in parallel starves the AP_DDS IDL
#    generator's JVM, which dies with exit 255.
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --executor sequential \
      --packages-up-to ardupilot_sitl ardupilot_msgs micro_ros_agent

# 4. The biguasim Python package (simulator client) is installed at container
#    start from the mounted bs-drone-competition repo — see docker/entrypoint.sh.
#    Above `COPY src/` so editing a node doesn't re-run it (and vice versa: the
#    entrypoint changes rarely, and then only step 5 replays).
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 5. Project packages — LAST, so a source edit replays only this build.
#    `--symlink-install` chains install/ -> build/ -> src/, which is what makes
#    docker-compose.dev.yml's bind mounts live without any rebuild at all.
COPY src/ src/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
      --packages-select hydrone_msgs biguasim_interfaces biguasim_main \
        hydrone_bringup hydrone_vision hydrone_controller hydrone_nav \
        hydrone_map hydrone_mission

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "hydrone_bringup", "hydrone_sim.launch.py"]

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'source /ws/install/setup.bash' >> /etc/bash.bashrc
