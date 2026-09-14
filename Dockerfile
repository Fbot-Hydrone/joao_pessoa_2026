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

# OctoMap: the 3-D occupancy map. octomap_server turns a PointCloud2 plus TF
# into occupied/free/unknown by casting a ray to every point — the free/unknown
# part is what a planner needs and what our voxel hash cannot give it. Fed by
# hydrone_map/cloud_filter_node, NOT by the raw cloud (see that node's
# docstring: a flying pixel's ray carves free space through a real wall).
# octomap-rviz-plugins is NOT optional for actually looking at the map: rviz2
# cannot draw an octomap_msgs/Octomap without it, so /octomap_full and
# /octomap_binary are invisible and the only thing left to add is
# /octomap_point_cloud_centers — a plain PointCloud2 of occupied voxel centres
# that looks exactly like the voxel map we already had, which is precisely how
# a real octree gets mistaken for a copy of the old one. The plugin adds the
# OccupancyGrid/OccupancyMap displays that render the tree itself, with its
# probabilities and its free space.
RUN apt-get update && apt-get install -y \
      ros-humble-octomap-server ros-humble-octomap-msgs \
      ros-humble-octomap-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

# rqt_image_view, for `--phase1 --debug` to open the belly camera's annotated
# view next to rviz. hydrone_vision/view_topic.py exists because rqt "pulls in
# most of rqt and Qt for one window" — that objection was written before rviz2
# was in this image, and rviz2 brings the whole of Qt anyway, so the marginal
# cost is now the rqt shell rather than a graphics stack. The script stays: it
# is what runs from the HOST against a real drone, where the container's Qt is
# not in play.
RUN apt-get update && apt-get install -y \
      ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# octomap-python: the ONLY way a Python node can ask the octree a question.
# ROS's own converters (octomap_msgs::binaryMsgToMap) are C++ only, so without
# this /octomap_binary is a byte blob to everything in this stack — a 3-D map
# we build and cannot read. hydrone_map/octree.py wraps it.
RUN pip install --no-cache-dir octomap-python==1.10.0.0

# Placed DOWN HERE, below every apt layer, and that position is deliberate.
# It belongs above `COPY src/` by the rule below, but it must also stay BELOW
# the apt layers: `ros-humble-mavros` and `-extras` have been REMOVED from
# packages.ros.org (2026-09-07 sync leaves only `ros-humble-mavros-msgs`), so
# the MAVROS layer no longer rebuilds. Anything inserted above it invalidates
# its cache and the image stops building — which is exactly what happened when
# this block was first written next to torch. Until that layer is fixed, treat
# everything above as frozen and add pip dependencies here.
# The YOLO backend of the pad detector (hydrone_vision/yolo_pad_detector.py,
# reached with detector_backend:="yolo"). torch is already installed above;
# ultralytics needs torchvision on top of it, because the segmentation head
# runs torchvision.ops.nms on every frame — without it the import succeeds and
# the FIRST inference is what fails.
#
# --no-deps is load-bearing, not tidiness. ultralytics' own requirements pull
# pandas and scipy, and their wheels drag numpy 2.x in, which breaks cv_bridge
# exactly the way the pin above describes (MEASURED here: pandas took numpy to
# 2.2.6 and `import matplotlib` then died with "numpy.core.multiarray failed to
# import"). Inference needs neither pandas nor scipy. The packages on the third
# line are ultralytics' real import-time needs and are installed WITH their own
# dependencies, because none of them touch numpy — installing THOSE --no-deps
# is its own trap: requests without urllib3 fails at `import ultralytics`.
#
# The assert is the regression test. If a future edit reintroduces numpy 2 the
# build stops here, instead of shipping an image whose visual_odometry_node
# segfaults on the first frame in flight.
#
# MEASURED in this image: numpy stays 1.21.5, cv_bridge still round-trips, and
# pad_seg_yolo11 runs in ~17 ms/frame on CPU after a 1.2 s load — unprivileged
# and with the network off.
RUN pip3 install --no-cache-dir --no-deps torchvision==0.22.1 \
        --index-url https://download.pytorch.org/whl/cpu && \
    pip3 install --no-cache-dir --no-deps ultralytics==8.3.40 ultralytics-thop && \
    pip3 install --no-cache-dir pyyaml tqdm psutil py-cpuinfo pillow requests && \
    python3 -c "import numpy, sys; v = numpy.__version__; print('numpy kept at', v); sys.exit(0 if int(v.split('.')[0]) < 2 else 1)" && \
    bash -c ". /opt/ros/humble/setup.sh && python3 -c 'from cv_bridge import CvBridge; CvBridge()' && echo 'cv_bridge still imports'"

# Ultralytics writes a settings file and offers to phone home on first import.
# The entrypoint drops to the unprivileged 'hydrone' user, so point it at a
# world-writable directory outside $HOME (which is partly a read-only bind
# mount) and pre-seed it with sync OFF: a detector node must never block on the
# network while the drone is in the air.
ENV YOLO_CONFIG_DIR=/opt/ultralytics-cfg
RUN mkdir -p $YOLO_CONFIG_DIR && \
    python3 -c "from ultralytics import settings; settings.update({'sync': False})" && \
    chmod -R a+rwX $YOLO_CONFIG_DIR

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
# --base-paths instead of a hand-written --packages-select list: colcon
# discovers every package under src/hydrone_* and src/biguasim-ros2, and a new
# package needs no edit here. src/ardupilot is left out by not being named.
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --base-paths src/hydrone_* src/biguasim-ros2

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "hydrone_bringup", "hydrone_sim.launch.py"]

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'source /ws/install/setup.bash' >> /etc/bash.bashrc
