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

# Hydrone stack runtime deps
RUN pip3 install --no-cache-dir mediapipe pyzbar opencv-python numpy

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

# 4. Project packages
COPY src/ src/
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install \
      --packages-select hydrone_msgs biguasim_interfaces biguasim_main \
        hydrone_bringup hydrone_vision hydrone_controller hydrone_nav \
        hydrone_mission

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

# The biguasim Python package (simulator client) is installed at container
# start from the mounted bs-drone-competition repo — see docker/entrypoint.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "hydrone_bringup", "hydrone_sim.launch.py"]
