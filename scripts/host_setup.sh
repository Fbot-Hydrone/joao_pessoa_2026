#!/usr/bin/env bash
# One-shot host setup: everything README "Option 2" needs, in the right order.
# Safe to re-run — every step is idempotent.
#
#   ./scripts/host_setup.sh
#
# Needs: Ubuntu 22.04, sudo. Installs ROS 2 Humble itself if missing.
# After it finishes, work in any terminal with:  source scripts/env.sh
set -eo pipefail
cd "$(dirname "$0")/.."
WS=$PWD

step() { printf '\n\033[1;36m── %s\033[0m\n' "$*"; }

# ── 0. Sanity: Ubuntu 22.04, no conda in the environment ───────────────────
if ! grep -q 'VERSION_ID="22.04"' /etc/os-release 2>/dev/null; then
    echo "WARNING: this workspace targets Ubuntu 22.04 (ROS 2 Humble)." >&2
    echo "         On other systems prefer the Docker path (scripts/docker_up.sh)." >&2
fi

# Conda (even the auto-activated 'base') poisons CMake and pip. Instead of
# asking users to deactivate, scrub it from this script's own environment.
source scripts/env.sh 2>/dev/null || true   # reuses the PATH-scrub logic
if command -v python3 >/dev/null && [[ $(command -v python3) == *conda* ]]; then
    echo "ERROR: python3 still resolves into conda ($(command -v python3))." >&2
    echo "       Run 'conda deactivate' (repeat until no env is active) and retry." >&2
    exit 1
fi

# ── 1. ROS 2 Humble ─────────────────────────────────────────────────────────
if [ ! -f /opt/ros/humble/setup.bash ]; then
    step "Installing ROS 2 Humble (missing)"
    sudo apt-get update
    sudo apt-get install -y curl gnupg lsb-release software-properties-common
    sudo add-apt-repository -y universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y ros-humble-desktop
else
    step "ROS 2 Humble already installed"
fi

# ── 2. System packages ──────────────────────────────────────────────────────
step "Checking apt dependencies"
apt_pkgs=(git build-essential cmake pkg-config python3-dev python3-pip
          python3-vcstool python3-colcon-common-extensions default-jdk
          ros-humble-cv-bridge ros-humble-image-transport libzbar0)
missing=()
for p in "${apt_pkgs[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
done
if [ ${#missing[@]} -gt 0 ]; then
    echo "Installing: ${missing[*]}"
    sudo apt-get update
    sudo apt-get install -y "${missing[@]}"
else
    echo "All present — skipping"
fi

# ── 3. Python layer (system pip, --user; NEVER conda) ──────────────────────
step "Installing Python dependencies (user site)"
# Stock pip 22 / setuptools 59 can't do 'pip install -e' with a pyproject
# (falls back to setup.py develop, which ignores --user) — upgrade first.
/usr/bin/python3 -m pip install --user -U pip setuptools wheel
python3 -m pip install --user \
    empy==3.3.4 pexpect pymavlink dronecan future lxml MAVProxy \
    mediapipe pyzbar opencv-python numpy
# biguasim needs torch; install the CPU wheel explicitly or pip pulls the
# multi-GB CUDA build as a dependency.
python3 -m pip install --user torch==2.7.1 --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install --user roma matplotlib

# ── 4. BiguaSim simulator repo ──────────────────────────────────────────────
step "Locating BiguaSim repo"
if [ -z "${BS_SIM_DIR:-}" ]; then
    for candidate in ../bs-competition/bs-drone-competition ../bs-drone-competition; do
        [ -d "$candidate" ] && BS_SIM_DIR=$candidate && break
    done
fi
if [ -z "${BS_SIM_DIR:-}" ]; then
    BS_SIM_DIR=../bs-drone-competition
    echo "Not found — cloning next to this repo: $BS_SIM_DIR"
    git clone https://github.com/HenriqueReichow/bs-drone-competition.git "$BS_SIM_DIR"
fi
BS_SIM_DIR=$(cd "$BS_SIM_DIR" && pwd)
echo "BiguaSim repo: $BS_SIM_DIR"
python3 -m pip install --user -e "$BS_SIM_DIR"

# ── 5. Pinned source dependencies (ArduPilot, micro-ROS, XRCE-Gen) ─────────
step "Importing pinned source dependencies"
if [ -d src/ardupilot/.git ] && [ -d src/micro_ros_agent/.git ] \
   && [ -d src/micro_ros_msgs/.git ] && [ -d tools/Micro-XRCE-DDS-Gen/.git ]; then
    echo "Already imported — skipping (delete the dirs to re-import)"
else
    vcs import --recursive . < deps.repos
fi

# ── 6. XRCE-DDS IDL generator (ArduPilot's build calls it) ─────────────────
step "Building Micro-XRCE-DDS-Gen"
( cd tools/Micro-XRCE-DDS-Gen && ./gradlew assemble -x submodulesUpdate )

# ── 7. Build the workspace ──────────────────────────────────────────────────
# Third-party first, sequentially: a parallel ardupilot+agent build can starve
# the IDL generator's JVM (exit 255). Then everything else in parallel.
step "Building third-party packages (sequential — the slow part, ~15 min)"
source scripts/env.sh
colcon build --symlink-install --executor sequential \
    --packages-up-to ardupilot_sitl ardupilot_msgs micro_ros_agent

step "Building project packages"
colcon build --symlink-install

step "Done"
echo
echo "In every terminal:   source scripts/env.sh"
echo "Run the simulation:  ros2 launch hydrone_bringup hydrone_sim.launch.py"
echo "Autonomy stack:      ros2 launch hydrone_bringup hydrone.launch.py phase:=1"
