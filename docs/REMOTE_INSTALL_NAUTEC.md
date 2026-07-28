# Remote install debrief — `nautec` machine (10.228.249.136)

Diagnosis of the problems hit while installing the **refactored**
`refactor/workspace-cleanup` branch of `joao_pessoa_2026` on the lab desktop
`nautec@10.228.249.136` (reached over SSH / AnyDesk), and how to fix them.

**Bottom line:** the workspace was never really broken. It *builds and runs* on
that machine — I reproduced a full, healthy bringup (ArduPilot + micro-ROS +
MAVProxy + BiguaSim UE5) there. Every blocker was **environment/host setup**,
not the code, and none of them are called out in the README. The most damaging
one is a *misleading error message* that makes a "no display" problem look like
a "root privileges" problem.

---

## 1. The machine as found

| Item | State |
|---|---|
| OS | Ubuntu 22.04.5 LTS ✓ |
| ROS 2 | Humble installed ✓ |
| Repo | `~/Documents/joao_pessoa_2026`, branch `refactor/workspace-cleanup` at `3e5b348d` ✓ |
| Sim repo | `~/Documents/bs-drone-competition` (not the `../bs-competition/...` layout the scripts try first) |
| Deps | `src/ardupilot`, `micro_ros_agent`, `micro_ros_msgs`, `tools/Micro-XRCE-DDS-Gen` all imported ✓ |
| Build | `colcon build` **succeeded** — `install/` fully populated, last build exit 0 ✓ |
| Python | `torch==2.7.1+cpu` ✓, `biguasim` importable ✓ |
| GPU | NVIDIA RTX 4070 Ti, driver 580, Vulkan ICD present ✓ — **no usable iGPU** (dGPU-only desktop) |
| World assets | `~/.local/share/biguasim/1.0.0/worlds/Competition` fully downloaded (1.4 GB, Holodeck binary + pak) ✓ |
| Docker | Engine 29.6 + compose v5.3 + `nvidia-container-toolkit` 1.19.1 (CDI) now installed ✓; image `joao_pessoa_2026-hydrone` built ✓ |
| conda | miniconda **base auto-activates** in every interactive shell (`.bashrc` conda-init block) ⚠ |

The reason "it builds fine on my machine but not there" is that most of the
above (Docker, the NVIDIA toolkit, conda, the sim-repo path) are *host state*
the README assumes is already correct.

---

## 2. What actually happened (reconstructed from `~/.bash_history`)

The install did **not** follow either README path cleanly. Instead of
`scripts/host_setup.sh`, the commands were run by hand and then the Docker path
was attempted:

```
# manual host attempt
sudo apt install python3-vcstool python3-pip default-jdk ros-humble-cv-bridge ros-humble-image-transport libzbar0
pip3 install empy==3.3.4 pexpect pymavlink dronecan future lxml MAVProxy mediapipe pyzbar opencv-python numpy   # ⚠ no --user, no torch-cpu, no roma/matplotlib
(cd ../bs-drone-competition && pip3 install -e .)
vcs import --recursive . < deps.repos
(cd tools/Micro-XRCE-DDS-Gen && ./gradlew assemble -x submodulesUpdate)
export MICROXRCEDDSGEN_DIR=$PWD/tools/Micro-XRCE-DDS-Gen ; export PATH=$MICROXRCEDDSGEN_DIR/scripts:$PATH
colcon build --symlink-install
source install/setup.bash
# then the Docker path
./scripts/docker_up.sh          # failed: docker not installed
sudo apt install docker         # ⚠ WRONG package (see 3.3)
./scripts/docker_up.sh          # still failing at that point
```

Earlier history (pre-refactor) shows the same person fighting the two most
common traps over and over: repeatedly re-running `ros2 launch ...` that did
nothing, and repeatedly typing `conda deactivate`.

---

## 3. Problems, root causes, and fixes

### 3.1 — The confusing one: "Timed out waiting for binary to load … root privileges"  ★ most important

Running the sim over a plain SSH shell (or any session without a GPU display)
dies with:

```
posix_ipc.BusyError: Semaphore is busy
biguasim.exceptions.BiguaSimException: Timed out waiting for binary to load.
Ensure that biguasim is not being run with root priveleges.
```

**This message is misleading.** It has nothing to do with root. The BiguaSim
UE5 binary (`Holodeck`) is launched, tries to open a window on `$DISPLAY`,
finds none (or no GPU), never signals "loaded", and the 30 s load semaphore
times out — which `posix_ipc` reports as `BusyError`.

- Confirmed: with **no** `DISPLAY` (SSH) → this exact failure.
- Confirmed: with `DISPLAY=:1` + `XAUTHORITY` pointed at the live GDM session →
  the binary loads and the **whole stack comes up healthy**:
  `ArduPilot connected / online`, 7 sensor publishers, MAVProxy heartbeat
  `AP: ArduCopter V4.8.0-dev`.

**Fix / how to actually run it:** launch from a session that has the GPU
desktop — i.e. a terminal **inside the AnyDesk/physical desktop**, not a bare
`ssh` shell. To drive it from SSH anyway, export the local display first:

```bash
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority   # path of the logged-in GDM session
```

This is the single most likely thing the user "saw that wasn't in the README":
the build works, then the run fails with a message that sends you down the
wrong (root/permissions) rabbit hole.

### 3.2 — `ros2: command not found` / `ros2 launch` "does nothing"

A fresh shell has neither ROS nor the workspace overlay sourced, so `ros2 launch
hydrone_bringup ...` fails or can't find the package. The history shows this hit
repeatedly before `source /opt/ros/humble/setup.bash` + `source
install/setup.bash` were run.

**Fix:** the refactor already provides the one-liner — use it in *every*
terminal:

```bash
source scripts/env.sh
```

(It sources ROS + the overlay, exposes the XRCE-DDS generator, and scrubs
conda.) The README mentions it, but doesn't warn that skipping it looks like
"ros2 is broken." On this machine only `source /opt/ros/humble/setup.bash` was
appended to `.bashrc`; the workspace overlay still has to be sourced per build.

### 3.3 — Docker path: `sudo apt install docker` installs the wrong package

On Ubuntu 22.04 the `docker` apt package is **not** Docker Engine (it's an
unrelated system-tray applet). So `docker_up.sh` kept failing even after the
"install". The machine now has the *correct* Engine (29.6, from Docker's
official repo) + the compose plugin, so the path works — but the README never
says how to install Docker.

**Fix:** README "Option 1" should link the official install and require the
compose plugin, e.g.:

```bash
# Docker Engine + compose plugin (official repo) — see docs.docker.com/engine/install/ubuntu
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo usermod -aG docker "$USER"   # then log out/in
```

`docker-compose` (the old standalone) is **not** installed here — the scripts
correctly use `docker compose` (plugin), so that's fine, just worth stating.

### 3.4 — NVIDIA-only desktop: the iGPU fallback can't render UE5

The README frames NVIDIA as optional ("by default renders on the integrated
GPU"). This box has **no usable iGPU** — it's a dGPU-only desktop. Without
`nvidia-container-toolkit`, the container's `/dev/dri` mesa path can't run UE5
and you get the same 3.1 timeout *inside Docker*. It's now installed and
`docker info` exposes both the `nvidia` runtime and CDI (`nvidia.com/gpu=all`),
so `scripts/docker_up.sh` auto-detects it ("NVIDIA container runtime detected —
rendering on the dGPU") and the container starts correctly.

**Fix / note:** for any NVIDIA-only machine the toolkit is **required**, not
optional:

```bash
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Fragility worth fixing in the script: `docker_up.sh` decides via
`docker info | grep -qi 'runtimes:.*nvidia'`. With newer toolkits that register
**only** CDI (no named `nvidia` runtime) that grep misses and it silently falls
back to the iGPU path → the confusing 3.1 timeout again. Consider also detecting
CDI (`nvidia-ctk cdi list` / `grep -q 'cdi: nvidia'`).

### 3.5 — conda `base` auto-activates and poisons builds

`.bashrc` has a conda-init block, so every interactive shell starts in `base`.
Per the README's own gotcha, that makes CMake resolve `fmt`/`spdlog` from
miniconda and breaks the micro-ROS agent build. The history shows `conda
deactivate` typed many times. `scripts/env.sh` scrubs conda from `PATH`, but
only if you actually source it — a manual `colcon build` in a conda shell does
not get that protection.

**Fix:** always build/run through `source scripts/env.sh` (it removes conda),
or `conda config --set auto_activate_base false` once on this machine.

### 3.6 — Sim-repo path assumption

Scripts probe `../bs-competition/bs-drone-competition` then
`../bs-drone-competition`. Here the repo is `~/Documents/bs-drone-competition`,
which the **second** probe happens to catch (they're siblings under
`Documents/`), so it works — but only by luck of layout. If it's ever elsewhere,
set `BS_SIM_DIR` explicitly:

```bash
BS_SIM_DIR=~/Documents/bs-drone-competition ./scripts/docker_up.sh
```

### 3.7 — Process hygiene (git)

Not a build blocker, but the history shows local machine config being committed
onto shared branches (`pc_nautec`, a `"testing"` commit, and a
`develop_bridge_ros2 → main` merge pushed with `user.email` set to a malformed
address). Worth cleaning up so per-machine state doesn't leak into the repo.

---

## 4. How to run it on this machine today (both paths verified)

**Host path** — from a terminal *on the AnyDesk/physical desktop* (has the GPU
display):

```bash
cd ~/Documents/joao_pessoa_2026
source scripts/env.sh
ros2 launch hydrone_bringup hydrone_sim.launch.py           # sim + SITL + agent + MAVProxy
# second terminal (also: source scripts/env.sh)
ros2 launch hydrone_bringup hydrone.launch.py phase:=1      # autonomy stack
```

**Docker path** (NVIDIA auto-detected, image already built):

```bash
cd ~/Documents/joao_pessoa_2026
./scripts/docker_up.sh
```

From a bare SSH shell, prepend the display exports from 3.1 or you'll hit the
misleading "root privileges" timeout.

---

## 5. Suggested README / script changes

1. **Add a "Troubleshooting" box** decoding *"Timed out waiting for binary to
   load … root privileges"* → really means *no GPU display*; run from the
   desktop session or export `DISPLAY`/`XAUTHORITY` (3.1).
2. **README Option 1**: add the official Docker Engine + compose-plugin install
   line and the `usermod -aG docker` step; state that plain `apt install docker`
   is the wrong package (3.3).
3. **Promote the NVIDIA toolkit from "optional" to "required on dGPU-only
   machines"**, and harden `docker_up.sh` to also detect CDI, not just a named
   `nvidia` runtime (3.4).
4. **Call out that skipping `source scripts/env.sh` looks like "ros2 is
   broken"** and that the overlay must be re-sourced after every build (3.2).
5. **Recommend `conda config --set auto_activate_base false`** on lab machines
   in the conda gotcha (3.5).

---

*Diagnosis performed over SSH on 2026-07-07. Both bringup paths were reproduced
end-to-end on the machine; all stray sim/SITL processes and test containers were
cleaned up afterward.*
