# Workspace environment — source this in EVERY terminal you work in:
#
#   source scripts/env.sh
#
# It replaces the manual export/source dance: strips conda (which otherwise
# poisons CMake builds), loads ROS 2 + the workspace overlay, and exposes the
# XRCE-DDS IDL generator that ArduPilot's build needs.
#
# Bash only (Ubuntu default). Must be sourced, not executed.

if [ -z "${BASH_SOURCE:-}" ]; then
    echo "scripts/env.sh: please source this from bash" >&2
    return 1 2>/dev/null || exit 1
fi

# ── 1. Get conda out of the way ─────────────────────────────────────────────
# An active conda env (even the auto-activated 'base') makes CMake resolve
# libraries from ~/miniconda3 (fmt, spdlog, ...) and breaks the build.
_hydrone_path=
_hydrone_ifs=$IFS; IFS=:
for _hydrone_p in $PATH; do
    case $_hydrone_p in
        *conda*/bin|*conda*/condabin) ;;              # drop conda entries
        *) _hydrone_path=${_hydrone_path:+$_hydrone_path:}$_hydrone_p ;;
    esac
done
IFS=$_hydrone_ifs
export PATH=$_hydrone_path
unset _hydrone_path _hydrone_p _hydrone_ifs
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M 2>/dev/null

# pip --user scripts (MAVProxy, f2py, upgraded pip, ...)
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) export PATH="$HOME/.local/bin:$PATH";; esac

# ── 2. ROS 2 + workspace ────────────────────────────────────────────────────
_HYDRONE_WS=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "scripts/env.sh: ROS 2 Humble not found — run scripts/host_setup.sh first" >&2
fi
[ -f "$_HYDRONE_WS/install/setup.bash" ] && source "$_HYDRONE_WS/install/setup.bash"

# Private DDS domain — must match ROS_DOMAIN_ID in docker-compose.yml so host
# `ros2` commands see the containerized stack and other apps on domain 0 don't
# poison the micro-ROS agent.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# ── 3. XRCE-DDS IDL generator (needed whenever ArduPilot rebuilds) ─────────
export MICROXRCEDDSGEN_DIR="$_HYDRONE_WS/tools/Micro-XRCE-DDS-Gen"
case ":$PATH:" in
    *":$MICROXRCEDDSGEN_DIR/scripts:"*) ;;
    *) export PATH="$MICROXRCEDDSGEN_DIR/scripts:$PATH" ;;
esac

unset _HYDRONE_WS
