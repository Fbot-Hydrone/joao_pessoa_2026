# Workspace environment — source this in EVERY terminal you work in:
#
#   source scripts/env.sh
#
# It replaces the manual export/source dance: strips conda (which otherwise
# poisons CMake builds), loads ROS 2 + the workspace overlay, and exposes the
# XRCE-DDS IDL generator that ArduPilot's build needs.
#
# Works in bash and zsh. Must be sourced, not executed.

# ── 0. Which shell are we in? ───────────────────────────────────────────────
# Needed twice below: to locate this file (bash and zsh spell that
# differently) and to pick ROS's matching setup.<shell>.
if [ -n "${BASH_VERSION:-}" ]; then
    _hydrone_shell=bash
    _hydrone_self=${BASH_SOURCE[0]}
elif [ -n "${ZSH_VERSION:-}" ]; then
    _hydrone_shell=zsh
    # zsh's equivalent of BASH_SOURCE[0]; the parameter-expansion flag is zsh
    # syntax, so it must stay inside this branch — bash would choke on it.
    _hydrone_self=${(%):-%x}
else
    echo "scripts/env.sh: please source this from bash or zsh" >&2
    return 1 2>/dev/null || exit 1
fi

# ── 1. Get conda out of the way ─────────────────────────────────────────────
# An active conda env (even the auto-activated 'base') makes CMake resolve
# libraries from ~/miniconda3 (fmt, spdlog, ...) and breaks the build. It also
# hijacks `python3`: the distrobox shares $HOME, so conda activates in there
# too and rclpy then fails looking for _rclpy_pybind11.cpython-311-*.so while
# /opt/ros/humble is built for 3.10.
#
# Split on ':' by hand rather than `for p in $PATH` with IFS=':' — that relies
# on word splitting, which zsh does not do on unquoted parameters.
_hydrone_path=
_hydrone_rest=$PATH
while [ -n "$_hydrone_rest" ]; do
    _hydrone_p=${_hydrone_rest%%:*}
    case $_hydrone_rest in
        *:*) _hydrone_rest=${_hydrone_rest#*:} ;;
        *)   _hydrone_rest= ;;
    esac
    [ -n "$_hydrone_p" ] || continue
    case $_hydrone_p in
        *conda*/bin|*conda*/condabin) ;;              # drop conda entries
        *) _hydrone_path=${_hydrone_path:+$_hydrone_path:}$_hydrone_p ;;
    esac
done
export PATH=$_hydrone_path
unset _hydrone_path _hydrone_rest _hydrone_p
unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL CONDA_EXE CONDA_PYTHON_EXE _CE_CONDA _CE_M 2>/dev/null

# pip --user scripts (MAVProxy, f2py, upgraded pip, ...)
case ":$PATH:" in *":$HOME/.local/bin:"*) ;; *) export PATH="$HOME/.local/bin:$PATH";; esac

# ── 2. ROS 2 + workspace ────────────────────────────────────────────────────
_HYDRONE_WS=$(cd "$(dirname "$_hydrone_self")/.." && pwd)

if [ -f "/opt/ros/humble/setup.$_hydrone_shell" ]; then
    . "/opt/ros/humble/setup.$_hydrone_shell"
else
    echo "scripts/env.sh: ROS 2 Humble not found — run scripts/host_setup.sh first" >&2
fi
[ -f "$_HYDRONE_WS/install/setup.$_hydrone_shell" ] && . "$_HYDRONE_WS/install/setup.$_hydrone_shell"

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

unset _HYDRONE_WS _hydrone_shell _hydrone_self
