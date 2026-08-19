#!/usr/bin/env bash
# Toggle the host CPU power cap that throttles BiguaSim.
#
#   ./scripts/host_perf.sh status     # what the CPU/GPU are doing right now
#   ./scripts/host_perf.sh on         # unclamp — full clocks, for simulator work
#   ./scripts/host_perf.sh off        # restore the power-saving settings
#
# WHY THIS EXISTS
# The default policy on this laptop (platform_profile=quiet, governor=powersave,
# EPP=power) pins the CPU at ~1.0 GHz of 4.7 GHz and leaves the dGPU downclocked
# and reporting "Idle". Measured 2026-08-19 that cost the simulator 3.1x:
#   quiet/powersave : 17.0 tick/s (0.085x real-time), CPU 1.0 GHz, GPU 1500 MHz 20 W
#   performance     : 52.8 tick/s (0.264x real-time), CPU 4.7 GHz, GPU 2475 MHz 54 W
#
# The governor never ramps on its own here because BiguaSim is a closed lockstep
# loop: slowing the CPU slows the whole loop proportionally, so the busy/blocked
# ratio — which is all intel_pstate measures — stays flat at any clock speed.
# Low CPU% is the SYMPTOM, not spare headroom. See ~/work/biguasim-problems.md.
#
# SAFETY
#   * 'on' snapshots the CURRENT settings to /run before changing anything, and
#     'off' puts exactly those back. Nothing is hardcoded in the normal path.
#   * The snapshot lives in /run (tmpfs), which is wiped on reboot — the same
#     event that resets these knobs. State and reality can't drift apart.
#   * Every value is validated against the kernel's own list of accepted values
#     before it is written. Unknown/absent knobs are skipped, not forced.
#   * Only these three knobs are touched. Turbo, thermal and power limits, and
#     anything GPU-side are left alone (the GPU ramps on its own once fed).
#   * 'status' is read-only and needs no root.
#
# COST OF LEAVING IT ON: the laptop runs hotter and drains faster. It is a
# runtime setting only — a reboot always returns you to the power-saving state,
# so 'on' can never be left on by accident across a reboot.
set -eo pipefail

STATE=/run/hydrone-perf.state

# Fallback used only by 'off' when no snapshot exists (e.g. someone ran 'on'
# before this script did, or /run was cleared mid-session). These are the
# observed factory defaults on this machine — edit if yours differ.
DEFAULT_PROFILE=quiet
DEFAULT_GOVERNOR=powersave
DEFAULT_EPP=power

PROFILE_F=/sys/firmware/acpi/platform_profile
PROFILE_CHOICES_F=/sys/firmware/acpi/platform_profile_choices

red()  { printf '\033[1;31m%s\033[0m\n' "$*"; }
grn()  { printf '\033[1;32m%s\033[0m\n' "$*"; }
ylw()  { printf '\033[1;33m%s\033[0m\n' "$*"; }
bold() { printf '\033[1m%s\033[0m\n' "$*"; }

usage() { sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit "${1:-1}"; }

# ── helpers ────────────────────────────────────────────────────────────────
governors() { echo /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; }
epps()      { echo /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference; }

# Is $1 one of the whitespace-separated values in file $2?
accepts() {
    local want=$1 file=$2
    [[ -r $file ]] || return 1
    local v
    for v in $(<"$file"); do [[ $v == "$want" ]] && return 0; done
    return 1
}

# Write $1 to every file in $2..; tolerate individual failures (offline cores,
# or EPP being read-only while the performance governor holds it).
write_all() {
    local val=$1; shift
    local f ok=0 fail=0
    for f in "$@"; do
        [[ -w $f ]] || { fail=$((fail+1)); continue; }
        if echo "$val" > "$f" 2>/dev/null; then ok=$((ok+1)); else fail=$((fail+1)); fi
    done
    printf '%s %s\n' "$ok" "$fail"
}

# power-profiles-daemon (GNOME) OWNS platform_profile and the intel_pstate EPP.
# Writing those files directly "works" for a few seconds and is then silently
# reverted by the daemon, so when it is running we go through its API instead of
# fighting it. It does NOT manage scaling_governor — that stays ours, and it is
# the knob that actually delivers the speedup.
ppd() { command -v powerprofilesctl >/dev/null 2>&1 && systemctl is-active --quiet power-profiles-daemon; }
ppd_get() { powerprofilesctl get 2>/dev/null || echo ""; }
ppd_has() {
    local want=$1 l
    while read -r l; do
        l=${l#\*}; l=${l//[[:space:]]/}
        [[ $l == "${want}:" ]] && return 0
    done < <(powerprofilesctl list 2>/dev/null)
    return 1
}
# Set a PPD profile. Under sudo, prefer the invoking user's session so polkit
# sees an active local seat; fall back to root, then report failure.
ppd_set() {
    local want=$1
    ppd_has "$want" || return 1
    if [[ -n ${SUDO_USER:-} ]] && sudo -u "$SUDO_USER" powerprofilesctl set "$want" 2>/dev/null; then return 0; fi
    powerprofilesctl set "$want" 2>/dev/null
}

cur_governor() { cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo "n/a"; }
cur_epp()      { cat /sys/devices/system/cpu/cpu0/cpufreq/energy_performance_preference 2>/dev/null || echo "n/a"; }
cur_profile()  { cat "$PROFILE_F" 2>/dev/null || echo "n/a"; }

cpu_mhz() {
    grep '^cpu MHz' /proc/cpuinfo 2>/dev/null | awk '{print $4}' | sort -n |
        awk 'NR==1{min=$1} {a[NR]=$1} END{if(NR)printf "%.0f min / %.0f med / %.0f max", min, a[int((NR+1)/2)], a[NR]}'
}
cpu_max_mhz() {
    local k; k=$(cat /sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq 2>/dev/null) || return
    awk -v k="$k" 'BEGIN{printf "%.0f", k/1000}'
}

on_battery() {
    local p
    for p in /sys/class/power_supply/A*/online /sys/class/power_supply/AC*/online; do
        [[ -r $p ]] && [[ $(<"$p") == 1 ]] && return 1
    done
    # No AC adapter reported as online -> assume battery, but only if a battery exists.
    [[ -e /sys/class/power_supply/BAT0 || -e /sys/class/power_supply/BAT1 ]]
}

need_root() {
    if [[ $EUID -ne 0 ]]; then
        command -v sudo >/dev/null || { red "Needs root and sudo is not installed."; exit 1; }
        exec sudo -- "$0" "$@"
    fi
}

# ── status ─────────────────────────────────────────────────────────────────
do_status() {
    local gov epp prof maxm
    gov=$(cur_governor); epp=$(cur_epp); prof=$(cur_profile); maxm=$(cpu_max_mhz)

    bold "CPU"
    ppd && printf '  power profile    : %s   (power-profiles-daemon)\n' "$(ppd_get)"
    printf '  platform profile : %s\n' "$prof"
    printf '  governor         : %s\n' "$gov"
    printf '  energy pref      : %s\n' "$epp"
    printf '  clock (MHz)      : %s%s\n' "$(cpu_mhz)" \
        "${maxm:+   (rated max ${maxm})}"

    if command -v nvidia-smi >/dev/null 2>&1; then
        bold "GPU"
        nvidia-smi --query-gpu=name,clocks.sm,clocks.max.sm,power.draw,utilization.gpu \
                   --format=csv,noheader 2>/dev/null |
            awk -F', ' '{printf "  %s\n  clock            : %s of %s\n  power / util     : %s / %s\n",$1,$2,$3,$4,$5}'
    fi

    echo
    if [[ $gov == performance ]]; then
        grn "UNCLAMPED (on)"
        [[ -f $STATE ]] || ylw "  note: no snapshot in $STATE — 'off' will fall back to built-in defaults."
    else
        ylw "CLAMPED (off) — the simulator will run roughly 3x slower than it can."
        echo "  run: $0 on"
    fi
}

# ── on ─────────────────────────────────────────────────────────────────────
do_on() {
    need_root on

    if [[ $(cur_governor) == performance && -f $STATE ]]; then
        grn "Already unclamped."; echo; do_status; return
    fi

    if on_battery; then
        ylw "WARNING: running on battery. Full clocks will drain it fast and run hot."
        read -r -p "Continue anyway? [y/N] " ans </dev/tty || ans=n
        [[ $ans == [yY]* ]] || { echo "Aborted."; exit 0; }
    fi

    # Snapshot BEFORE touching anything. Never overwrite an existing snapshot —
    # that would capture the already-modified state and make 'off' a no-op.
    if [[ ! -f $STATE ]]; then
        { echo "PROFILE=$(cur_profile)"
          echo "GOVERNOR=$(cur_governor)"
          echo "EPP=$(cur_epp)"
          ppd && echo "PPD=$(ppd_get)"; } > "$STATE"
        chmod 0644 "$STATE"
        echo "Saved current settings to $STATE"
    else
        echo "Keeping existing snapshot in $STATE"
    fi

    # Power profile first: it can re-arm the firmware's own limits.
    if ppd; then
        if ppd_set performance; then
            echo "  power profile -> $(ppd_get)   (via power-profiles-daemon)"
        else
            # Expected on this laptop: platform_profile lists 'performance' but
            # the ACPI firmware rejects writing it (EIO), so PPD cannot select
            # that profile. Harmless — the governor below is what delivers the
            # speedup, and it is not managed by PPD.
            echo "  power profile    : left at $(ppd_get) (firmware refuses 'performance' — expected, not a problem)"
        fi
    elif [[ -w $PROFILE_F ]]; then
        local want=balanced-performance
        accepts "$want" "$PROFILE_CHOICES_F" || want=performance
        if accepts "$want" "$PROFILE_CHOICES_F"; then
            echo "$want" > "$PROFILE_F" 2>/dev/null &&
                echo "  platform profile -> $(cur_profile)" ||
                ylw "  platform profile: firmware rejected '$want' (left at $(cur_profile))"
        fi
    fi

    # Governor. With intel_pstate in active mode this also pins EPP to
    # 'performance', so EPP is set first and any later refusal is harmless.
    if accepts performance /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_governors; then
        read -r ok fail <<<"$(write_all performance $(governors))"
        echo "  governor -> performance   (${ok} cores$([[ $fail -gt 0 ]] && echo ", ${fail} skipped"))"
    else
        red "  'performance' governor not available — is intel_pstate/acpi-cpufreq loaded?"
    fi

    if accepts performance /sys/devices/system/cpu/cpu0/cpufreq/energy_performance_available_preferences; then
        write_all performance $(epps) >/dev/null
        echo "  energy pref -> $(cur_epp)"
    fi

    echo; grn "Unclamped."; echo; do_status
}

# ── off ────────────────────────────────────────────────────────────────────
do_off() {
    need_root off

    local prof gov epp
    PROFILE=; GOVERNOR=; EPP=; PPD=
    if [[ -f $STATE ]]; then
        # shellcheck disable=SC1090
        source "$STATE" 2>/dev/null || true
        prof=$PROFILE; gov=$GOVERNOR; epp=$EPP
        echo "Restoring the settings saved in $STATE"
    fi

    # A missing, empty or truncated snapshot must not silently leave the machine
    # unclamped: the governor is what actually matters, so if it did not survive
    # the round trip, fall back rather than skipping every write below.
    if [[ -z $gov || $gov == n/a ]]; then
        [[ -f $STATE ]] && ylw "Snapshot in $STATE is unusable (no governor recorded)."
        prof=$DEFAULT_PROFILE; gov=$DEFAULT_GOVERNOR; epp=$DEFAULT_EPP
        ylw "Falling back to this script's defaults"
        ylw "  (profile=$prof governor=$gov epp=$epp; edit the top of $0 if wrong)"
    fi

    # Governor BEFORE EPP: intel_pstate holds EPP read-only while the
    # performance governor is active, so the EPP write would silently fail.
    if [[ -n $gov && $gov != n/a ]] &&
       accepts "$gov" /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_governors; then
        read -r ok fail <<<"$(write_all "$gov" $(governors))"
        echo "  governor -> $gov   (${ok} cores$([[ $fail -gt 0 ]] && echo ", ${fail} skipped"))"
    fi

    if [[ -n $epp && $epp != n/a ]] &&
       accepts "$epp" /sys/devices/system/cpu/cpu0/cpufreq/energy_performance_available_preferences; then
        write_all "$epp" $(epps) >/dev/null
        echo "  energy pref -> $(cur_epp)"
    fi

    if ppd; then
        local want=${PPD:-balanced}
        if ppd_set "$want"; then
            echo "  power profile -> $(ppd_get)   (via power-profiles-daemon)"
        else
            ylw "  power-profiles-daemon refused '$want' (still $(ppd_get))"
        fi
    elif [[ -n $prof && $prof != n/a && -w $PROFILE_F ]] && accepts "$prof" "$PROFILE_CHOICES_F"; then
        echo "$prof" > "$PROFILE_F" 2>/dev/null &&
            echo "  platform profile -> $(cur_profile)" ||
            ylw "  platform profile: firmware rejected '$prof' (left at $(cur_profile))"
    fi

    rm -f "$STATE"
    echo; grn "Restored."; echo; do_status
}

case "${1:-status}" in
    on)              do_on ;;
    off)             do_off ;;
    status|"")       do_status ;;
    -h|--help|help)  usage 0 ;;
    *)               red "Unknown command: $1"; echo; usage 1 ;;
esac
