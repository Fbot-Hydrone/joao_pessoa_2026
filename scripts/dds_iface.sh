# Pin DDS to one network interface. Sourced, not executed:
#
#   . "$(dirname "$0")/dds_iface.sh"
#   dds_iface_setup auto         # -> DDS_MODE, DDS_ADDR, DDS_PROFILE, DDS_IFACE
#   dds_iface_report             # -> one line saying what it picked
#
# WHY THIS EXISTS
# The workstation and the Jetson are on TWO networks at once: the shared wifi
# (192.168.0.0/24) and a direct cable (10.10.0.0/24). Nothing in ROS picks
# between them. Fast DDS announces a locator for every interface it can see and
# then uses whichever the peer answers on, so an image stream that could have
# had a dedicated gigabit link happily goes out over the wifi the drone is also
# flying on -- 640x480 BGR at 15 Hz is ~110 Mbit/s, competing with MAVLink.
#
# The knob is Fast DDS's interfaceWhiteList: restrict the UDPv4 transport to a
# single local address and both the announced locators and the sockets follow.
# This image only ships rmw_fastrtps_cpp (there is no CycloneDDS in
# /opt/ros/humble/lib), so the XML below is the only mechanism available.
#
# WHY THE SHM TRANSPORT IS DECLARED EXPLICITLY
# Setting <useBuiltinTransports>false</useBuiltinTransports> is what stops Fast
# DDS from ALSO adding its default, unrestricted UDPv4 -- without it the
# whitelist is decorative. But that same flag drops shared memory, which is how
# nodes inside one container talk to each other. On the Jetson that is the ZED
# feeding the detectors: forcing them through the loopback UDP stack instead of
# SHM is a real cost on a Tegra X1. So SHM is added back by hand, and only the
# NETWORK path ends up pinned.
#
# WHY THE DEFAULT IS `auto` AND NOT `cable`
# Which link to use is not a preference, it is a fact about the hardware:
# either the direct cable is plugged in and carrying a link, or it is not.
# Asking a human to restate that fact on every command -- on BOTH machines, in
# agreement -- is asking them to get it wrong, and the way it goes wrong is
# silent (see _dds_cable_is_live). `auto` measures it instead, on a signal both
# ends read identically. `cable` and `wifi` remain, and still fail loudly, for
# when you want to assert rather than detect.
#
# Overridable by environment: CABLE_SUBNET, CABLE_IFACE, WIFI_IFACE, CABLE_PEER.

# Bare IPv4 of one interface, or nothing if it has no address.
_dds_addr_of() {
    ip -4 -br addr show "$1" 2>/dev/null | awk 'NR==1 {print $3}' | cut -d/ -f1
}

# First interface whose name starts with one of the given prefixes AND that
# actually carries a usable IPv4. Echoes "<iface> <addr>".
#
# Prefixes and not fixed names because this file is sourced on both machines:
# the workstation calls the cable enp63s0 and the wifi wlp62s0, the Jetson
# calls them eth0 and wlan0, and a different laptop will use something else
# again. Docker's bridges are skipped explicitly -- docker0 is 172.17.0.1 on
# BOTH machines, which is exactly the address that sends people chasing a
# connection to their own laptop.
_dds_find_iface() {
    local prefix name state addr rest
    for prefix in "$@"; do
        while read -r name state addr rest; do
            [ -n "$addr" ] || continue
            case "$name" in
                "$prefix"*)          ;;
                *)                   continue ;;
            esac
            case "$name" in
                docker*|veth*|br-*|virbr*|tailscale*) continue ;;
            esac
            addr=${addr%%/*}
            case "$addr" in
                127.*|172.17.*|169.254.*) continue ;;
            esac
            printf '%s %s\n' "$name" "$addr"
            return 0
        done < <(ip -4 -br addr show 2>/dev/null)
    done
    return 1
}

# First interface whose IPv4 falls inside a given subnet prefix, e.g. "10.10.0.".
# Echoes "<iface> <addr>".
#
# This is the PRIMARY way the cable is found, because that link is statically
# configured on both ends and therefore genuinely fixed: the workstation is
# 10.10.0.1 and the Jetson 10.10.0.2, set with NetworkManager ipv4.method=manual
# and autoconnect, so they survive reboots and replugs. Matching the subnet is
# exact where matching a name prefix ("en*") is a guess that a second wired
# interface would win by accident.
_dds_find_in_subnet() {
    local want="$1" name state addr rest
    while read -r name state addr rest; do
        [ -n "$addr" ] || continue
        addr=${addr%%/*}
        case "$addr" in
            "$want"*) printf '%s %s\n' "$name" "$addr"; return 0 ;;
        esac
    done < <(ip -4 -br addr show 2>/dev/null)
    return 1
}

# Is the direct cable actually a live link? $1 is the interface, $2 our
# address on it.
#
# An address on 10.10.0.x proves the link is CONFIGURED, not that it exists:
# NetworkManager will happily hold a static address on an interface whose
# carrier is gone. That distinction is the whole reason `auto` can be trusted,
# because picking the cable while the OTHER machine quietly picked the wifi is
# the one failure nothing downstream can detect -- no error, an empty rviz,
# indistinguishable from a domain-id mismatch.
#
# THE TEST IS CARRIER, NOT REACHABILITY, and that choice matters:
#
#   * Carrier is a property of the CABLE, so both ends read the same value at
#     any time. Reachability is a property of the OTHER HOST, and the two ends
#     do not observe it at the same moment: the Jetson starts on the drone,
#     often minutes before anyone opens rviz. A Jetson that pinged a
#     not-yet-booted workstation would settle on the wifi for the whole flight,
#     and the workstation would then pick the cable and see nothing. Deciding
#     on the cable itself removes that race.
#   * A silent peer is not a dead link. A workstation that is off, or has a
#     default-deny firewall, does not answer ICMP -- and downgrading on that
#     would put a 110 Mbit/s image stream on the link the drone is flying
#     MAVLink on, which is the exact accident this file exists to prevent.
#
# So ping only ever prints a hint, and never changes the answer. Likewise an
# unreadable carrier ("unknown") keeps the cable: the cost of wrongly choosing
# the cable is a viewer that shows nothing, which you SEE; the cost of wrongly
# choosing the wifi is a starved flight link, which you do not.
_dds_cable_is_live() {
    local iface="$1" mine="$2" carrier peer rc=0
    # $(cat), not `read < file`: a redirection that fails prints its own error
    # BEFORE the 2>/dev/null on the same command takes effect, so an interface
    # that has gone away would spray a shell error over the console.
    carrier=$(cat "/sys/class/net/$iface/carrier" 2>/dev/null || true)
    if [ "$carrier" = 0 ]; then
        echo "NOTE: $iface has $mine but no carrier -- the cable is unplugged" >&2
        echo "      (or dead) and NetworkManager is still holding the static" >&2
        echo "      address. Using the wifi." >&2
        return 1
    fi

    # Advisory only. The peer is the other of .1/.2 on the link (workstation
    # and Jetson), a guess this file is entitled to make because it also
    # chooses the subnet. CABLE_PEER overrides it; CABLE_PEER= (set but empty)
    # skips the hint. Only status 1 -- "sent, nothing came back" -- is worth
    # mentioning; ping exits 2 for no permission or no such host.
    if [ -n "${CABLE_PEER+x}" ]; then
        peer="$CABLE_PEER"
    else
        case "$mine" in
            *.1) peer="${mine%.1}.2" ;;
            *.2) peer="${mine%.2}.1" ;;
            *)   peer="" ;;
        esac
    fi
    if [ -n "$peer" ] && command -v ping >/dev/null 2>&1; then
        ping -c1 -W1 "$peer" >/dev/null 2>&1 || rc=$?
        [ "$rc" -eq 1 ] && echo "note: cable is up, but $peer is not answering yet." >&2
    fi
    return 0
}

# What `auto` resolves to: cable|wifi|any on stdout, reasons on stderr.
#
# The cable is found by SUBNET only, never by the enp*/eth* name fallback that
# explicit --cable allows: on the workstation that fallback would cheerfully
# pick the office LAN and pin everything to an interface the drone is not on.
# An assertion may guess; a measurement may not.
_dds_auto_mode() {
    local found
    if found=$(_dds_find_in_subnet "${CABLE_SUBNET:-10.10.0.}"); then
        if _dds_cable_is_live "${found%% *}" "${found##* }"; then
            echo cable
            return 0
        fi
    fi
    if _dds_find_iface ${WIFI_IFACE:-} wlp wlan wl >/dev/null; then
        echo wifi
        return 0
    fi
    echo "NOTE: no cable on ${CABLE_SUBNET:-10.10.0.}x and no wireless" >&2
    echo "      interface with an address. Leaving DDS unpinned." >&2
    echo any
}

# Write the Fast DDS profile that pins UDPv4 to $1, into file $2.
_dds_write_profile() {
    local addr="$1" out="$2"
    cat > "$out" <<XML
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
  <profiles>
    <transport_descriptors>
      <transport_descriptor>
        <transport_id>pinned_udp</transport_id>
        <type>UDPv4</type>
        <interfaceWhiteList>
          <address>${addr}</address>
        </interfaceWhiteList>
      </transport_descriptor>
      <transport_descriptor>
        <transport_id>local_shm</transport_id>
        <type>SHM</type>
      </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="pinned" is_default_profile="true">
      <rtps>
        <userTransports>
          <transport_id>local_shm</transport_id>
          <transport_id>pinned_udp</transport_id>
        </userTransports>
        <useBuiltinTransports>false</useBuiltinTransports>
      </rtps>
    </participant>
  </profiles>
</dds>
XML
}

# dds_iface_setup <auto|cable|wifi|any>
#
# Sets, for the caller to use:
#   DDS_REQUESTED  the mode as given
#   DDS_MODE       what it RESOLVED to; auto becomes cable, wifi or any
#   DDS_IFACE      interface chosen ("" for any)
#   DDS_ADDR       its IPv4    ("" for any)
#   DDS_PROFILE    path to the generated XML ("" for any)
dds_iface_setup() {
    DDS_REQUESTED="$1"
    DDS_MODE="$1"
    DDS_IFACE=""
    DDS_ADDR=""
    DDS_PROFILE=""

    # Resolve first, then fall into the same branches an explicit mode takes,
    # so auto cannot drift away from what --cable/--wifi actually do.
    [ "$DDS_MODE" = auto ] && DDS_MODE=$(_dds_auto_mode)

    case "$DDS_MODE" in
        any)
            # No profile at all: whatever DDS negotiates, the behaviour this
            # project had before the cable existed.
            return 0
            ;;
        cable)
            # Subnet first (exact, and the link is static), interface names only
            # as a fallback for a rebuilt link that has not been renumbered yet.
            local found
            if found=$(_dds_find_in_subnet "${CABLE_SUBNET:-10.10.0.}"); then
                :
            elif found=$(_dds_find_iface ${CABLE_IFACE:-} enp eth en); then
                echo "NOTE: nothing on ${CABLE_SUBNET:-10.10.0.}x; falling back to" >&2
                echo "      ${found%% *} at ${found##* }. Is this the direct cable?" >&2
            else
                echo "ERROR: --cable, but no wired interface has an IPv4 address." >&2
                echo "       Looked for ${CABLE_SUBNET:-10.10.0.}x, then" >&2
                echo "       ${CABLE_IFACE:+$CABLE_IFACE, }enp*, eth*, en*" >&2
                echo "       Is the cable plugged in? Otherwise use --wifi." >&2
                return 1
            fi
            DDS_IFACE=${found%% *}
            DDS_ADDR=${found##* }
            ;;
        wifi)
            local found
            if found=$(_dds_find_iface ${WIFI_IFACE:-} wlp wlan wl); then
                DDS_IFACE=${found%% *}
                DDS_ADDR=${found##* }
            else
                echo "ERROR: --wifi, but no wireless interface has an IPv4 address." >&2
                echo "       Tried: ${WIFI_IFACE:+$WIFI_IFACE, }wlp*, wlan*, wl*" >&2
                return 1
            fi
            ;;
        *)
            echo "dds_iface_setup: unknown mode '$DDS_MODE' (auto|cable|wifi|any)" >&2
            return 2
            ;;
    esac

    DDS_PROFILE=$(mktemp -t dds_profile.XXXXXX.xml)
    _dds_write_profile "$DDS_ADDR" "$DDS_PROFILE"
    # The callers all run --rm containers; nothing should outlive them.
    trap 'rm -f "$DDS_PROFILE"' EXIT
    return 0
}

# One line saying which link was chosen, with an optional trailing note. Three
# callers printed their own near-identical version of this and each had to be
# taught separately that DDS_MODE is now the RESOLVED mode, not the flag.
dds_iface_report() {
    local via=""
    [ "${DDS_REQUESTED:-}" = auto ] && via=" (auto)"
    if [ -n "$DDS_PROFILE" ]; then
        echo "link: ${DDS_MODE}${via}  ($DDS_IFACE $DDS_ADDR)${1:+  $1}"
    else
        echo "link: any${via} (DDS picks; may use the wifi)${1:+  $1}"
    fi
}
