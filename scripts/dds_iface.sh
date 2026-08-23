# Pin DDS to one network interface. Sourced, not executed:
#
#   . "$(dirname "$0")/dds_iface.sh"
#   dds_iface_setup cable        # -> DDS_ADDR, DDS_PROFILE, DDS_IFACE
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
# Overridable by environment: CABLE_SUBNET, CABLE_IFACE, WIFI_IFACE.

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

# dds_iface_setup <cable|wifi|any>
#
# Sets, for the caller to use:
#   DDS_MODE     the mode as given
#   DDS_IFACE    interface chosen ("" for any)
#   DDS_ADDR     its IPv4    ("" for any)
#   DDS_PROFILE  path to the generated XML ("" for any)
dds_iface_setup() {
    DDS_MODE="$1"
    DDS_IFACE=""
    DDS_ADDR=""
    DDS_PROFILE=""

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
            echo "dds_iface_setup: unknown mode '$DDS_MODE' (cable|wifi|any)" >&2
            return 2
            ;;
    esac

    DDS_PROFILE=$(mktemp -t dds_profile.XXXXXX.xml)
    _dds_write_profile "$DDS_ADDR" "$DDS_PROFILE"
    # The callers all run --rm containers; nothing should outlive them.
    trap 'rm -f "$DDS_PROFILE"' EXIT
    return 0
}
