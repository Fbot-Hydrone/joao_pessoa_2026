# The full landing-pad map, republished on every change and at a slow heartbeat.
# Small enough (a handful of pads) that sending the whole map beats deltas.
std_msgs/Header header
Pad[] pads
