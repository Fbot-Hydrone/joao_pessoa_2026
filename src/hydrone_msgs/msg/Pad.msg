# A landing pad in the world map, fused from many PadDetection observations.
# Maintained by hydrone_nav/pad_map_node.

# frame_id is the world frame every field below lives in (the frame of
# /mavros/local_position/pose, normally "map"). stamp = last map update.
std_msgs/Header header

# Stable id, assigned in order of first sighting. Never reused.
uint16 id

# Fused centre of the pad. x/y are the running mean of the accepted
# observations; z is the pad's TOP surface (see `height`).
geometry_msgs/Point position

# Best confidence seen so far, 0..1.
float32 confidence
# How many detections have been fused into this entry. Low counts are noise.
uint32 observations
# Stamp of the most recent detection fused in.
builtin_interfaces/Time last_seen

# Height of the pad's top surface above the takeoff plane, in metres. Starts at
# the projected z (0 for a ground pad under the flat-ground assumption) and is
# corrected to drone_z - rangefinder_range the first time the drone hovers over
# it — that is how the 0.5 m elevated base gets its real height.
float32 height
bool height_measured

# True once the drone has landed on this pad. The mission never targets a
# visited pad again.
bool visited

# True for the pad the drone was standing on when it first armed, registered
# through the RegisterTakeoffBase service rather than detected. The mission
# never treats it as a landing candidate — it is the one pad whose identity is
# known before any camera looks at it, so no confirmation hover is ever spent
# on it — and it is the target of the final return leg.
bool is_takeoff_base
