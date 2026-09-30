# One landing-pad detection produced from a SINGLE camera frame.
#
# The detector (hydrone_vision/pad_detector_node) publishes one of these per pad
# it finds per frame. It is a raw observation, not a map entry: nothing here is
# fused or tracked. hydrone_nav/pad_map_node consumes these and maintains the
# world map.

# stamp   = stamp of the source image
# frame_id = the WORLD frame `position` is expressed in (the frame of
#            /mavros/local_position/pose, normally "map"). Empty when
#            position_valid is false.
std_msgs/Header header

# Which camera saw it — "forward" (ZED) or "down". Free-form so a third camera
# needs no message change.
string camera

# Pad centre in the SOURCE IMAGE, in pixels (always valid).
float32 u
float32 v
# Equivalent radius of the blue region: sqrt(area/pi), in pixels. A cheap
# apparent-size measure that does not assume the pad is circular on screen.
float32 radius_px

# Detector confidence, 0..1 (see pad_detector.PadDetector for the scoring).
float32 confidence

# Pad centre in the world frame named by header.frame_id.
geometry_msgs/Point position
# False when the pixel could not be projected to the world (no camera_info, no
# vehicle pose, or a ray that never meets the ground plane). u/v/confidence are
# still meaningful in that case.
bool position_valid
# Camera-to-pad distance used for the projection, in metres (0 if invalid).
float32 range_m

# How `position` was obtained.
uint8 SOURCE_NONE=0
uint8 SOURCE_DEPTH=1          # back-projected with the registered depth image
uint8 SOURCE_GROUND_PLANE=2   # ray intersected with the assumed ground plane
uint8 source
