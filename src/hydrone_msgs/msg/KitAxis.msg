# Which way a detected kit lies in the image (hydrone_vision/kit_axis_node).
# One per kit detection; header = the source image's.
std_msgs/Header header

# The detection it belongs to, pixels.
float32 u
float32 v

# Long axis in the image, rad, from +u towards +v, in [-pi/2, pi/2).
float32 angle

# 0 = no preferred direction .. 1 = a single edge. Low means do not turn.
float32 coherence
