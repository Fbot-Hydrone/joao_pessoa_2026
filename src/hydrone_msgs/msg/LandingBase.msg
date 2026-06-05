# Represents a detected landing base in the arena
std_msgs/Header header
uint8 base_id          # 0-5 (6 bases total)
geometry_msgs/Pose pose  # Position and orientation of the base
bool is_suspended       # True if base is elevated (suspended)
bool is_visited         # Whether drone has already landed here
float32 confidence      # Detection confidence 0.0-1.0
float32 height          # Height above ground in meters
