# Human gesture/command detected by vision system (Phase 3)
std_msgs/Header header
string gesture_name     # e.g. "arms_up", "point_left", "point_right", etc.
float32 confidence      # Detection confidence 0.0-1.0
geometry_msgs/Point human_position  # Position of the human in the world
float32[] skeleton_keypoints        # Flat array of skeleton keypoints (x,y,z per joint)
# gesture_detector_node (one message per processed frame, person or not):
bool person_found       # false: nobody in the frame, gesture_name is "NENHUM"
float32 image_x         # operator's shoulder midpoint, [-1, 1], + = right of the image centre
float32 image_size      # shoulder width / image width (grows as the operator gets closer)
