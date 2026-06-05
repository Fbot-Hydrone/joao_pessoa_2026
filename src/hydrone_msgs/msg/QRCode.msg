# QR Code detection result (Phase 4)
std_msgs/Header header
string qr_id            # Letter identifier: A, B, C, D or E
geometry_msgs/Pose pose # Estimated pose of the QR code in the world
bool is_new             # True if this is the first time this QR was detected
