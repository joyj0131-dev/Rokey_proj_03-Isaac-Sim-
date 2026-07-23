
bool data_ready          # false = /parking_slots 캐시 비어있음(runner 미기동)
bool exists
bool occupied
bool is_accessible
geometry_msgs/Pose pose  # map 프레임, orientation = 목표 yaw(quaternion z/w)
