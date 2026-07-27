uint8 STATUS_WAITING=0
uint8 STATUS_EMPTY=1
uint8 STATUS_OCCUPIED=2
uint8 STATUS_UNCERTAIN=3

string slot_id
uint8 status
uint32 point_count
uint32 point_threshold
float32 height_threshold_m
geometry_msgs/Point center
float32 width
float32 length
