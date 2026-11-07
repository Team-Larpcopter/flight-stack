# Geometry only: this route can traverse unknown space. Execution must validate
# against the current observed-free inflated ROG-Map and a safe stopping path.
uint8 INVALID=0
uint8 VALID=1
uint8 REACHED=2
std_msgs/Header header
uint64 localization_epoch
uint64 goal_id
uint64 map_revision
uint64 route_id
uint8 status
geometry_msgs/Point[] points
string reason
