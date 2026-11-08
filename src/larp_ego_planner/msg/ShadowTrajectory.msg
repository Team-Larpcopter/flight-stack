# Diagnostic candidate only. NOT a collision certificate or executable command.
# Empty control_points/knots and valid=false clear a previous candidate.
std_msgs/Header header
uint64 localization_epoch
uint64 map_revision
uint64 goal_id
bool valid
uint8 order
float64[] knots
geometry_msgs/Point[] control_points
