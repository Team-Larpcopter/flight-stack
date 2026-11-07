# Every VALID update certifies the complete stopping curve against free space.
# Dynamic maps use the sensor capture time in header.stamp. Explicit immutable
# CAD maps use the completed revalidation time, with fresh localization required.
# A controller must enforce certificate freshness in both modes.
std_msgs/Header header
uint64 localization_epoch
uint64 goal_id
uint64 map_revision
uint64 sequence
builtin_interfaces/Time start_time
uint8 INVALID=0
uint8 VALID=1
uint8 status
PolynomialPiece[] position
PolynomialPiece[] yaw
