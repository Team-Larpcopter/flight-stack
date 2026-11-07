# Atomic classified ROG-Map snapshot. Header is depth CAPTURE time in map frame.
# Within [lower, upper], cells absent from both clouds are UNKNOWN, never free.
# Each cloud contains voxel centers; occupied wins if classifications overlap.
std_msgs/Header header
uint64 localization_epoch
uint64 revision
float64 resolution
geometry_msgs/Point lower
geometry_msgs/Point upper
sensor_msgs/PointCloud2 occupied
sensor_msgs/PointCloud2 free
# Body pose interpolated at the depth capture time; not a propagated control state.
geometry_msgs/PoseStamped body_pose
