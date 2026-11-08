# Lightweight localization lease for an explicitly static MapSnapshot.
# Geometry is immutable within (localization_epoch, revision); this does NOT
# renew sensor observations in dynamic mapping mode. Published only while VIO
# is healthy. Header is the measured body pose timestamp, in map frame.
std_msgs/Header header
uint64 localization_epoch
uint64 revision
geometry_msgs/Pose body_pose
