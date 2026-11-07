# Local NED, frame_id="local_ned", with the same origin as PX4's local estimator.
# Header stamp is informational; the receiver watches monotonic receipt age.
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
# Optional PX4 heading (radians, clockwise from north) and its rate (rad/s).
# When yaw_valid is false the executive keeps the HOLD heading.
bool yaw_valid
float32 yaw
float32 yawspeed
# A trajectory ID is fixed for one flight; repeat the stationary final sample.
string trajectory_id
bool is_final

# Optional absolute ROS-clock deadline, converted to monotonic time on receipt.
# Required for planner collision certificates; false retains legacy behavior.
bool certificate_required
builtin_interfaces/Time valid_until
