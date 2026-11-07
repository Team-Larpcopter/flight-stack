# Local NED, frame_id="local_ned", with the same origin as PX4's local estimator.
# Header stamp is informational; the receiver watches monotonic receipt age.
std_msgs/Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
