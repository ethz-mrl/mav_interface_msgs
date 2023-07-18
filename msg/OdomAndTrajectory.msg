# The current MAV state estimate.
nav_msgs/Odometry odometry

# The reference trajectory for the current MAV state estimate. If empty the current reference
# trajectory will be retained, otherwise the MPC reference queue will be flushed and the supplied
# reference trajectory used instead.
mav_interface_msgs/FullStateTrajectory trajectory
