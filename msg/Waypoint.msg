# A waypoint is considered complete when
#   positionError <= positionTolerance && orientationError <= orientationTolerance
# with
#   positionError = (r_WB_ref - r_WB).norm()
# and
#   orientationError = 2 * acos((q_WB_ref.inverse() * q_WB).w())

geometry_msgs/Vector3    position              # Position of the body frame expressed in the world frame r_WB in [m].
geometry_msgs/Quaternion orientation           # Orientation of the body frame expressed in the world frame q_WB.
float64                  positionTolerance     # Position tolerance in [m].
float64                  orientationTolerance  # Orientation tolerance in [rad].
