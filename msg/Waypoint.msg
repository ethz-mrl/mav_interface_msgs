# A waypoint is considered complete when
#   positionError <= positionTolerance && orientationError <= orientationTolerance
# with
#   positionError = (r_WB_ref - r_WB).norm()
# and
#   orientationError = 2 * acos((q_WB_ref.inverse() * q_WB).w())

geometry_msgs/Vector3    position              # Reference position r_WB_ref in [m].
geometry_msgs/Quaternion orientation           # Reference orientation q_WB_ref.
float64                  positionTolerance     # Position tolerance in [m].
float64                  orientationTolerance  # Orientation tolerance in [rad].
