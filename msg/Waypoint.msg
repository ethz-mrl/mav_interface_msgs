# A waypoint is considered complete when
#   position_error <= position_tolerance && orientation_error <= orientation_tolerance
# with
#   position_error = (r_WB_ref - r_WB).norm()
# and
#   orientation_error = 2 * acos((q_WB_ref.inverse() * q_WB).w())

geometry_msgs/Pose pose                  # Pose of the body frame expressed in the world frame T_WB in [m].
float64            position_tolerance    # Position tolerance in [m].
float64            orientation_tolerance # Orientation tolerance in [rad].
