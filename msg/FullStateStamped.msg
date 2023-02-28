uint64                   timestampNanoSeconds  # Timestamp in [ns] relative to the trajectory initial state.
geometry_msgs/Vector3    position              # Reference position r_WB_ref in [m].
geometry_msgs/Quaternion orientation           # Reference orientation q_WB_ref.
geometry_msgs/Vector3    linearVelocity        # Reference linear velocity v_W_WB in [m/s].
geometry_msgs/Vector3    linearAcceleration    # Reference linear acceleration in world frame a_W_WB in [m/s^2].
geometry_msgs/Vector3    angularVelocity       # Reference angular velocity w_B_WB  in [rad/s].
