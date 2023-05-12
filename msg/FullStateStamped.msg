uint64                   timestampNanoSeconds  # Timestamp in [ns] relative to the trajectory initial state.
geometry_msgs/Vector3    position              # Position of the body frame expressed in the world frame r_WB in [m].
geometry_msgs/Quaternion orientation           # Orientation of the body frame expressed in the world frame q_WB.
geometry_msgs/Vector3    linearVelocity        # Linear velocity in the body frame v_B in [m/s].
geometry_msgs/Vector3    linearAcceleration    # Linear acceleration in the world frame a_W in [m/s^2].
geometry_msgs/Vector3    angularVelocity       # Angular velocity in the body frame w_B  in [rad/s].
