geometry_msgs/Vector3    position  		       # Reference position r_WB_ref. Units in [m]
geometry_msgs/Quaternion orientation		   # Reference orientation q_WB_ref. 
float64 		         positionTolerance     # Waypoint position tolerance in [m]
float64 		         orientationTolerance  # Orientation tolerance in [rad]


# A waypoint is considered complete when positionError.norm() <= positionTolerance && orientationError <= orientationTolerance
# with positionError = r_WB_ref - r_WB and orientationError = 2.0 * acos( ( q_WB_ref^(-1) * q_WB ).w() )