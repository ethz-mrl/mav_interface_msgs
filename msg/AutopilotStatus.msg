# Possible status values.
uint8 INACTIVE = 0   # Autopilot is not active.
uint8 IDLE = 1       # Autopilot is active, reference queue is empty.
uint8 WAYPOINT = 2   # Autopilot is active, current reference is a waypoint.
uint8 TRAJECTORY = 3 # Autopilot is active, current reference is a trajectory.

Header header
uint8  status # See possible values above.
string taskID # ID of the active reference task.
