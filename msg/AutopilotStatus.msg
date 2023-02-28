# taskType enum values
uint8  Idle = 0        # Autopilot is active, reference queue is empty.
uint8  Waypoint = 1    # Autopilot is active, current reference is a waypoint.
uint8  Trajectory = 2  # Autopilot is active, current reference is a trajectory.

Header header

bool   initialised  # Whether the autopilot is active.
uint8  taskType     # Type (Idle, Waypoint, Trajectory) of the active reference task.
string taskID       # ID of the active reference task.
