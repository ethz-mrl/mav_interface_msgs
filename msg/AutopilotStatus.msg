Header header

bool   initialised  # Is the Autopilot up and running ? 
uint8  taskType     # Type (Idle, Waypoint, Trajectory) of the active reference task.
string taskID       # ID of the active reference task

# Reserved for the taskType
uint8  Idle = 0        # Autopilot is active, Reference Queue is empty
uint8  Waypoint = 1    # Autopilot is active, Current reference is a waypoint
uint8  Trajectory = 2  # Autopilot is active, Current reference is a trajectory