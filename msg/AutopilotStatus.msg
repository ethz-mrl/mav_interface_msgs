Header header

uint8  taskType  # Type (Idle, Waypoint, Trajectory) of the active reference task.
string taskID    # ID of the active reference task

uint8  Idle = 0
uint8  Waypoint = 1
uint8  Trajectory = 2