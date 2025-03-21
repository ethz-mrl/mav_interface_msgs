# taskType enum values
#uint8  idle = 0        # Autopilot is active, reference queue is empty.
#uint8  Waypoint = 1    # Autopilot is active, current reference is a waypoint.
#uint8  Trajectory = 2  # Autopilot is active, current reference is a trajectory.

std_msgs/Header header

bool   initialised  # Whether the autopilot is active.
uint8  task_type     # Type (Idle, Waypoint, Trajectory) of the active reference task.
string task_id       # ID of the active reference task.
