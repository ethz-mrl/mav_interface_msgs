std_msgs/Header   header

string     task_id               # ID of the reference path.
Waypoint[] waypoints            # A path is a sequence of waypoints.
bool       flush_reference_queue  # Whether to flush the reference queue.
