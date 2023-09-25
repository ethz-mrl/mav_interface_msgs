Header     header
string     child_frame_id         # Waypoints contain poses of the child_frame_id expressed in the
                                  # header.frame_id.
string     task_id                # ID of the reference path.
Waypoint[] waypoints              # A path is a sequence of waypoints.
bool       clear_past_references  # Whether to remove all previous references from the queue.
