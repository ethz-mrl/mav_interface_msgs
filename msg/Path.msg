Header     header
string     child_frame_id       # Waypoints contain poses of the child_frame_id expressed in the
                                # header.frame_id.
string     taskID               # ID of the reference path.
Waypoint[] waypoints            # A path is a sequence of waypoints.
bool       flushReferenceQueue  # Whether to flush the reference queue.
