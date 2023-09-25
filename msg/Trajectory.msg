Header         header
string         child_frame_id       # States contain poses of the child_frame_id expressed in
                                    # the header.frame_id.
string         taskID               # ID of the reference trajectory.
Waypoint       initialWaypoint      # Initial waypoint, where the trajectory starts from.
StateStamped[] trajectory           # A full state trajectory is a sequence of timestamped
                                    # state references. The state timestamps should be relative
                                    # to the first state in the trajectory whose timestamp
                                    # should be 0.
bool           flushReferenceQueue  # Whether to flush the reference queue.
