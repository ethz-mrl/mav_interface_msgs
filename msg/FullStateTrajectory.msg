Header header

string             taskID               # ID of the reference trajectory.
Waypoint           initialWaypoint      # Initial waypoint, where the trajectory starts from.
FullStateStamped[] trajectory           # A full state trajectory is a sequence of timestamped
                                        # state references. The state timestamps should be relative
                                        # to the first state in the trajectory whose timestamp
                                        # should be 0.
bool               flushReferenceQueue  # Whether to flush the reference queue.
