Header header 

string 			   taskID   				 # String ID of the reference trajectory.
Waypoint           initialWaypoint	         # Initial waypoint, where the trajectory starts from
FullStateStamped[] trajectory				 # Full state trajectory is a sequence of timestamped fullstate references
bool               flushReferenceQueue       # When true flushes the reference Queue.