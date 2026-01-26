## TaskDecision.msg
## High-level task decision from FSM

# Decision type
string SEARCH = "search"
string APPROACH = "approach"
string GRASP = "grasp"
string TRANSPORT = "transport"
string RELEASE = "release"
string RECOVER = "recover"

string task_type                  # Current task to execute

# Target object (if applicable)
int32 target_object_id            # ID of selected object (-1 if none)
string target_bin                 # Target bin color ("red" or "blue")

# Task parameters
geometry_msgs/PoseStamped target_pose  # Target pose for motion
float32 timeout                   # Task timeout (seconds)
int32 retry_count                 # Number of retries allowed
