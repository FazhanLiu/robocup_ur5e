## PathPlanRequest.msg
## Request for path planning service

# Planning parameters
geometry_msgs/PoseStamped start_pose
geometry_msgs/PoseStamped goal_pose

string planning_group             # MoveIt planning group name
float32 planning_time             # Max planning time (seconds)
int32 num_attempts                # Number of planning attempts

bool avoid_collisions             # Enable collision avoidance
string[] obstacle_ids             # List of obstacles to avoid

# Constraints (optional)
moveit_msgs/Constraints path_constraints
