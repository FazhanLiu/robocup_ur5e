## MotionCommand.msg
## Command for motion control (IK/FK)

# Command types
string MOVE_TO_POSE = "move_to_pose"
string MOVE_TO_JOINT = "move_to_joint"
string EXECUTE_TRAJECTORY = "execute_trajectory"
string STOP = "stop"
string HOME = "home"

string command_type               # Type of motion command

# Pose target (for MOVE_TO_POSE)
geometry_msgs/PoseStamped target_pose

# Joint target (for MOVE_TO_JOINT)
float64[] joint_positions         # Joint angles (radians)

# Trajectory (for EXECUTE_TRAJECTORY)
trajectory_msgs/JointTrajectory trajectory

# Execution parameters
float32 max_velocity              # Max velocity factor [0-1]
float32 max_acceleration          # Max acceleration factor [0-1]
bool collision_check              # Enable collision checking
