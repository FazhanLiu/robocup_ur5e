## GraspResult.msg
## Result from grasp planning or execution

# Status codes
uint8 SUCCESS = 0
uint8 NO_GRASP_FOUND = 1
uint8 UNREACHABLE = 2
uint8 COLLISION = 3
uint8 EXECUTION_FAILED = 4

uint8 status                      # Result status code
string message                    # Human-readable message

# Grasp information (if successful)
geometry_msgs/PoseStamped grasp_pose  # Final grasp pose
float32 grasp_quality             # Quality metric [0-1]
float32 execution_time            # Time taken (seconds)
