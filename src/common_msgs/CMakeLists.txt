cmake_minimum_required(VERSION 3.0.2)
project(common_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
)

## 生成消息文件
add_message_files(
  FILES
  DetectedObject.msg
  GraspCandidate.msg
  ObjectScore.msg
  TaskDecision.msg
  GraspResult.msg
  MotionCommand.msg
  PathPlanRequest.msg
)

## Generate message dependencies
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    std_msgs 
    sensor_msgs 
    geometry_msgs
    trajectory_msgs
    moveit_msgs
)
