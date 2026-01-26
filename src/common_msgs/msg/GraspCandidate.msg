# GraspCandidate.msg - 抓取候选姿态
# 从感知模块传递到规划模块的抓取建议

geometry_msgs/PoseStamped pose  # 抓取姿态（在相机或机器人坐标系中）
float32 quality                  # 抓取质量评分 [0.0, 1.0]
