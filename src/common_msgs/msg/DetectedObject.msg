# DetectedObject.msg - 检测到的物体信息
# 作为感知与规划模块之间的接口契约

string label                      # 物体类别标签
float32 score                     # 置信度得分 [0.0, 1.0]
sensor_msgs/RegionOfInterest roi  # 图像中的感兴趣区域
