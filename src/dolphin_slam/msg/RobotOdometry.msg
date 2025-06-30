# 机器人里程计消息
# 扩展的里程计信息

std_msgs/Header header
geometry_msgs/Pose pose          # 位姿
geometry_msgs/Twist twist        # 速度
float64 total_distance          # 总行驶距离
float64[36] pose_covariance     # 位姿协方差
float64[36] twist_covariance    # 速度协方差
