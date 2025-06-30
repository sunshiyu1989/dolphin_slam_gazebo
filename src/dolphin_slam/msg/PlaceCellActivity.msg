# 位置细胞活动消息
# 传输位置细胞网络的活动状态

std_msgs/Header header
float32[] activity          # 活动矩阵（展平的 3D 数组）
int32[] shape              # 形状 [x, y, z]
float32[] peak_position    # 峰值位置 [x, y, z]
float32 peak_value         # 峰值强度
float32[] current_position # 当前估计位置
