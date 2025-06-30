# 经验事件消息
# 用于通知经验的创建或更新

std_msgs/Header header
int32 experience_id        # 经验 ID
int32 action              # 动作类型（0=创建，1=更新，2=闭环）
float64 x                 # 位置
float64 y
float64 z
float64 theta             # 朝向
int32 visual_template_id  # 关联的视觉模板
float32 confidence        # 置信度
int32[] connected_to      # 连接的经验 ID 列表
