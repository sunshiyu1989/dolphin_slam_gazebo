# 视觉描述符消息
# 用于传输特征描述符

std_msgs/Header header
int32 source_id              # 来源 ID（相机或声呐）
int32 template_id            # 视觉模板 ID
float32[] bow_descriptor     # BoW 描述符
int32 num_features          # 特征点数量
float32[] raw_descriptors   # 原始描述符（展平）
int32 descriptor_dim        # 每个描述符的维度
