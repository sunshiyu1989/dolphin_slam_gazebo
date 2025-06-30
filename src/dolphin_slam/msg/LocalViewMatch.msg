# 局部视觉匹配消息
# 传输视觉匹配结果

std_msgs/Header header
int32 query_id            # 查询 ID
int32 matched_template_id # 匹配的模板 ID（-1 表示无匹配）
float32 similarity        # 相似度分数
int32[] top_matches       # 前 N 个匹配的模板 ID
float32[] similarities    # 对应的相似度分数
