# 一帧RGB图像对应的全部YOLO检测结果。
# header来自原始RGB图像，用于下游时间同步。
# 推理成功但无目标时，detections为空数组。
std_msgs/Header header
uint32 image_width
uint32 image_height
Detection[] detections
