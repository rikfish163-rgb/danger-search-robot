# 单帧目标观测。
# header.stamp为原始RGB采集时间；header.frame_id为center所在坐标系。
std_msgs/Header header
# 通过TF转换后的球心；valid=false时为NaN。
geometry_msgs/Point center
# 球体RANSAC拟合半径，单位m。
float32 fitted_radius
# 内点到拟合球面的径向残差RMSE，单位m。
float32 sphere_rmse
# 球体内点数占ROI有效点数的比例。
float32 inlier_ratio
uint32 roi_point_count
uint32 inlier_count
# 二维检测器置信度，范围[0,1]。
float32 detector_confidence
# true表示球体拟合、质量门和TF转换均成功。
bool valid
# 无效原因：0有效，1无检测，2 ROI失败，3估计失败，
# 4 TF失败，5质量拒绝，6未验证估计，255内部错误。
uint8 status_code
