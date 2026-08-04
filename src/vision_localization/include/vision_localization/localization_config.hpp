#pragma once

#include <ros/ros.h>

#include <string>

#include "vision_localization/pipeline_types.hpp"

namespace vision_localization
{

/**
 * @brief 视觉定位节点的统一配置。
 *
 * 所有ROS话题名和运行参数集中放在这里，算法模块内部不应写死话题名。
 */
struct LocalizationConfig
{
  // 输入接口。
  std::string detection_topic;
  std::string pointcloud_topic;

  // 通过质量检查后的三维观测输出话题。
  std::string danger_observation_topic;

  // 目标坐标系中的调试候选点，消息类型为PointStamped。
  std::string world_output_topic;

  // RViz调试接口。
  std::string camera_output_topic;
  std::string roi_debug_topic;

  // 坐标系和检测类别。
  std::string target_frame;

  // 可选的TF源坐标系覆盖。
  // 为空时保持原始点云frame逻辑不变；
  // 非空时只在TF转换阶段覆盖候选点的frame_id，
  // 不修改点云XYZ、ROI或球体拟合结果。
  std::string tf_source_frame_override;

  std::string target_class;

  // ROS通信参数。
  int subscriber_queue_size = 0;
  int publisher_queue_size = 0;
  int sync_queue_size = 0;

  // 时间同步和TF参数。
  double sync_tolerance = 0.0;

  // ROI点云提取参数。
  double bbox_shrink_ratio = 0.0;
  double min_range = 0.0;
  double max_range = 0.0;
  int min_roi_points = 0;

  // 已知半径约束的球体RANSAC与质量门参数。
  double expected_sphere_radius = 0.0;
  double sphere_radius_tolerance = 0.0;
  double ransac_distance_threshold = 0.0;
  int ransac_max_iterations = 0;
  int min_sphere_inliers = 0;
  double min_sphere_inlier_ratio = 0.0;
  double max_sphere_rmse = 0.0;

  double tf_timeout = 0.0;


  // 是否发布点云坐标系候选点和ROI调试点云。
  bool publish_debug = false;
};

bool loadParameters(
    ros::NodeHandle& node_handle,
    ros::NodeHandle& private_node_handle,
    LocalizationConfig& config);

PipelineResult validateConfig(const LocalizationConfig& config);

}  // namespace vision_localization
