#pragma once

#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <danger_target_manager/DangerObservation.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "vision_localization/localization_config.hpp"
#include "vision_localization/pipeline_types.hpp"

namespace vision_localization
{

/**
 * 将球心定位结果转换成标准ROS消息并发布。
 *
 * 输出：
 * 1. 下游目标管理观测：danger_target_manager/DangerObservation；
 * 2. 点云坐标系球心：geometry_msgs/PointStamped；
 * 3. 目标坐标系球心：geometry_msgs/PointStamped；
 * 4. ROI调试点云：sensor_msgs/PointCloud2。
 */
class OutputAdapter
{
public:
  OutputAdapter(
      ros::NodeHandle& node_handle,
      ros::NodeHandle& private_node_handle);

  bool initialize(
      const LocalizationConfig& config);

  PipelineResult publishCameraCandidate(
      const Point3D& point);

  PipelineResult publishWorldCandidate(
      const Point3D& point);

  /** 发布一条通过RANSAC、质量门和TF的有效观测。 */
  PipelineResult publishDangerObservation(
      const DangerObservationData& observation);

  /**
   * 发布失败/无检测帧。数值字段写NaN且valid=false，
   * 下游节点将其处理为一次未命中。
   */
  PipelineResult publishInvalidDangerObservation(
      const ros::Time& stamp,
      DangerObservationStatusCode status_code,
      float detector_confidence);

  PipelineResult publishRoiDebug(
      const pcl::PointCloud<pcl::PointXYZ>& cloud,
      const std::string& frame_id,
      const ros::Time& stamp);

  bool isConfigured() const;

private:
  PipelineResult validatePoint(
      const Point3D& point) const;

  static geometry_msgs::PointStamped
  makePointStamped(
      const Point3D& point);

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  ros::Publisher camera_candidate_publisher_;
  ros::Publisher world_candidate_publisher_;
  ros::Publisher roi_debug_publisher_;
  ros::Publisher danger_observation_publisher_;

  std::string target_frame_;

  bool publish_debug_ = false;
  bool configured_ = false;
};

}  // namespace vision_localization
