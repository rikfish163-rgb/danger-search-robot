#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "vision_localization/localization_config.hpp"
#include "vision_localization/input_adapter.hpp"
#include "vision_localization/pointcloud_roi.hpp"
#include "vision_localization/coordinate_estimator.hpp"
#include "vision_localization/tf_transformer.hpp"
#include "vision_localization/output_adapter.hpp"

namespace vision_localization
{

class DangerLocalizationNode
{
public:
  DangerLocalizationNode(
      ros::NodeHandle& node_handle,
      ros::NodeHandle& private_node_handle);

  bool initialize();

private:
  void handleSynchronizedFrame(
      const DetectionFrame& detection_frame,
      const sensor_msgs::PointCloud2ConstPtr& cloud_message);

  bool processDetection(
      const DetectionFrame& detection_frame,
      const BoundingBox2D& detection,
      const sensor_msgs::PointCloud2& cloud_message);

  void reportFailure(
      const std::string& message,
      const std::string& stage);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  LocalizationConfig config_;
  InputAdapter input_adapter_;
  PointCloudROI roi_extractor_;
  CoordinateEstimator coordinate_estimator_;
  TFTransformer tf_transformer_;
  OutputAdapter output_adapter_;

  bool initialized_ = false;
};

}  // namespace vision_localization
