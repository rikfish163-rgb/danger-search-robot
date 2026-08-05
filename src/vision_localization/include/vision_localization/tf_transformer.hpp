#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "vision_localization/pipeline_types.hpp"

namespace vision_localization
{

class TFTransformer
{
public:
  explicit TFTransformer(ros::NodeHandle& node_handle);

  PipelineResult transform(
      const Point3D& source_point,
      const std::string& target_frame,
      double timeout_seconds,
      Point3D& transformed_point);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace vision_localization
