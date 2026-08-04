#include "vision_localization/tf_transformer.hpp"

#include <cmath>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace vision_localization
{

TFTransformer::TFTransformer(ros::NodeHandle& node_handle)
  : tf_listener_(tf_buffer_)
{
  (void)node_handle;
}

PipelineResult TFTransformer::transform(
    const Point3D& source_point,
    const std::string& target_frame,
    double timeout_seconds,
    Point3D& transformed_point)
{
  PipelineResult result;

  if (source_point.frame_id.empty())
  {
    result.status = PipelineStatus::kInvalidFrame;
    result.message = "source_point.frame_id为空，无法执行TF转换。";
    return result;
  }

  if (source_point.stamp.isZero())
  {
    result.status = PipelineStatus::kInvalidTimestamp;
    result.message = "source_point.stamp无效，无法执行TF转换。";
    return result;
  }

  if (target_frame.empty())
  {
    result.status = PipelineStatus::kInvalidFrame;
    result.message = "target_frame为空，无法执行TF转换。";
    return result;
  }

  geometry_msgs::PointStamped source_msg;
  source_msg.header.frame_id = source_point.frame_id;
  source_msg.header.stamp = source_point.stamp;
  source_msg.point.x = source_point.x;
  source_msg.point.y = source_point.y;
  source_msg.point.z = source_point.z;

  try
  {
    geometry_msgs::PointStamped world_msg =
        tf_buffer_.transform(source_msg,
                             target_frame,
                             ros::Duration(timeout_seconds));

    if (!std::isfinite(world_msg.point.x) ||
        !std::isfinite(world_msg.point.y) ||
        !std::isfinite(world_msg.point.z))
    {
      result.status = PipelineStatus::kTransformUnavailable;
      result.message = "TF转换结果包含NaN或Inf。";
      return result;
    }

    transformed_point.x = world_msg.point.x;
    transformed_point.y = world_msg.point.y;
    transformed_point.z = world_msg.point.z;
    transformed_point.frame_id = world_msg.header.frame_id;
    transformed_point.stamp = world_msg.header.stamp;

    result.status = PipelineStatus::kSuccess;
    result.message = "TF转换成功。";
    return result;
  }
  catch (const tf2::TransformException& exception)
  {
    result.status = PipelineStatus::kTransformUnavailable;
    result.message = exception.what();
    return result;
  }
}

}  // namespace vision_localization
