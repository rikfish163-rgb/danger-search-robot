#include "vision_localization/output_adapter.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <string>

#include <pcl_conversions/pcl_conversions.h>

namespace vision_localization
{

OutputAdapter::OutputAdapter(
    ros::NodeHandle& node_handle,
    ros::NodeHandle& private_node_handle)
  : node_handle_(node_handle),
    private_node_handle_(private_node_handle)
{
}

bool OutputAdapter::initialize(
    const LocalizationConfig& config)
{
  configured_ = false;
  publish_debug_ = config.publish_debug;
  target_frame_ = config.target_frame;

  if (publish_debug_ && config.camera_output_topic.empty())
  {
    ROS_ERROR(
        "OutputAdapter: camera_output_topic is empty.");
    return false;
  }

  if (config.world_output_topic.empty())
  {
    ROS_ERROR(
        "OutputAdapter: world_output_topic is empty.");
    return false;
  }

  if (config.danger_observation_topic.empty())
  {
    ROS_ERROR(
        "OutputAdapter: danger_observation_topic is empty.");
    return false;
  }

  if (target_frame_.empty())
  {
    ROS_ERROR("OutputAdapter: target_frame is empty.");
    return false;
  }

  if (publish_debug_ &&
      config.roi_debug_topic.empty())
  {
    ROS_ERROR(
        "OutputAdapter: publish_debug is enabled "
        "but roi_debug_topic is empty.");
    return false;
  }

  if (publish_debug_)
  {
    camera_candidate_publisher_ =
        node_handle_.advertise<
            geometry_msgs::PointStamped>(
            config.camera_output_topic,
            config.publisher_queue_size,
            false);
  }

  world_candidate_publisher_ =
      node_handle_.advertise<
          geometry_msgs::PointStamped>(
          config.world_output_topic,
          config.publisher_queue_size,
          false);

  danger_observation_publisher_ =
      node_handle_.advertise<
          danger_target_manager::DangerObservation>(
          config.danger_observation_topic,
          config.publisher_queue_size,
          false);

  if (publish_debug_)
  {
    roi_debug_publisher_ =
        node_handle_.advertise<
            sensor_msgs::PointCloud2>(
            config.roi_debug_topic,
            1,
            false);
  }

  configured_ = true;

  ROS_INFO(
      "OutputAdapter ready | camera=%s | world=%s | "
      "danger_observation=%s | roi_debug=%s | debug_enabled=%s",
      publish_debug_
          ? config.camera_output_topic.c_str()
          : "(disabled)",
      config.world_output_topic.c_str(),
      config.danger_observation_topic.c_str(),
      publish_debug_
          ? config.roi_debug_topic.c_str()
          : "(disabled)",
      publish_debug_ ? "true" : "false");

  return true;
}

PipelineResult OutputAdapter::validatePoint(
    const Point3D& point) const
{
  PipelineResult result;

  if (!configured_)
  {
    result.status =
        PipelineStatus::kOutputNotConfigured;
    result.message =
        "OutputAdapter尚未初始化。";
    return result;
  }

  if (point.frame_id.empty())
  {
    result.status =
        PipelineStatus::kInvalidFrame;
    result.message =
        "待发布候选点缺少frame_id。";
    return result;
  }

  if (point.stamp.isZero())
  {
    result.status =
        PipelineStatus::kInvalidTimestamp;
    result.message =
        "待发布候选点缺少有效时间戳。";
    return result;
  }

  if (!std::isfinite(point.x) ||
      !std::isfinite(point.y) ||
      !std::isfinite(point.z))
  {
    result.status =
        PipelineStatus::kInternalError;
    result.message =
        "待发布候选点包含NaN或Inf。";
    return result;
  }

  result.status = PipelineStatus::kSuccess;
  result.message = "候选点有效。";
  return result;
}

geometry_msgs::PointStamped
OutputAdapter::makePointStamped(
    const Point3D& point)
{
  geometry_msgs::PointStamped message;

  message.header.frame_id = point.frame_id;
  message.header.stamp = point.stamp;

  message.point.x = point.x;
  message.point.y = point.y;
  message.point.z = point.z;

  return message;
}

PipelineResult OutputAdapter::publishCameraCandidate(
    const Point3D& point)
{
  if (!publish_debug_)
  {
    PipelineResult result;
    result.status = PipelineStatus::kOutputNotConfigured;
    result.message = "相机球心调试输出未启用。";
    return result;
  }

  const PipelineResult validation =
      validatePoint(point);

  if (validation.status !=
      PipelineStatus::kSuccess)
  {
    return validation;
  }

  camera_candidate_publisher_.publish(
      makePointStamped(point));

  PipelineResult result;
  result.status = PipelineStatus::kSuccess;
  result.message =
      "点云坐标系候选点发布成功。";

  ROS_DEBUG(
      "Camera candidate | frame=%s | "
      "stamp=%.9f | xyz=(%.6f, %.6f, %.6f)",
      point.frame_id.c_str(),
      point.stamp.toSec(),
      point.x,
      point.y,
      point.z);

  return result;
}

PipelineResult OutputAdapter::publishDangerObservation(
    const DangerObservationData& observation)
{
  PipelineResult result;

  const PipelineResult point_validation =
      validatePoint(observation.center);

  if (point_validation.status != PipelineStatus::kSuccess)
  {
    return point_validation;
  }

  if (observation.center.frame_id != target_frame_)
  {
    result.status = PipelineStatus::kInvalidFrame;
    result.message =
        "有效观测frame_id与配置的target_frame不一致。";
    return result;
  }

  if (!std::isfinite(observation.fitted_radius) ||
      !std::isfinite(observation.sphere_rmse) ||
      !std::isfinite(observation.inlier_ratio) ||
      !std::isfinite(observation.detector_confidence) ||
      observation.fitted_radius <= 0.0F ||
      observation.sphere_rmse < 0.0F ||
      observation.inlier_ratio <= 0.0F ||
      observation.inlier_ratio > 1.0F ||
      observation.detector_confidence < 0.0F ||
      observation.detector_confidence > 1.0F ||
      observation.roi_point_count == 0U ||
      observation.inlier_count == 0U ||
      observation.inlier_count > observation.roi_point_count)
  {
    result.status = PipelineStatus::kQualityRejected;
    result.message = "有效观测的球体质量字段无效。";
    return result;
  }

  danger_target_manager::DangerObservation message;
  message.header.frame_id = observation.center.frame_id;
  message.header.stamp = observation.center.stamp;
  message.center.x = observation.center.x;
  message.center.y = observation.center.y;
  message.center.z = observation.center.z;
  message.fitted_radius = observation.fitted_radius;
  message.sphere_rmse = observation.sphere_rmse;
  message.inlier_ratio = observation.inlier_ratio;
  message.roi_point_count = observation.roi_point_count;
  message.inlier_count = observation.inlier_count;
  message.detector_confidence = observation.detector_confidence;
  message.valid = true;
  message.status_code = static_cast<std::uint8_t>(
      DangerObservationStatusCode::kValid);

  danger_observation_publisher_.publish(message);

  result.status = PipelineStatus::kSuccess;
  result.message = "DangerObservation发布成功。";
  return result;
}

PipelineResult OutputAdapter::publishInvalidDangerObservation(
    const ros::Time& stamp,
    DangerObservationStatusCode status_code,
    float detector_confidence)
{
  PipelineResult result;

  if (!configured_)
  {
    result.status = PipelineStatus::kOutputNotConfigured;
    result.message = "OutputAdapter尚未初始化。";
    return result;
  }

  if (stamp.isZero())
  {
    result.status = PipelineStatus::kInvalidTimestamp;
    result.message = "无效观测缺少原始RGB时间戳。";
    return result;
  }

  if (status_code == DangerObservationStatusCode::kValid)
  {
    result.status = PipelineStatus::kInternalError;
    result.message = "invalid观测不得使用kValid状态码。";
    return result;
  }

  const float nan =
      std::numeric_limits<float>::quiet_NaN();

  danger_target_manager::DangerObservation message;
  message.header.frame_id = target_frame_;
  message.header.stamp = stamp;
  message.center.x = nan;
  message.center.y = nan;
  message.center.z = nan;
  message.fitted_radius = nan;
  message.sphere_rmse = nan;
  message.inlier_ratio = nan;
  message.roi_point_count = 0U;
  message.inlier_count = 0U;
  message.detector_confidence =
      std::isfinite(detector_confidence)
          ? detector_confidence
          : nan;
  message.valid = false;
  message.status_code =
      static_cast<std::uint8_t>(status_code);

  danger_observation_publisher_.publish(message);

  result.status = PipelineStatus::kSuccess;
  result.message = "无效DangerObservation状态已发布。";
  return result;
}

PipelineResult OutputAdapter::publishWorldCandidate(
    const Point3D& point)
{
  const PipelineResult validation =
      validatePoint(point);

  if (validation.status !=
      PipelineStatus::kSuccess)
  {
    return validation;
  }

  world_candidate_publisher_.publish(
      makePointStamped(point));

  PipelineResult result;
  result.status = PipelineStatus::kSuccess;
  result.message =
      "world坐标系候选点发布成功。";

  ROS_INFO_THROTTLE(
      1.0,
      "World candidate | frame=%s | "
      "stamp=%.9f | xyz=(%.6f, %.6f, %.6f)",
      point.frame_id.c_str(),
      point.stamp.toSec(),
      point.x,
      point.y,
      point.z);

  return result;
}

PipelineResult OutputAdapter::publishRoiDebug(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const std::string& frame_id,
    const ros::Time& stamp)
{
  PipelineResult result;

  if (!configured_)
  {
    result.status =
        PipelineStatus::kOutputNotConfigured;
    result.message =
        "OutputAdapter尚未初始化。";
    return result;
  }

  if (!publish_debug_)
  {
    result.status =
        PipelineStatus::kOutputNotConfigured;
    result.message =
        "ROI调试点云发布未启用。";
    return result;
  }

  if (frame_id.empty())
  {
    result.status =
        PipelineStatus::kInvalidFrame;
    result.message =
        "ROI调试点云缺少frame_id。";
    return result;
  }

  if (stamp.isZero())
  {
    result.status =
        PipelineStatus::kInvalidTimestamp;
    result.message =
        "ROI调试点云缺少有效时间戳。";
    return result;
  }

  if (cloud.empty())
  {
    result.status =
        PipelineStatus::kInsufficientPoints;
    result.message =
        "ROI调试点云为空。";
    return result;
  }

  for (const pcl::PointXYZ& point : cloud.points)
  {
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z))
    {
      result.status =
          PipelineStatus::kInternalError;
      result.message =
          "ROI调试点云包含NaN或Inf。";
      return result;
    }
  }

  sensor_msgs::PointCloud2 cloud_message;

  try
  {
    pcl::toROSMsg(
        cloud,
        cloud_message);
  }
  catch (const std::exception& exception)
  {
    result.status =
        PipelineStatus::kInternalError;

    result.message =
        std::string("ROI点云转换失败：") +
        exception.what();

    return result;
  }

  // 明确覆盖PCL头，保证使用点云坐标系以及
  // 原始RGB图像采集时刻。
  cloud_message.header.frame_id = frame_id;
  cloud_message.header.stamp = stamp;

  roi_debug_publisher_.publish(
      cloud_message);

  result.status = PipelineStatus::kSuccess;
  result.message = "ROI调试点云发布成功。";

  return result;
}

bool OutputAdapter::isConfigured() const
{
  return configured_;
}

}  // namespace vision_localization
