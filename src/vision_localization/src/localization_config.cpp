#include "vision_localization/localization_config.hpp"

#include <cmath>

#include <ros/ros.h>

namespace vision_localization
{

bool loadParameters(
    ros::NodeHandle& node_handle,
    ros::NodeHandle& private_node_handle,
    LocalizationConfig& config)
{
  // 当前所有参数都从节点私有命名空间读取。
  // 保留node_handle参数是为了维持现有公共接口。
  (void)node_handle;

  private_node_handle.param<std::string>(
      "detection_topic",
      config.detection_topic,
      "");

  private_node_handle.param<std::string>(
      "pointcloud_topic",
      config.pointcloud_topic,
      "");

  private_node_handle.param<std::string>(
      "danger_observation_topic",
      config.danger_observation_topic,
      "/danger_observation");

  private_node_handle.param<std::string>(
      "camera_output_topic",
      config.camera_output_topic,
      "/vision_localization/camera_candidate");

  private_node_handle.param<std::string>(
      "world_output_topic",
      config.world_output_topic,
      "/vision_localization/world_candidate");

  private_node_handle.param<std::string>(
      "roi_debug_topic",
      config.roi_debug_topic,
      "/vision_localization/roi_cloud");

  private_node_handle.param<std::string>(
      "target_frame",
      config.target_frame,
      "world");

  private_node_handle.param<std::string>(
      "tf_source_frame_override",
      config.tf_source_frame_override,
      "");

  private_node_handle.param<std::string>(
      "target_class",
      config.target_class,
      "");

  private_node_handle.param<int>(
      "subscriber_queue_size",
      config.subscriber_queue_size,
      5);

  private_node_handle.param<int>(
      "publisher_queue_size",
      config.publisher_queue_size,
      20);

  private_node_handle.param<int>(
      "sync_queue_size",
      config.sync_queue_size,
      20);

  private_node_handle.param<double>(
      "sync_tolerance",
      config.sync_tolerance,
      0.08);

  private_node_handle.param<double>(
      "bbox_shrink_ratio",
      config.bbox_shrink_ratio,
      0.0);

  private_node_handle.param<double>(
      "min_range",
      config.min_range,
      0.0);

  private_node_handle.param<double>(
      "max_range",
      config.max_range,
      0.0);

  private_node_handle.param<int>(
      "min_roi_points",
      config.min_roi_points,
      0);

  private_node_handle.param<double>(
      "expected_sphere_radius",
      config.expected_sphere_radius,
      0.15);

  private_node_handle.param<double>(
      "sphere_radius_tolerance",
      config.sphere_radius_tolerance,
      0.02);

  private_node_handle.param<double>(
      "ransac_distance_threshold",
      config.ransac_distance_threshold,
      0.02);

  private_node_handle.param<int>(
      "ransac_max_iterations",
      config.ransac_max_iterations,
      500);

  private_node_handle.param<int>(
      "min_sphere_inliers",
      config.min_sphere_inliers,
      50);

  private_node_handle.param<double>(
      "min_sphere_inlier_ratio",
      config.min_sphere_inlier_ratio,
      0.15);

  private_node_handle.param<double>(
      "max_sphere_rmse",
      config.max_sphere_rmse,
      0.025);

  private_node_handle.param<double>(
      "tf_timeout",
      config.tf_timeout,
      0.15);

  private_node_handle.param<bool>(
      "publish_debug",
      config.publish_debug,
      false);

  return true;
}

PipelineResult validateConfig(const LocalizationConfig& config)
{
  PipelineResult result;

  if (config.detection_topic.empty())
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "detection_topic尚未配置。";
    return result;
  }

  if (config.pointcloud_topic.empty())
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "pointcloud_topic尚未配置。";
    return result;
  }

  if (config.danger_observation_topic.empty())
  {
    result.status = PipelineStatus::kOutputNotConfigured;
    result.message =
        "danger_observation_topic不能为空。";
    return result;
  }

  if (config.publish_debug &&
      config.roi_debug_topic.empty())
  {
    result.status =
        PipelineStatus::kOutputNotConfigured;

    result.message =
        "publish_debug已启用，但roi_debug_topic为空。";

    return result;
  }

  if (config.world_output_topic.empty())
  {
    result.status = PipelineStatus::kOutputNotConfigured;
    result.message =
        "world_output_topic不能为空。";
    return result;
  }

  if (config.target_frame.empty())
  {
    result.status = PipelineStatus::kInvalidFrame;
    result.message =
        "target_frame不能为空。";
    return result;
  }

  if (config.target_class.empty())
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "target_class尚未配置。";
    return result;
  }

  if (config.subscriber_queue_size <= 0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "subscriber_queue_size必须大于0。";
    return result;
  }

  if (config.publisher_queue_size <= 0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "publisher_queue_size必须大于0。";
    return result;
  }

  if (config.sync_queue_size <= 0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "sync_queue_size必须大于0。";
    return result;
  }

  if (!std::isfinite(config.sync_tolerance) ||
      config.sync_tolerance <= 0.0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "sync_tolerance必须大于0。";
    return result;
  }

  if (!std::isfinite(config.bbox_shrink_ratio) ||
      config.bbox_shrink_ratio < 0.0 ||
      config.bbox_shrink_ratio >= 0.5)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "bbox_shrink_ratio必须位于[0.0, 0.5)范围。";
    return result;
  }


  if (!std::isfinite(config.min_range) ||
      config.min_range <= 0.0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "min_range必须大于0。";
    return result;
  }

  if (!std::isfinite(config.max_range) ||
      config.max_range <= config.min_range)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "max_range必须大于min_range。";
    return result;
  }

  if (config.min_roi_points <= 0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "min_roi_points必须大于0。";
    return result;
  }

  if (!std::isfinite(config.expected_sphere_radius) ||
      config.expected_sphere_radius <= 0.0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "expected_sphere_radius必须为有限正数。";
    return result;
  }

  if (!std::isfinite(config.sphere_radius_tolerance) ||
      config.sphere_radius_tolerance <= 0.0 ||
      config.sphere_radius_tolerance >=
          config.expected_sphere_radius)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "sphere_radius_tolerance必须为正数且小于目标半径。";
    return result;
  }

  if (!std::isfinite(config.ransac_distance_threshold) ||
      config.ransac_distance_threshold <= 0.0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "ransac_distance_threshold必须为有限正数。";
    return result;
  }

  if (config.ransac_max_iterations <= 0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "ransac_max_iterations必须大于0。";
    return result;
  }

  if (config.min_sphere_inliers < 4)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "min_sphere_inliers不得小于4。";
    return result;
  }

  if (!std::isfinite(config.min_sphere_inlier_ratio) ||
      config.min_sphere_inlier_ratio <= 0.0 ||
      config.min_sphere_inlier_ratio > 1.0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "min_sphere_inlier_ratio必须位于(0, 1]。";
    return result;
  }

  if (!std::isfinite(config.max_sphere_rmse) ||
      config.max_sphere_rmse <= 0.0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "max_sphere_rmse必须为有限正数。";
    return result;
  }

  if (config.tf_timeout < 0.0)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "tf_timeout必须大于等于0。";
    return result;
  }

  // 只有打开调试输出时，才强制要求调试话题。
  if (config.publish_debug)
  {
    if (config.camera_output_topic.empty())
    {
      result.status = PipelineStatus::kOutputNotConfigured;
      result.message =
          "publish_debug为true时，camera_output_topic不能为空。";
      return result;
    }

    if (config.roi_debug_topic.empty())
    {
      result.status = PipelineStatus::kOutputNotConfigured;
      result.message =
          "publish_debug为true时，roi_debug_topic不能为空。";
      return result;
    }
  }

  result.status = PipelineStatus::kSuccess;
  result.message = "参数配置有效。";
  return result;
}

}  // namespace vision_localization
