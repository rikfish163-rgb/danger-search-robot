#include "vision_localization/coordinate_estimator.hpp"

#include <cmath>
#include <sstream>

namespace vision_localization
{

bool CoordinateEstimator::initialize(
    const LocalizationConfig& config)
{
  configured_ = false;

  if (!sphere_ransac_.initialize(config))
  {
    ROS_ERROR("CoordinateEstimator: SphereRansac初始化失败。");
    return false;
  }

  if (!quality_gate_.initialize(config))
  {
    ROS_ERROR("CoordinateEstimator: QualityGate初始化失败。");
    return false;
  }

  configured_ = true;
  return true;
}

EstimateResult CoordinateEstimator::estimate(
    const pcl::PointCloud<pcl::PointXYZ>& roi_points,
    const std::string& source_frame,
    const ros::Time& measurement_stamp) const
{
  EstimateResult result;

  if (!configured_)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "CoordinateEstimator尚未初始化。";
    return result;
  }

  if (source_frame.empty())
  {
    result.status = PipelineStatus::kInvalidFrame;
    result.message = "球心估计失败：source_frame为空。";
    return result;
  }

  if (measurement_stamp.isZero())
  {
    result.status = PipelineStatus::kInvalidTimestamp;
    result.message = "球心估计失败：RGB采集时间戳无效。";
    return result;
  }

  if (roi_points.empty())
  {
    result.status = PipelineStatus::kInsufficientPoints;
    result.message = "球心估计失败：ROI点云为空。";
    return result;
  }

  const SphereFitResult fit =
      sphere_ransac_.fit(roi_points);

  if (fit.status != PipelineStatus::kSuccess)
  {
    result.status = fit.status;
    result.message = fit.message;
    return result;
  }

  const PipelineResult quality =
      quality_gate_.evaluate(fit);

  if (quality.status != PipelineStatus::kSuccess)
  {
    result.status = quality.status;
    result.message = quality.message;
    return result;
  }

  if (!std::isfinite(fit.center.x) ||
      !std::isfinite(fit.center.y) ||
      !std::isfinite(fit.center.z))
  {
    result.status = PipelineStatus::kInternalError;
    result.message = "质量门通过后球心仍包含NaN或Inf。";
    return result;
  }

  result.candidate = fit.center;
  result.candidate.frame_id = source_frame;
  result.candidate.stamp = measurement_stamp;

  result.fitted_radius = fit.radius;
  result.sphere_rmse = fit.rmse;
  result.inlier_ratio = fit.inlier_ratio;
  result.roi_point_count = fit.input_point_count;
  result.inlier_count = fit.inlier_count;
  result.status = PipelineStatus::kSuccess;

  std::ostringstream stream;
  stream
      << "球心估计成功：frame="
      << source_frame
      << ", xyz=("
      << result.candidate.x
      << ", "
      << result.candidate.y
      << ", "
      << result.candidate.z
      << "), radius="
      << result.fitted_radius
      << ", rmse="
      << result.sphere_rmse
      << ", inliers="
      << result.inlier_count
      << "/"
      << result.roi_point_count
      << "。";
  result.message = stream.str();

  return result;
}

}  // namespace vision_localization
