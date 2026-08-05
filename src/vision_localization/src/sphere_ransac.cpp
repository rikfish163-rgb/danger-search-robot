#include "vision_localization/sphere_ransac.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <sstream>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace vision_localization
{

bool SphereRansac::initialize(
    const LocalizationConfig& config)
{
  configured_ = false;

  if (!std::isfinite(config.expected_sphere_radius) ||
      !std::isfinite(config.sphere_radius_tolerance) ||
      !std::isfinite(config.ransac_distance_threshold) ||
      config.expected_sphere_radius <= 0.0 ||
      config.sphere_radius_tolerance <= 0.0 ||
      config.sphere_radius_tolerance >=
          config.expected_sphere_radius ||
      config.ransac_distance_threshold <= 0.0 ||
      config.ransac_max_iterations <= 0)
  {
    return false;
  }

  minimum_radius_ =
      config.expected_sphere_radius -
      config.sphere_radius_tolerance;

  maximum_radius_ =
      config.expected_sphere_radius +
      config.sphere_radius_tolerance;

  distance_threshold_ =
      config.ransac_distance_threshold;

  maximum_iterations_ =
      config.ransac_max_iterations;

  configured_ = true;
  return true;
}

SphereFitResult SphereRansac::fit(
    const pcl::PointCloud<pcl::PointXYZ>& points) const
{
  SphereFitResult result;

  if (!configured_)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message = "SphereRansac尚未初始化。";
    return result;
  }

  if (points.size() < 4U)
  {
    result.status = PipelineStatus::kInsufficientPoints;
    result.message = "球体拟合至少需要4个三维点。";
    return result;
  }

  if (points.size() >
      static_cast<std::size_t>(
          std::numeric_limits<std::uint32_t>::max()))
  {
    result.status = PipelineStatus::kInternalError;
    result.message = "输入点数超过uint32_t可表示范围。";
    return result;
  }

  for (const pcl::PointXYZ& point : points.points)
  {
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z))
    {
      result.status = PipelineStatus::kInternalError;
      result.message = "球体拟合输入仍包含NaN或Inf。";
      return result;
    }
  }

  result.input_point_count =
      static_cast<std::uint32_t>(points.size());

  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud =
      points.makeShared();

  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_SPHERE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setMaxIterations(maximum_iterations_);
  segmentation.setDistanceThreshold(distance_threshold_);
  segmentation.setRadiusLimits(
      minimum_radius_,
      maximum_radius_);
  segmentation.setInputCloud(cloud);

  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;

  try
  {
    segmentation.segment(inliers, coefficients);
  }
  catch (const std::exception& exception)
  {
    result.status = PipelineStatus::kSphereFitFailed;
    result.message =
        std::string("PCL球体RANSAC抛出异常：") +
        exception.what();
    return result;
  }
  catch (...)
  {
    result.status = PipelineStatus::kSphereFitFailed;
    result.message = "PCL球体RANSAC发生未知异常。";
    return result;
  }

  if (coefficients.values.size() < 4U ||
      inliers.indices.empty())
  {
    result.status = PipelineStatus::kSphereFitFailed;
    result.message = "RANSAC未找到有效球体模型。";
    return result;
  }

  const float center_x = coefficients.values[0];
  const float center_y = coefficients.values[1];
  const float center_z = coefficients.values[2];
  const float radius = coefficients.values[3];

  if (!std::isfinite(center_x) ||
      !std::isfinite(center_y) ||
      !std::isfinite(center_z) ||
      !std::isfinite(radius) ||
      radius <= 0.0F)
  {
    result.status = PipelineStatus::kSphereFitFailed;
    result.message = "球体模型系数包含无效数值。";
    return result;
  }

  double squared_error_sum = 0.0;
  std::uint32_t checked_inlier_count = 0U;

  for (const int index : inliers.indices)
  {
    if (index < 0 ||
        static_cast<std::size_t>(index) >= points.size())
    {
      result.status = PipelineStatus::kInternalError;
      result.message = "PCL返回了越界的球体内点索引。";
      return result;
    }

    const pcl::PointXYZ& point =
        points.points[static_cast<std::size_t>(index)];

    const double dx =
        static_cast<double>(point.x) - center_x;
    const double dy =
        static_cast<double>(point.y) - center_y;
    const double dz =
        static_cast<double>(point.z) - center_z;

    const double radial_distance =
        std::sqrt(dx * dx + dy * dy + dz * dz);

    const double residual =
        radial_distance - static_cast<double>(radius);

    if (!std::isfinite(residual))
    {
      result.status = PipelineStatus::kInternalError;
      result.message = "球面残差计算产生NaN或Inf。";
      return result;
    }

    squared_error_sum += residual * residual;
    ++checked_inlier_count;
  }

  if (checked_inlier_count == 0U)
  {
    result.status = PipelineStatus::kSphereFitFailed;
    result.message = "球体模型没有可用于计算RMSE的内点。";
    return result;
  }

  const double rmse =
      std::sqrt(
          squared_error_sum /
          static_cast<double>(checked_inlier_count));

  const double inlier_ratio =
      static_cast<double>(checked_inlier_count) /
      static_cast<double>(points.size());

  if (!std::isfinite(rmse) ||
      !std::isfinite(inlier_ratio))
  {
    result.status = PipelineStatus::kInternalError;
    result.message = "球体质量指标包含NaN或Inf。";
    return result;
  }

  result.center.x = center_x;
  result.center.y = center_y;
  result.center.z = center_z;
  result.radius = radius;
  result.rmse = static_cast<float>(rmse);
  result.inlier_count = checked_inlier_count;
  result.inlier_ratio = static_cast<float>(inlier_ratio);
  result.status = PipelineStatus::kSuccess;

  std::ostringstream stream;
  stream
      << "球体RANSAC拟合完成：radius="
      << result.radius
      << ", rmse="
      << result.rmse
      << ", inliers="
      << result.inlier_count
      << "/"
      << result.input_point_count
      << "。";
  result.message = stream.str();

  return result;
}

bool SphereRansac::isConfigured() const
{
  return configured_;
}

}  // namespace vision_localization
