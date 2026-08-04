#pragma once

#include <cstdint>
#include <string>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "vision_localization/localization_config.hpp"
#include "vision_localization/pipeline_types.hpp"
#include "vision_localization/quality_gate.hpp"
#include "vision_localization/sphere_ransac.hpp"

namespace vision_localization
{

/**
 * @brief 已通过球体RANSAC与质量门的相机坐标系球心及质量指标。
 */
struct EstimateResult
{
  PipelineStatus status = PipelineStatus::kNotConfigured;
  std::string message;

  Point3D candidate;

  float fitted_radius = 0.0F;
  float sphere_rmse = 0.0F;
  float inlier_ratio = 0.0F;
  std::uint32_t roi_point_count = 0U;
  std::uint32_t inlier_count = 0U;
};

/**
 * @brief 基于已知半径约束的球心估计器。
 *
 * 输入是PointCloudROI已经过滤后的有限点；输出必须先通过已知半径约束
 * RANSAC和QualityGate。失败时返回错误状态，不生成有效观测。
 */
class CoordinateEstimator
{
public:
  CoordinateEstimator() = default;
  ~CoordinateEstimator() = default;

  bool initialize(const LocalizationConfig& config);

  EstimateResult estimate(
      const pcl::PointCloud<pcl::PointXYZ>& roi_points,
      const std::string& source_frame,
      const ros::Time& measurement_stamp) const;

private:
  SphereRansac sphere_ransac_;
  QualityGate quality_gate_;
  bool configured_ = false;
};

}  // namespace vision_localization
