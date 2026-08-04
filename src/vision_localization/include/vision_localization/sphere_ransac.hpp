#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "vision_localization/localization_config.hpp"
#include "vision_localization/pipeline_types.hpp"

namespace vision_localization
{

/**
 * @brief 一次球体RANSAC拟合的原始结果。
 *
 * center只包含点云坐标数值；frame_id和stamp由CoordinateEstimator在
 * 调用成功后由CoordinateEstimator使用同步输入补齐。
 */
struct SphereFitResult
{
  PipelineStatus status = PipelineStatus::kNotConfigured;
  std::string message;

  Point3D center;

  float radius = 0.0F;
  float rmse = 0.0F;
  float inlier_ratio = 0.0F;

  std::uint32_t input_point_count = 0U;
  std::uint32_t inlier_count = 0U;
};

/**
 * @brief 使用已知半径范围约束的PCL球体RANSAC。
 *
 * 本类只负责模型拟合和原始残差统计。结果在发布前
 * 还会经过QualityGate和TF转换。
 */
class SphereRansac
{
public:
  SphereRansac() = default;
  ~SphereRansac() = default;

  bool initialize(const LocalizationConfig& config);

  SphereFitResult fit(
      const pcl::PointCloud<pcl::PointXYZ>& points) const;

  bool isConfigured() const;

private:
  double minimum_radius_ = 0.0;
  double maximum_radius_ = 0.0;
  double distance_threshold_ = 0.0;
  int maximum_iterations_ = 0;
  bool configured_ = false;
};

}  // namespace vision_localization
