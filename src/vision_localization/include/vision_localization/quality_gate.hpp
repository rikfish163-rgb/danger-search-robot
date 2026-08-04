#pragma once

#include <cmath>
#include <sstream>

#include "vision_localization/localization_config.hpp"
#include "vision_localization/sphere_ransac.hpp"

namespace vision_localization
{

/**
 * @brief 球体模型进入TF转换前的质量门。
 *
 * 该类不修改拟合值，只决定是否允许其继续进入TF和目标管理器。
 */
class QualityGate
{
public:
  bool initialize(const LocalizationConfig& config)
  {
    configured_ = false;

    if (!std::isfinite(config.expected_sphere_radius) ||
        !std::isfinite(config.sphere_radius_tolerance) ||
        !std::isfinite(config.min_sphere_inlier_ratio) ||
        !std::isfinite(config.max_sphere_rmse) ||
        config.expected_sphere_radius <= 0.0 ||
        config.sphere_radius_tolerance <= 0.0 ||
        config.min_sphere_inliers < 4 ||
        config.min_sphere_inlier_ratio <= 0.0 ||
        config.min_sphere_inlier_ratio > 1.0 ||
        config.max_sphere_rmse <= 0.0)
    {
      return false;
    }

    expected_radius_ = config.expected_sphere_radius;
    radius_tolerance_ = config.sphere_radius_tolerance;
    minimum_inliers_ = config.min_sphere_inliers;
    minimum_inlier_ratio_ = config.min_sphere_inlier_ratio;
    maximum_rmse_ = config.max_sphere_rmse;
    configured_ = true;
    return true;
  }

  PipelineResult evaluate(const SphereFitResult& fit) const
  {
    PipelineResult result;

    if (!configured_)
    {
      result.status = PipelineStatus::kNotConfigured;
      result.message = "QualityGate尚未初始化。";
      return result;
    }

    if (fit.status != PipelineStatus::kSuccess)
    {
      result.status = fit.status;
      result.message = "质量门收到失败的拟合结果：" + fit.message;
      return result;
    }

    const bool radius_valid =
        std::isfinite(fit.radius) &&
        std::fabs(
            static_cast<double>(fit.radius) -
            expected_radius_) <= radius_tolerance_;

    const bool count_valid =
        fit.inlier_count >=
        static_cast<std::uint32_t>(minimum_inliers_);

    const bool ratio_valid =
        std::isfinite(fit.inlier_ratio) &&
        fit.inlier_ratio >= minimum_inlier_ratio_ &&
        fit.inlier_ratio <= 1.0F;

    const bool rmse_valid =
        std::isfinite(fit.rmse) &&
        fit.rmse >= 0.0F &&
        fit.rmse <= maximum_rmse_;

    if (!radius_valid ||
        !count_valid ||
        !ratio_valid ||
        !rmse_valid)
    {
      std::ostringstream stream;
      stream
          << "球体质量门拒绝：radius="
          << fit.radius
          << ", inliers="
          << fit.inlier_count
          << ", ratio="
          << fit.inlier_ratio
          << ", rmse="
          << fit.rmse
          << "。";

      result.status = PipelineStatus::kQualityRejected;
      result.message = stream.str();
      return result;
    }

    result.status = PipelineStatus::kSuccess;
    result.message = "球体拟合通过质量门。";
    return result;
  }

private:
  double expected_radius_ = 0.0;
  double radius_tolerance_ = 0.0;
  int minimum_inliers_ = 0;
  double minimum_inlier_ratio_ = 0.0;
  double maximum_rmse_ = 0.0;
  bool configured_ = false;
};

}  // namespace vision_localization
