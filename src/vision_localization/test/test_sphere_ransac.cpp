#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "vision_localization/localization_config.hpp"
#include "vision_localization/quality_gate.hpp"
#include "vision_localization/sphere_ransac.hpp"

namespace vl = vision_localization;

namespace
{

vl::LocalizationConfig makeConfig()
{
  vl::LocalizationConfig config;
  config.expected_sphere_radius = 0.15;
  config.sphere_radius_tolerance = 0.02;
  config.ransac_distance_threshold = 0.01;
  config.ransac_max_iterations = 1000;
  config.min_sphere_inliers = 80;
  config.min_sphere_inlier_ratio = 0.50;
  config.max_sphere_rmse = 0.01;
  return config;
}

pcl::PointCloud<pcl::PointXYZ> makeSphereWithOutliers()
{
  pcl::PointCloud<pcl::PointXYZ> cloud;

  constexpr double center_x = 0.10;
  constexpr double center_y = -0.05;
  constexpr double center_z = 2.80;
  constexpr double radius = 0.15;
  constexpr double pi = 3.14159265358979323846;

  // 构造可重复的球面点，不使用随机数，避免测试结果随平台变化。
  for (int latitude = 1; latitude < 18; ++latitude)
  {
    const double theta =
        pi * static_cast<double>(latitude) / 18.0;

    for (int longitude = 0; longitude < 36; ++longitude)
    {
      const double phi =
          2.0 * pi * static_cast<double>(longitude) / 36.0;

      pcl::PointXYZ point;
      point.x = static_cast<float>(
          center_x + radius * std::sin(theta) * std::cos(phi));
      point.y = static_cast<float>(
          center_y + radius * std::sin(theta) * std::sin(phi));
      point.z = static_cast<float>(
          center_z + radius * std::cos(theta));
      cloud.push_back(point);
    }
  }

  // 少量确定性离群点，验证RANSAC不是对全部ROI求最小二乘。
  for (int index = 0; index < 30; ++index)
  {
    pcl::PointXYZ point;
    point.x = static_cast<float>(0.6 + 0.01 * index);
    point.y = static_cast<float>(-0.4 + 0.005 * index);
    point.z = static_cast<float>(3.3 + 0.01 * index);
    cloud.push_back(point);
  }

  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  return cloud;
}

}  // namespace

TEST(SphereRansacTest, RecoversKnownSphere)
{
  const vl::LocalizationConfig config = makeConfig();
  vl::SphereRansac fitter;
  ASSERT_TRUE(fitter.initialize(config));

  const vl::SphereFitResult result =
      fitter.fit(makeSphereWithOutliers());

  ASSERT_EQ(result.status, vl::PipelineStatus::kSuccess)
      << result.message;
  EXPECT_NEAR(result.center.x, 0.10, 0.005);
  EXPECT_NEAR(result.center.y, -0.05, 0.005);
  EXPECT_NEAR(result.center.z, 2.80, 0.005);
  EXPECT_NEAR(result.radius, 0.15, 0.005);
  EXPECT_GT(result.inlier_count, 500U);
  EXPECT_GT(result.inlier_ratio, 0.90F);
  EXPECT_LT(result.rmse, 0.005F);
}

TEST(SphereRansacTest, QualityGateAcceptsGoodFit)
{
  const vl::LocalizationConfig config = makeConfig();
  vl::SphereRansac fitter;
  vl::QualityGate gate;
  ASSERT_TRUE(fitter.initialize(config));
  ASSERT_TRUE(gate.initialize(config));

  const vl::SphereFitResult fit =
      fitter.fit(makeSphereWithOutliers());
  ASSERT_EQ(fit.status, vl::PipelineStatus::kSuccess);

  const vl::PipelineResult quality = gate.evaluate(fit);
  EXPECT_EQ(quality.status, vl::PipelineStatus::kSuccess)
      << quality.message;
}

TEST(SphereRansacTest, RejectsTooFewPoints)
{
  const vl::LocalizationConfig config = makeConfig();
  vl::SphereRansac fitter;
  ASSERT_TRUE(fitter.initialize(config));

  pcl::PointCloud<pcl::PointXYZ> points;
  points.push_back(pcl::PointXYZ(0.0F, 0.0F, 1.0F));
  points.push_back(pcl::PointXYZ(0.1F, 0.0F, 1.0F));
  points.push_back(pcl::PointXYZ(0.0F, 0.1F, 1.0F));

  const vl::SphereFitResult result = fitter.fit(points);
  EXPECT_EQ(result.status, vl::PipelineStatus::kInsufficientPoints);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}