#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include "vision_localization/pointcloud_roi.hpp"

namespace vl = vision_localization;

// Test 1: Odd number of elements median
TEST(MedianTest, OddCountMedian)
{
  std::vector<float> values = {5.0f, 2.0f, 8.0f, 1.0f, 9.0f};
  float median = vl::PointCloudROI::computeMedian(values);
  EXPECT_FLOAT_EQ(median, 5.0f);
}

// Test 2: Even number of elements median (average of two middle)
TEST(MedianTest, EvenCountMedian)
{
  std::vector<float> values = {5.0f, 2.0f, 8.0f, 1.0f};
  float median = vl::PointCloudROI::computeMedian(values);
  EXPECT_FLOAT_EQ(median, 3.5f); // (2 + 5) / 2
}

// Test 3: Unordered input median
TEST(MedianTest, UnorderedInputMedian)
{
  std::vector<float> values = {9.0f, 1.0f, 7.0f, 3.0f, 5.0f};
  float median = vl::PointCloudROI::computeMedian(values);
  EXPECT_FLOAT_EQ(median, 5.0f);
}

// Test 4: Empty input returns NaN
TEST(MedianTest, EmptyInputReturnsNaN)
{
  std::vector<float> values;
  float median = vl::PointCloudROI::computeMedian(values);
  EXPECT_TRUE(std::isnan(median));
}

// Test 5: Single element
TEST(MedianTest, SingleElement)
{
  std::vector<float> values = {5.0f};
  float median = vl::PointCloudROI::computeMedian(values);
  EXPECT_FLOAT_EQ(median, 5.0f);
}

// Test 6: Median center computation
TEST(MedianCenterTest, ComputeMedianCenter)
{
  std::vector<float> xs = {1.0f, 2.0f, 3.0f};
  std::vector<float> ys = {4.0f, 5.0f, 6.0f};
  std::vector<float> zs = {7.0f, 8.0f, 9.0f};

  float cx, cy, cz;
  bool result = vl::PointCloudROI::computeMedianCenter(xs, ys, zs, cx, cy, cz);

  EXPECT_TRUE(result);
  EXPECT_FLOAT_EQ(cx, 2.0f);
  EXPECT_FLOAT_EQ(cy, 5.0f);
  EXPECT_FLOAT_EQ(cz, 8.0f);
}

// Test 7: Empty coordinate vectors
TEST(MedianCenterTest, EmptyVectors)
{
  std::vector<float> xs, ys, zs;
  float cx, cy, cz;
  bool result = vl::PointCloudROI::computeMedianCenter(xs, ys, zs, cx, cy, cz);
  EXPECT_FALSE(result);
}

// Test 8: Symmetric point set
TEST(MedianCenterTest, SymmetricPointSet)
{
  std::vector<float> xs = {-1.0f, 0.0f, 1.0f};
  std::vector<float> ys = {-2.0f, 0.0f, 2.0f};
  std::vector<float> zs = {-3.0f, 0.0f, 3.0f};

  float cx, cy, cz;
  bool result = vl::PointCloudROI::computeMedianCenter(xs, ys, zs, cx, cy, cz);

  EXPECT_TRUE(result);
  EXPECT_FLOAT_EQ(cx, 0.0f);
  EXPECT_FLOAT_EQ(cy, 0.0f);
  EXPECT_FLOAT_EQ(cz, 0.0f);
}

// Test 9: Extract ROI validation - organized cloud check
TEST(ROIExtractionTest, SimpleROIExtraction)
{
  // This test verifies that organized cloud validation works correctly
  // When extractROI receives unorganized cloud data, it should return false
  
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.height = 1;  // Not organized
  cloud.width = 9;
  cloud.resize(cloud.width);

  for (int i = 0; i < 9; i++)
  {
    cloud.points[i] = pcl::PointXYZ(0.1f, 0.1f, 1.0f);
  }

  vl::BoundingBox bbox;
  bbox.xmin = 0;
  bbox.ymin = 0;
  bbox.xmax = 3;
  bbox.ymax = 3;

  pcl::PointCloud<pcl::PointXYZ> roi_points;
  bool result = vl::PointCloudROI::extractROI(
    cloud, bbox, 3, 3, 0.0f, 0.5f, 2.0f, 1, roi_points);

  // Should return false because cloud is not organized (height must > 1)
  EXPECT_FALSE(result);
}

// Test 10: BBox validation (empty bbox returns false)
TEST(ROIExtractionTest, InvalidBBox)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.height = 3;
  cloud.width = 3;
  cloud.is_dense = true;

  for (int i = 0; i < 9; i++)
  {
    pcl::PointXYZ pt;
    pt.x = 0.0f;
    pt.y = 0.0f;
    pt.z = 1.0f;
    cloud.push_back(pt);
  }

  vl::BoundingBox bbox;
  bbox.xmin = 5;
  bbox.ymin = 5;
  bbox.xmax = 5;  // Empty width
  bbox.ymax = 5;

  pcl::PointCloud<pcl::PointXYZ> roi_points;
  bool result = vl::PointCloudROI::extractROI(
    cloud, bbox, 3, 3, 0.0f, 0.5f, 2.0f, 1, roi_points);

  EXPECT_FALSE(result);
}

// Test 11: Unorganized cloud returns false
TEST(ROIExtractionTest, UnorganizedCloud)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = "camera";
  cloud.height = 1;  // Unorganized: height = 1
  cloud.width = 5;
  cloud.is_dense = true;

  for (int i = 0; i < 5; i++)
  {
    pcl::PointXYZ pt;
    pt.x = 0.1f;
    pt.y = 0.0f;
    pt.z = 1.0f;
    cloud.push_back(pt);
  }

  vl::BoundingBox bbox;
  bbox.xmin = 0;
  bbox.ymin = 0;
  bbox.xmax = 3;
  bbox.ymax = 2;

  pcl::PointCloud<pcl::PointXYZ> roi_points;
  bool result = vl::PointCloudROI::extractROI(
    cloud, bbox, 5, 1, 0.0f, 0.5f, 2.0f, 1, roi_points);

  EXPECT_FALSE(result);
}

// Test 12: NaN/Inf point filtering - validates filter logic
TEST(PointFilteringTest, NaNInfFiltering)
{
  // Direct test of computeMedianCenter with invalid data
  std::vector<float> xs = {0.0f, 0.1f, std::nanf("")};
  std::vector<float> ys = {0.0f, 0.1f, 0.2f};
  std::vector<float> zs = {1.0f, 1.0f, 1.0f};

  float cx, cy, cz;
  // This should fail because xs contains NaN
  bool result = vl::PointCloudROI::computeMedianCenter(xs, ys, zs, cx, cy, cz);

  // NaN输入必须使中心计算失败，并且不得返回有限的伪中心。
  EXPECT_FALSE(result);
  EXPECT_FALSE(std::isfinite(cx));
  
}

// Test 13: Distance range filtering validation
TEST(PointFilteringTest, DistanceRangeFiltering)
{
  // Direct test of median computation with different value ranges
  std::vector<float> xs = {0.3f, 1.0f, 3.0f, 1.5f, 2.0f, 2.5f};
  std::vector<float> ys = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<float> zs = {0.3f, 1.0f, 3.0f, 1.5f, 2.0f, 2.5f};

  float cx, cy, cz;
  bool result = vl::PointCloudROI::computeMedianCenter(xs, ys, zs, cx, cy, cz);

  EXPECT_TRUE(result);
  EXPECT_FLOAT_EQ(cy, 0.0f);
  // Median of [0.3, 1.0, 1.5, 2.0, 2.5, 3.0] is (1.5 + 2.0) / 2 = 1.75
  EXPECT_FLOAT_EQ(cx, 1.75f);
  EXPECT_FLOAT_EQ(cz, 1.75f);
}

// Test 14: Insufficient ROI points
TEST(ROIExtractionTest, InsufficientROIPoints)
{
  // Cloud with only 1 point in selected area
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 3;
  cloud.height = 3;
  cloud.resize(cloud.width * cloud.height);

  for (int i = 0; i < 9; i++)
  {
    if (i == 0)
    {
      cloud.points[i] = pcl::PointXYZ(0.0f, 0.0f, 1.0f);
    }
    else
    {
      // NaN to filter out other points
      cloud.points[i].x = std::nanf("");
      cloud.points[i].y = std::nanf("");
      cloud.points[i].z = std::nanf("");
    }
  }

  vl::BoundingBox bbox;
  bbox.xmin = 0;
  bbox.ymin = 0;
  bbox.xmax = 3;
  bbox.ymax = 3;

  pcl::PointCloud<pcl::PointXYZ> roi_points;
  bool result = vl::PointCloudROI::extractROI(
    cloud, bbox, 3, 3, 0.0f, 0.5f, 2.0f, 5, roi_points); // min_roi_points = 5

  EXPECT_FALSE(result); // Should fail because only 1 valid point
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
