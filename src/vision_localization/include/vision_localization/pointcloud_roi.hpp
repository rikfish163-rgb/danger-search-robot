#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include "vision_localization/localization_config.hpp"
#include "vision_localization/pipeline_types.hpp"

namespace vision_localization
{

/**
 * ROI 内部使用的整数检测框。
 *
 * 像素范围采用半开区间：
 * [xmin, xmax) × [ymin, ymax)
 */
struct BoundingBox
{
  std::string class_name;
  float confidence = 0.0F;

  int xmin = 0;
  int ymin = 0;
  int xmax = 0;
  int ymax = 0;
};

struct RoiExtractionResult
{
  PipelineStatus status =
      PipelineStatus::kNotConfigured;

  std::string message;

  pcl::PointCloud<pcl::PointXYZ> points;
};

class PointCloudROI
{
public:
  PointCloudROI() = default;
  ~PointCloudROI() = default;

  /**
   * 保存并验证 ROI 提取参数。
   */
  bool initialize(
      const LocalizationConfig& config);

  /**
   * 从有组织点云中提取检测框对应的有效三维点。
   *
   * 必要前提：
   * 1. 点云必须是有组织点云；
   * 2. 点云宽高必须等于检测图像宽高；
   * 3. 点云像素必须和 RGB 像素完成对齐；
   * 4. 检测框采用原始 RGB 图像像素坐标；
   * 5. 点云必须包含 x/y/z 字段。
   */
  RoiExtractionResult extract(
      const sensor_msgs::PointCloud2& cloud_message,
      const DetectionFrame& detection_frame,
      const BoundingBox2D& bounding_box) const;

  /**
   * 提取 ROI 的纯 PCL 辅助接口。
   */
  static bool extractROI(
      const pcl::PointCloud<pcl::PointXYZ>& cloud,
      const BoundingBox& bbox,
      std::uint32_t image_width,
      std::uint32_t image_height,
      float bbox_shrink_ratio,
      float min_range,
      float max_range,
      int min_roi_points,
      pcl::PointCloud<pcl::PointXYZ>& roi_points);

  /**
   * 计算一维浮点数组中值。
   */
  static float computeMedian(
      std::vector<float> values);

  /**
   * 分别计算 x、y、z 三个分量的中值。
   */
  static bool computeMedianCenter(
      const std::vector<float>& xs,
      const std::vector<float>& ys,
      const std::vector<float>& zs,
      float& center_x,
      float& center_y,
      float& center_z);

private:
  static bool clipBoundingBox(
      const BoundingBox& bbox,
      std::uint32_t image_width,
      std::uint32_t image_height,
      BoundingBox& clipped_bbox);

  static bool shrinkBoundingBox(
      const BoundingBox& bbox,
      float shrink_ratio,
      BoundingBox& shrunk_bbox);

  PipelineResult validateCloudMetadata(
      const sensor_msgs::PointCloud2& cloud_message,
      const DetectionFrame& detection_frame) const;

  PipelineResult validateBoundingBox(
      const BoundingBox2D& bounding_box,
      std::uint32_t image_width,
      std::uint32_t image_height) const;

  float bbox_shrink_ratio_ = 0.0F;
  float min_range_ = 0.0F;
  float max_range_ = 0.0F;
  int min_roi_points_ = 0;

  bool configured_ = false;
};

}  // namespace vision_localization