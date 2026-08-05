#include "vision_localization/pointcloud_roi.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointField.h>

namespace vision_localization
{

bool PointCloudROI::initialize(
    const LocalizationConfig& config)
{
  // 初始化失败时保持未配置状态，后续ROI提取会返回错误。
  configured_ = false;

  if (!std::isfinite(config.bbox_shrink_ratio) ||
      config.bbox_shrink_ratio < 0.0 ||
      config.bbox_shrink_ratio >= 0.5)
  {
    return false;
  }

  if (!std::isfinite(config.min_range) ||
      config.min_range < 0.0)
  {
    return false;
  }

  if (!std::isfinite(config.max_range) ||
      config.max_range <= config.min_range)
  {
    return false;
  }

  if (config.min_roi_points <= 0)
  {
    return false;
  }

  bbox_shrink_ratio_ =
      static_cast<float>(config.bbox_shrink_ratio);

  min_range_ =
      static_cast<float>(config.min_range);

  max_range_ =
      static_cast<float>(config.max_range);

  min_roi_points_ =
      config.min_roi_points;

  configured_ = true;
  return true;
}

PipelineResult PointCloudROI::validateCloudMetadata(
    const sensor_msgs::PointCloud2& cloud_message,
    const DetectionFrame& detection_frame) const
{
  PipelineResult result;

  if (cloud_message.header.frame_id.empty())
  {
    result.status = PipelineStatus::kInvalidFrame;
    result.message = "点云消息缺少header.frame_id。";
    return result;
  }

  if (detection_frame.source_frame.empty())
  {
    result.status = PipelineStatus::kInvalidFrame;
    result.message =
        "DetectionFrame缺少点云source_frame。";
    return result;
  }

  if (detection_frame.source_frame !=
      cloud_message.header.frame_id)
  {
    result.status = PipelineStatus::kInvalidFrame;
    result.message =
        "DetectionFrame.source_frame与点云frame_id不一致。";
    return result;
  }

  if (cloud_message.header.stamp.isZero())
  {
    result.status = PipelineStatus::kInvalidTimestamp;
    result.message = "点云消息缺少有效时间戳。";
    return result;
  }

  if (detection_frame.stamp.isZero())
  {
    result.status = PipelineStatus::kInvalidTimestamp;
    result.message =
        "DetectionFrame缺少原始RGB时间戳。";
    return result;
  }

  if (detection_frame.image_width == 0U ||
      detection_frame.image_height == 0U)
  {
    result.status = PipelineStatus::kDimensionMismatch;
    result.message =
        "DetectionFrame的图像宽度或高度为0。";
    return result;
  }

  // BoundingBox2D使用int存储像素坐标，因此拒绝无法安全
  // 转换为int的异常图像尺寸。
  if (detection_frame.image_width >
          static_cast<std::uint32_t>(
              std::numeric_limits<int>::max()) ||
      detection_frame.image_height >
          static_cast<std::uint32_t>(
              std::numeric_limits<int>::max()))
  {
    result.status = PipelineStatus::kDimensionMismatch;
    result.message =
        "检测图像尺寸超过整数像素坐标的安全范围。";
    return result;
  }

  // 只有有组织点云才能通过像素索引直接提取ROI。
  if (cloud_message.width == 0U ||
      cloud_message.height <= 1U)
  {
    result.status = PipelineStatus::kCloudNotOrganized;
    result.message =
        "输入点云不是有效的有组织点云。";
    return result;
  }

  // 直接像素索引模式依赖像素一一对应关系，因此点云尺寸必须与
  // 原始RGB检测图像完全相同。
  if (cloud_message.width !=
          detection_frame.image_width ||
      cloud_message.height !=
          detection_frame.image_height)
  {
    result.status = PipelineStatus::kDimensionMismatch;
    result.message =
        "点云宽高与原始RGB图像宽高不一致，"
        "不能直接使用检测框索引点云。";
    return result;
  }

  if (cloud_message.point_step == 0U ||
      cloud_message.row_step == 0U)
  {
    result.status = PipelineStatus::kInternalError;
    result.message =
        "点云point_step或row_step无效。";
    return result;
  }

  const std::uint64_t minimum_row_bytes =
      static_cast<std::uint64_t>(
          cloud_message.point_step) *
      static_cast<std::uint64_t>(
          cloud_message.width);

  if (static_cast<std::uint64_t>(
          cloud_message.row_step) <
      minimum_row_bytes)
  {
    result.status = PipelineStatus::kInternalError;
    result.message =
        "点云row_step小于单行点数据所需字节数。";
    return result;
  }

  const std::uint64_t minimum_data_bytes =
      static_cast<std::uint64_t>(
          cloud_message.row_step) *
      static_cast<std::uint64_t>(
          cloud_message.height);

  if (static_cast<std::uint64_t>(
          cloud_message.data.size()) <
      minimum_data_bytes)
  {
    result.status = PipelineStatus::kInternalError;
    result.message =
        "点云data长度小于元数据声明的长度。";
    return result;
  }

  bool has_valid_x = false;
  bool has_valid_y = false;
  bool has_valid_z = false;

  for (const auto& field : cloud_message.fields)
  {
    const bool field_layout_valid =
        field.count >= 1U &&
        field.datatype ==
            sensor_msgs::PointField::FLOAT32 &&
        static_cast<std::uint64_t>(field.offset) +
                sizeof(float) <=
            static_cast<std::uint64_t>(
                cloud_message.point_step);

    if (!field_layout_valid)
    {
      continue;
    }

    if (field.name == "x")
    {
      has_valid_x = true;
    }
    else if (field.name == "y")
    {
      has_valid_y = true;
    }
    else if (field.name == "z")
    {
      has_valid_z = true;
    }
  }

  if (!has_valid_x ||
      !has_valid_y ||
      !has_valid_z)
  {
    result.status = PipelineStatus::kInternalError;
    result.message =
        "点云缺少有效的FLOAT32类型x/y/z字段。";
    return result;
  }

  result.status = PipelineStatus::kSuccess;
  result.message = "点云元数据校验成功。";
  return result;
}

PipelineResult PointCloudROI::validateBoundingBox(
    const BoundingBox2D& bounding_box,
    std::uint32_t image_width,
    std::uint32_t image_height) const
{
  PipelineResult result;

  if (image_width == 0U ||
      image_height == 0U)
  {
    result.status = PipelineStatus::kDimensionMismatch;
    result.message = "图像尺寸无效。";
    return result;
  }

  if (bounding_box.xmax <= bounding_box.xmin ||
      bounding_box.ymax <= bounding_box.ymin)
  {
    result.status = PipelineStatus::kInvalidBoundingBox;
    result.message =
        "二维检测框宽度或高度小于等于0。";
    return result;
  }

  const int width =
      static_cast<int>(image_width);

  const int height =
      static_cast<int>(image_height);

  // 允许检测框部分越界，后续会执行裁剪；
  // 但完全位于图像外部的检测框必须拒绝。
  if (bounding_box.xmax <= 0 ||
      bounding_box.ymax <= 0 ||
      bounding_box.xmin >= width ||
      bounding_box.ymin >= height)
  {
    result.status = PipelineStatus::kInvalidBoundingBox;
    result.message =
        "二维检测框完全位于图像范围之外。";
    return result;
  }

  result.status = PipelineStatus::kSuccess;
  result.message = "二维检测框校验成功。";
  return result;
}

RoiExtractionResult PointCloudROI::extract(
    const sensor_msgs::PointCloud2& cloud_message,
    const DetectionFrame& detection_frame,
    const BoundingBox2D& bounding_box) const
{
  RoiExtractionResult result;

  if (!configured_)
  {
    result.status = PipelineStatus::kNotConfigured;
    result.message =
        "PointCloudROI尚未完成参数初始化。";
    return result;
  }

  const PipelineResult cloud_validation =
      validateCloudMetadata(
          cloud_message,
          detection_frame);

  if (cloud_validation.status !=
      PipelineStatus::kSuccess)
  {
    result.status = cloud_validation.status;
    result.message = cloud_validation.message;
    return result;
  }

  const PipelineResult bbox_validation =
      validateBoundingBox(
          bounding_box,
          detection_frame.image_width,
          detection_frame.image_height);

  if (bbox_validation.status !=
      PipelineStatus::kSuccess)
  {
    result.status = bbox_validation.status;
    result.message = bbox_validation.message;
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ> organized_cloud;

  try
  {
    pcl::fromROSMsg(
        cloud_message,
        organized_cloud);
  }
  catch (const std::exception& exception)
  {
    result.status = PipelineStatus::kInternalError;
    result.message =
        std::string("PointCloud2转换为PCL点云失败：") +
        exception.what();
    return result;
  }
  catch (...)
  {
    result.status = PipelineStatus::kInternalError;
    result.message =
        "PointCloud2转换为PCL点云时发生未知异常。";
    return result;
  }

  if (!organized_cloud.isOrganized() ||
      organized_cloud.height <= 1U)
  {
    result.status = PipelineStatus::kCloudNotOrganized;
    result.message =
        "PointCloud2转换后不是有组织PCL点云。";
    return result;
  }

  if (organized_cloud.width !=
          detection_frame.image_width ||
      organized_cloud.height !=
          detection_frame.image_height)
  {
    result.status = PipelineStatus::kDimensionMismatch;
    result.message =
        "PCL转换后的点云尺寸与RGB图像尺寸不一致。";
    return result;
  }

  const std::size_t expected_point_count =
      static_cast<std::size_t>(
          organized_cloud.width) *
      static_cast<std::size_t>(
          organized_cloud.height);

  if (organized_cloud.points.size() !=
      expected_point_count)
  {
    result.status = PipelineStatus::kInternalError;
    result.message =
        "有组织点云实际点数与width×height不一致。";
    return result;
  }

  BoundingBox roi_box;
  roi_box.class_name = bounding_box.class_name;
  roi_box.confidence = bounding_box.confidence;
  roi_box.xmin = bounding_box.xmin;
  roi_box.ymin = bounding_box.ymin;
  roi_box.xmax = bounding_box.xmax;
  roi_box.ymax = bounding_box.ymax;

  pcl::PointCloud<pcl::PointXYZ> roi_points;

  if (!extractROI(
          organized_cloud,
          roi_box,
          detection_frame.image_width,
          detection_frame.image_height,
          bbox_shrink_ratio_,
          min_range_,
          max_range_,
          min_roi_points_,
          roi_points))
  {
    result.status = PipelineStatus::kInsufficientPoints;
    result.message =
        "ROI裁剪、内缩或有效点过滤后点数不足。";
    return result;
  }

  result.points = std::move(roi_points);
  result.status = PipelineStatus::kSuccess;
  result.message =
      "ROI点云提取成功，有效点数：" +
      std::to_string(result.points.size()) + "。";

  return result;
}

bool PointCloudROI::clipBoundingBox(
    const BoundingBox& bbox,
    std::uint32_t image_width,
    std::uint32_t image_height,
    BoundingBox& clipped_bbox)
{
  if (image_width == 0U ||
      image_height == 0U ||
      image_width >
          static_cast<std::uint32_t>(
              std::numeric_limits<int>::max()) ||
      image_height >
          static_cast<std::uint32_t>(
              std::numeric_limits<int>::max()))
  {
    return false;
  }

  const int width =
      static_cast<int>(image_width);

  const int height =
      static_cast<int>(image_height);

  clipped_bbox = bbox;

  clipped_bbox.xmin =
      std::max(0, std::min(width, bbox.xmin));

  clipped_bbox.ymin =
      std::max(0, std::min(height, bbox.ymin));

  clipped_bbox.xmax =
      std::max(0, std::min(width, bbox.xmax));

  clipped_bbox.ymax =
      std::max(0, std::min(height, bbox.ymax));

  return clipped_bbox.xmax >
             clipped_bbox.xmin &&
         clipped_bbox.ymax >
             clipped_bbox.ymin;
}

bool PointCloudROI::shrinkBoundingBox(
    const BoundingBox& bbox,
    float shrink_ratio,
    BoundingBox& shrunk_bbox)
{
  if (!std::isfinite(shrink_ratio) ||
      shrink_ratio < 0.0F ||
      shrink_ratio >= 0.5F)
  {
    return false;
  }

  const int width =
      bbox.xmax - bbox.xmin;

  const int height =
      bbox.ymax - bbox.ymin;

  if (width <= 0 || height <= 0)
  {
    return false;
  }

  // shrink_ratio表示每一侧向内收缩的比例。
  // 例如0.10表示左右各收缩原宽度的10%。
  const int dx =
      static_cast<int>(
          std::round(
              static_cast<float>(width) *
              shrink_ratio));

  const int dy =
      static_cast<int>(
          std::round(
              static_cast<float>(height) *
              shrink_ratio));

  shrunk_bbox = bbox;
  shrunk_bbox.xmin += dx;
  shrunk_bbox.ymin += dy;
  shrunk_bbox.xmax -= dx;
  shrunk_bbox.ymax -= dy;

  return shrunk_bbox.xmax >
             shrunk_bbox.xmin &&
         shrunk_bbox.ymax >
             shrunk_bbox.ymin;
}

bool PointCloudROI::extractROI(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const BoundingBox& bbox,
    std::uint32_t image_width,
    std::uint32_t image_height,
    float bbox_shrink_ratio,
    float min_range,
    float max_range,
    int min_roi_points,
    pcl::PointCloud<pcl::PointXYZ>& roi_points)
{
  // 不允许调用失败后遗留上一帧ROI。
  roi_points.clear();

  if (!cloud.isOrganized() ||
      cloud.width == 0U ||
      cloud.height <= 1U)
  {
    return false;
  }

  if (image_width == 0U ||
      image_height == 0U ||
      cloud.width != image_width ||
      cloud.height != image_height)
  {
    return false;
  }

  const std::size_t expected_point_count =
      static_cast<std::size_t>(cloud.width) *
      static_cast<std::size_t>(cloud.height);

  if (cloud.points.size() != expected_point_count)
  {
    return false;
  }

  if (!std::isfinite(bbox_shrink_ratio) ||
      bbox_shrink_ratio < 0.0F ||
      bbox_shrink_ratio >= 0.5F)
  {
    return false;
  }

  if (!std::isfinite(min_range) ||
      !std::isfinite(max_range) ||
      min_range < 0.0F ||
      max_range <= min_range ||
      min_roi_points <= 0)
  {
    return false;
  }

  BoundingBox clipped_bbox;

  if (!clipBoundingBox(
          bbox,
          image_width,
          image_height,
          clipped_bbox))
  {
    return false;
  }

  BoundingBox shrunk_bbox;

  if (!shrinkBoundingBox(
          clipped_bbox,
          bbox_shrink_ratio,
          shrunk_bbox))
  {
    return false;
  }

  const int roi_width =
      shrunk_bbox.xmax -
      shrunk_bbox.xmin;

  const int roi_height =
      shrunk_bbox.ymax -
      shrunk_bbox.ymin;

  if (roi_width <= 0 || roi_height <= 0)
  {
    return false;
  }

  const std::size_t approximate_roi_area =
      static_cast<std::size_t>(roi_width) *
      static_cast<std::size_t>(roi_height);

  roi_points.reserve(approximate_roi_area);
  roi_points.header = cloud.header;
  roi_points.sensor_origin_ = cloud.sensor_origin_;
  roi_points.sensor_orientation_ =
      cloud.sensor_orientation_;

  const double minimum_range_squared =
      static_cast<double>(min_range) *
      static_cast<double>(min_range);

  const double maximum_range_squared =
      static_cast<double>(max_range) *
      static_cast<double>(max_range);

  for (int y = shrunk_bbox.ymin;
       y < shrunk_bbox.ymax;
       ++y)
  {
    for (int x = shrunk_bbox.xmin;
         x < shrunk_bbox.xmax;
         ++x)
    {
      const std::size_t index =
          static_cast<std::size_t>(y) *
              static_cast<std::size_t>(
                  cloud.width) +
          static_cast<std::size_t>(x);

      if (index >= cloud.points.size())
      {
        continue;
      }

      const pcl::PointXYZ& point =
          cloud.points[index];

      if (!std::isfinite(point.x) ||
          !std::isfinite(point.y) ||
          !std::isfinite(point.z))
      {
        continue;
      }

      const double range_squared =
          static_cast<double>(point.x) *
              static_cast<double>(point.x) +
          static_cast<double>(point.y) *
              static_cast<double>(point.y) +
          static_cast<double>(point.z) *
              static_cast<double>(point.z);

      if (!std::isfinite(range_squared) ||
          range_squared < minimum_range_squared ||
          range_squared > maximum_range_squared)
      {
        continue;
      }

      roi_points.push_back(point);
    }
  }

  if (roi_points.size() <
      static_cast<std::size_t>(
          min_roi_points))
  {
    roi_points.clear();
    return false;
  }

  roi_points.width =
      static_cast<std::uint32_t>(
          roi_points.points.size());

  roi_points.height = 1U;

  // 所有非有限点已经被剔除。
  roi_points.is_dense = true;

  return true;
}

float PointCloudROI::computeMedian(
    std::vector<float> values)
{
  if (values.empty())
  {
    return std::numeric_limits<float>::quiet_NaN();
  }

  for (const float value : values)
  {
    if (!std::isfinite(value))
    {
      return std::numeric_limits<float>::quiet_NaN();
    }
  }

  const std::size_t middle =
      values.size() / 2U;

  std::nth_element(
      values.begin(),
      values.begin() + middle,
      values.end());

  const float upper_middle =
      values[middle];

  if ((values.size() % 2U) != 0U)
  {
    return upper_middle;
  }

  // nth_element之后，middle之前的元素均不大于
  // upper_middle；其中最大值即下中位数。
  const float lower_middle =
      *std::max_element(
          values.begin(),
          values.begin() + middle);

  return static_cast<float>(
      (static_cast<double>(lower_middle) +
       static_cast<double>(upper_middle)) /
      2.0);
}

bool PointCloudROI::computeMedianCenter(
    const std::vector<float>& xs,
    const std::vector<float>& ys,
    const std::vector<float>& zs,
    float& center_x,
    float& center_y,
    float& center_z)
{
  if (xs.empty() ||
      xs.size() != ys.size() ||
      xs.size() != zs.size())
  {
    return false;
  }

  center_x = computeMedian(xs);
  center_y = computeMedian(ys);
  center_z = computeMedian(zs);

  return std::isfinite(center_x) &&
         std::isfinite(center_y) &&
         std::isfinite(center_z);
}

}  // namespace vision_localization
