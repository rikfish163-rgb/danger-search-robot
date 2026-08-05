#pragma once

#include <ros/time.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace vision_localization
{

/**
 * @brief 二维目标检测框。
 *
 * 坐标约定：
 * - xmin、ymin：左上角像素；
 * - xmax、ymax：右下角像素；
 * - 坐标属于原始RGB图像；
 * - 置信度范围应为[0, 1]。
 */
struct BoundingBox2D
{
  int xmin = 0;
  int ymin = 0;
  int xmax = 0;
  int ymax = 0;

  std::string class_name;
  float confidence = 0.0F;
};

/**
 * @brief 一帧二维检测结果。
 *
 * stamp必须保留原始RGB图像采集时间，不能替换成ros::Time::now()。
 */
struct DetectionFrame
{
  ros::Time stamp;
  std::string source_frame;

  std::uint32_t image_width = 0U;
  std::uint32_t image_height = 0U;

  std::vector<BoundingBox2D> detections;
};

/**
 * @brief 带坐标系和采集时间的三维点。
 */
struct Point3D
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  std::string frame_id;
  ros::Time stamp;
};

/**
 * @brief DangerObservation消息中status_code字段的约定。
 *
 * valid字段表示观测是否可用，status_code表示无效原因，
 * 便于下游节点诊断数据链路。
 */
enum class DangerObservationStatusCode : std::uint8_t
{
  // 完成球体拟合、质量检查和TF转换后的有效观测。
  kValid = 0U,

  // 当前RGB帧没有检测到目标。
  kNoDetection = 1U,

  // 检测到了目标，但点云ROI提取失败。
  kRoiExtractionFailed = 2U,

  // ROI存在，但球心或候选点估计失败。
  kCoordinateEstimationFailed = 3U,

  // 相机坐标存在，但无法转换到目标坐标系。
  kTransformFailed = 4U,

  // 完成拟合，但没有通过半径、RMSE或内点率质量门限。
  kQualityRejected = 5U,

  // 未经球体拟合和质量检查的候选点。
  kUnverifiedEstimate = 6U,

  // 未分类的内部错误。
  kInternalError = 255U
};

/**
 * @brief 一条通过完整质量检查的危险源观测数据。
 *
 * 该结构只用于发布valid=true的DangerObservation。
 *
 * 默认浮点值使用NaN，使OutputAdapter能够拒绝未完整填充的数据。
 */
struct DangerObservationData
{
  Point3D center;

  float fitted_radius =
      std::numeric_limits<float>::quiet_NaN();

  float sphere_rmse =
      std::numeric_limits<float>::quiet_NaN();

  float inlier_ratio =
      std::numeric_limits<float>::quiet_NaN();

  std::uint32_t roi_point_count = 0U;
  std::uint32_t inlier_count = 0U;

  float detector_confidence =
      std::numeric_limits<float>::quiet_NaN();
};

enum class PipelineStatus
{
  kSuccess,
  kNotConfigured,
  kNoInput,
  kInvalidTimestamp,
  kInvalidFrame,
  kInvalidBoundingBox,
  kCloudNotOrganized,
  kDimensionMismatch,
  kInsufficientPoints,
  kEstimationNotImplemented,
  kSphereFitFailed,
  kQualityRejected,
  kTransformUnavailable,
  kOutputNotConfigured,
  kInternalError
};

struct PipelineResult
{
  PipelineStatus status = PipelineStatus::kNotConfigured;
  std::string message;
};

}  // namespace vision_localization
