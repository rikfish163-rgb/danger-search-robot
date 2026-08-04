#include "vision_localization/input_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <boost/bind/bind.hpp>

namespace vision_localization
{

InputAdapter::InputAdapter(
    ros::NodeHandle& node_handle,
    ros::NodeHandle& private_node_handle)
  : node_handle_(node_handle),
    private_node_handle_(private_node_handle)
{
}

bool InputAdapter::initialize(
    const LocalizationConfig& config)
{
  configured_ = false;

  detection_subscriber_.reset();
  pointcloud_subscriber_.reset();
  synchronizer_.reset();

  if (config.detection_topic.empty())
  {
    ROS_ERROR(
        "InputAdapter: detection_topic is empty.");
    return false;
  }

  if (config.pointcloud_topic.empty())
  {
    ROS_ERROR(
        "InputAdapter: pointcloud_topic is empty.");
    return false;
  }

  if (config.subscriber_queue_size <= 0)
  {
    ROS_ERROR(
        "InputAdapter: subscriber_queue_size "
        "must be positive.");
    return false;
  }

  if (config.sync_queue_size <= 0)
  {
    ROS_ERROR(
        "InputAdapter: sync_queue_size "
        "must be positive.");
    return false;
  }

  if (config.sync_tolerance <= 0.0)
  {
    ROS_ERROR(
        "InputAdapter: sync_tolerance "
        "must be positive.");
    return false;
  }

  if (config.target_class.empty())
  {
    ROS_ERROR(
        "InputAdapter: target_class is empty.");
    return false;
  }

  target_class_ = config.target_class;
  sync_tolerance_ = config.sync_tolerance;

  detection_subscriber_ =
      std::make_unique<DetectionSubscriber>(
          node_handle_,
          config.detection_topic,
          static_cast<uint32_t>(
              config.subscriber_queue_size));

  pointcloud_subscriber_ =
      std::make_unique<PointCloudSubscriber>(
          node_handle_,
          config.pointcloud_topic,
          static_cast<uint32_t>(
              config.subscriber_queue_size));

  SyncPolicy synchronization_policy(
      static_cast<uint32_t>(
          config.sync_queue_size));

  synchronization_policy.setMaxIntervalDuration(
      ros::Duration(config.sync_tolerance));

  synchronizer_ =
      std::make_unique<Synchronizer>(
          std::move(synchronization_policy),
          *detection_subscriber_,
          *pointcloud_subscriber_);

  synchronizer_->registerCallback(
      boost::bind(
          &InputAdapter::synchronizedCallback,
          this,
          boost::placeholders::_1,
          boost::placeholders::_2));

  configured_ = true;

  ROS_INFO(
      "InputAdapter ready | detection=%s | "
      "pointcloud=%s | subscriber_queue=%d | "
      "sync_queue=%d | tolerance=%.3f s | "
      "target_class=%s",
      config.detection_topic.c_str(),
      config.pointcloud_topic.c_str(),
      config.subscriber_queue_size,
      config.sync_queue_size,
      config.sync_tolerance,
      target_class_.c_str());

  return true;
}

void InputAdapter::setFrameCallback(
    const FrameCallback& callback)
{
  frame_callback_ = callback;
}

bool InputAdapter::isConfigured() const
{
  return configured_;
}

void InputAdapter::synchronizedCallback(
    const quadruped_vision::DetectionArrayConstPtr&
        detection_message,
    const sensor_msgs::PointCloud2ConstPtr&
        pointcloud_message)
{
  if (!configured_)
  {
    ROS_ERROR_THROTTLE(
        5.0,
        "InputAdapter received data before configuration.");
    return;
  }

  if (!detection_message || !pointcloud_message)
  {
    ROS_ERROR_THROTTLE(
        5.0,
        "InputAdapter received a null message pointer.");
    return;
  }

  if (detection_message->header.stamp.isZero())
  {
    ROS_WARN_THROTTLE(
        5.0,
        "DetectionArray has a zero timestamp.");
    return;
  }

  if (pointcloud_message->header.stamp.isZero())
  {
    ROS_WARN_THROTTLE(
        5.0,
        "PointCloud2 has a zero timestamp.");
    return;
  }

  if (pointcloud_message->header.frame_id.empty())
  {
    ROS_WARN_THROTTLE(
        5.0,
        "PointCloud2 has an empty frame_id.");
    return;
  }

  const double timestamp_difference =
      std::fabs(
          (detection_message->header.stamp -
           pointcloud_message->header.stamp)
              .toSec());

  if (timestamp_difference > sync_tolerance_)
  {
    ROS_WARN_THROTTLE(
        5.0,
        "Synchronized messages exceed tolerance: "
        "difference=%.6f s, tolerance=%.6f s",
        timestamp_difference,
        sync_tolerance_);
    return;
  }

  DetectionFrame detection_frame;

  if (!convertDetectionFrame(
          *detection_message,
          *pointcloud_message,
          detection_frame))
  {
    return;
  }

  if (!frame_callback_)
  {
    ROS_ERROR_THROTTLE(
        5.0,
        "InputAdapter frame callback is not set.");
    return;
  }

  frame_callback_(
      detection_frame,
      pointcloud_message);
}

bool InputAdapter::convertDetectionFrame(
    const quadruped_vision::DetectionArray&
        detection_message,
    const sensor_msgs::PointCloud2&
        pointcloud_message,
    DetectionFrame& output_frame) const
{
  if (detection_message.image_width == 0U ||
      detection_message.image_height == 0U)
  {
    ROS_WARN_THROTTLE(
        5.0,
        "DetectionArray contains an invalid image size.");
    return false;
  }

  output_frame = DetectionFrame();

  // 候选点来源于点云，因此坐标系必须使用点云坐标系。
  output_frame.source_frame =
      pointcloud_message.header.frame_id;

  // TF 查询必须使用原始 RGB 图像采集时刻。
  output_frame.stamp =
      detection_message.header.stamp;

  output_frame.image_width =
      detection_message.image_width;

  output_frame.image_height =
      detection_message.image_height;

  const double image_width =
      static_cast<double>(
          detection_message.image_width);

  const double image_height =
      static_cast<double>(
          detection_message.image_height);

  for (const auto& source_detection :
       detection_message.detections)
  {
    if (source_detection.class_name != target_class_)
    {
      continue;
    }

    if (!std::isfinite(source_detection.confidence) ||
        !std::isfinite(source_detection.xmin) ||
        !std::isfinite(source_detection.ymin) ||
        !std::isfinite(source_detection.xmax) ||
        !std::isfinite(source_detection.ymax))
    {
      ROS_WARN_THROTTLE(
          5.0,
          "Ignoring detection containing NaN or Inf.");
      continue;
    }

    const double xmin = std::max(
        0.0,
        std::min(
            static_cast<double>(source_detection.xmin),
            image_width - 1.0));

    const double ymin = std::max(
        0.0,
        std::min(
            static_cast<double>(source_detection.ymin),
            image_height - 1.0));

    // xmax/ymax 使用右侧和下侧开区间边界，
    // 因而可以等于 image_width/image_height。
    const double xmax = std::max(
        0.0,
        std::min(
            static_cast<double>(source_detection.xmax),
            image_width));

    const double ymax = std::max(
        0.0,
        std::min(
            static_cast<double>(source_detection.ymax),
            image_height));

    BoundingBox2D converted_detection;

    converted_detection.xmin =
        static_cast<int>(std::floor(xmin));

    converted_detection.ymin =
        static_cast<int>(std::floor(ymin));

    converted_detection.xmax =
        static_cast<int>(std::ceil(xmax));

    converted_detection.ymax =
        static_cast<int>(std::ceil(ymax));

    converted_detection.class_name =
        source_detection.class_name;

    converted_detection.confidence =
        source_detection.confidence;

    if (converted_detection.xmax <=
            converted_detection.xmin ||
        converted_detection.ymax <=
            converted_detection.ymin)
    {
      ROS_WARN_THROTTLE(
          5.0,
          "Ignoring an empty detection bounding box.");
      continue;
    }

    output_frame.detections.push_back(
        converted_detection);
  }

  return true;
}

}  // namespace vision_localization
