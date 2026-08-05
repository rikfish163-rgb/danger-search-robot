#pragma once

#include <functional>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <quadruped_vision/DetectionArray.h>

#include "vision_localization/localization_config.hpp"
#include "vision_localization/pipeline_types.hpp"

namespace vision_localization
{

class InputAdapter
{
public:
  using FrameCallback = std::function<void(
      const DetectionFrame&,
      const sensor_msgs::PointCloud2ConstPtr&)>;

  InputAdapter(
      ros::NodeHandle& node_handle,
      ros::NodeHandle& private_node_handle);

  bool initialize(const LocalizationConfig& config);

  void setFrameCallback(const FrameCallback& callback);

  bool isConfigured() const;

private:
  using DetectionMessage =
      quadruped_vision::DetectionArray;

  using DetectionSubscriber =
      message_filters::Subscriber<DetectionMessage>;

  using PointCloudSubscriber =
      message_filters::Subscriber<sensor_msgs::PointCloud2>;

  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<
          DetectionMessage,
          sensor_msgs::PointCloud2>;

  using Synchronizer =
      message_filters::Synchronizer<SyncPolicy>;

  void synchronizedCallback(
      const quadruped_vision::DetectionArrayConstPtr&
          detection_message,
      const sensor_msgs::PointCloud2ConstPtr&
          pointcloud_message);

  bool convertDetectionFrame(
      const quadruped_vision::DetectionArray&
          detection_message,
      const sensor_msgs::PointCloud2&
          pointcloud_message,
      DetectionFrame& output_frame) const;

  ros::NodeHandle node_handle_;
  ros::NodeHandle private_node_handle_;

  FrameCallback frame_callback_;

  std::unique_ptr<DetectionSubscriber>
      detection_subscriber_;

  std::unique_ptr<PointCloudSubscriber>
      pointcloud_subscriber_;

  std::unique_ptr<Synchronizer>
      synchronizer_;

  std::string target_class_;
  double sync_tolerance_ = 0.0;

  bool configured_ = false;
};

}  // namespace vision_localization
