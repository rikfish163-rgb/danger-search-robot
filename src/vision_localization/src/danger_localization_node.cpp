#include <cstdlib>
#include <limits>

#include <ros/ros.h>

#include "vision_localization/danger_localization_node.hpp"

namespace vision_localization
{

DangerLocalizationNode::DangerLocalizationNode(
    ros::NodeHandle& node_handle,
    ros::NodeHandle& private_node_handle)
  : nh_(node_handle),
    private_nh_(private_node_handle),
    input_adapter_(node_handle, private_node_handle),
    roi_extractor_(),
    coordinate_estimator_(),
    tf_transformer_(node_handle),
    output_adapter_(node_handle, private_node_handle)
{
}

bool DangerLocalizationNode::initialize()
{
  if (!loadParameters(
          nh_,
          private_nh_,
          config_))
  {
    ROS_FATAL(
        "LocalizationConfig加载失败。");
    return false;
  }

  const PipelineResult validation_result =
      validateConfig(config_);

  if (validation_result.status !=
      PipelineStatus::kSuccess)
  {
    ROS_FATAL(
        "参数校验失败：%s",
        validation_result.message.c_str());
    return false;
  }

  if (!roi_extractor_.initialize(config_))
  {
    ROS_FATAL(
        "PointCloudROI参数初始化失败。");
    return false;
  }

  if (!coordinate_estimator_.initialize(config_))
  {
    ROS_FATAL(
        "CoordinateEstimator参数初始化失败。");
    return false;
  }

  if (!output_adapter_.initialize(config_))
  {
    ROS_FATAL(
        "OutputAdapter初始化失败。");
    return false;
  }

  // 在建立实际订阅前注册主处理回调。
  input_adapter_.setFrameCallback(
      [this](
          const DetectionFrame& detection_frame,
          const sensor_msgs::PointCloud2ConstPtr&
              cloud_message)
      {
        handleSynchronizedFrame(
            detection_frame,
            cloud_message);
      });

  if (!input_adapter_.initialize(config_))
  {
    ROS_FATAL(
        "InputAdapter初始化失败。");
    return false;
  }

  initialized_ = true;

  ROS_INFO(
      "DangerLocalizationNode initialized successfully.");

  return true;
}



void DangerLocalizationNode::handleSynchronizedFrame(
    const DetectionFrame& detection_frame,
    const sensor_msgs::PointCloud2ConstPtr& cloud_message)
{
  if (!cloud_message)
  {
    ROS_WARN_THROTTLE(5.0, "收到空点云消息指针，跳过当前帧。");
    output_adapter_.publishInvalidDangerObservation(
        detection_frame.stamp,
        DangerObservationStatusCode::kInternalError,
        std::numeric_limits<float>::quiet_NaN());
    return;
  }

  if (detection_frame.detections.empty())
  {
    ROS_DEBUG_THROTTLE(
        5.0,
        "当前帧无红球检测，向目标管理器发布一次miss。");

    const PipelineResult invalid_result =
        output_adapter_.publishInvalidDangerObservation(
            detection_frame.stamp,
            DangerObservationStatusCode::kNoDetection,
            std::numeric_limits<float>::quiet_NaN());

    if (invalid_result.status != PipelineStatus::kSuccess)
    {
      reportFailure(
          invalid_result.message,
          "无检测状态输出");
    }
    return;
  }

  for (const auto& detection : detection_frame.detections)
  {
    if (!processDetection(detection_frame, detection, *cloud_message))
    {
      continue;
    }
  }
}

bool DangerLocalizationNode::processDetection(
    const DetectionFrame& detection_frame,
    const BoundingBox2D& detection,
    const sensor_msgs::PointCloud2& cloud_message)
{
  if (detection_frame.source_frame.empty())
  {
    ROS_ERROR_THROTTLE(
        5.0,
        "DetectionFrame缺少点云source_frame。");

    output_adapter_.publishInvalidDangerObservation(
        detection_frame.stamp,
        DangerObservationStatusCode::kInternalError,
        detection.confidence);
    return false;
  }

  const RoiExtractionResult roi_result =
      roi_extractor_.extract(
          cloud_message,
          detection_frame,
          detection);

  if (roi_result.status !=
      PipelineStatus::kSuccess)
  {
    reportFailure(
        roi_result.message,
        "ROI提取");

    output_adapter_.publishInvalidDangerObservation(
        detection_frame.stamp,
        DangerObservationStatusCode::kRoiExtractionFailed,
        detection.confidence);

    return false;
  }

  // ROI调试输出不应阻塞核心定位链路。
  if (config_.publish_debug)
  {
    const PipelineResult roi_publish_result =
        output_adapter_.publishRoiDebug(
            roi_result.points,
            detection_frame.source_frame,
            detection_frame.stamp);

    if (roi_publish_result.status !=
        PipelineStatus::kSuccess)
    {
      reportFailure(
          roi_publish_result.message,
          "ROI调试输出");
    }
  }

  const EstimateResult estimate_result =
      coordinate_estimator_.estimate(
          roi_result.points,
          detection_frame.source_frame,
          detection_frame.stamp);

  if (estimate_result.status !=
      PipelineStatus::kSuccess)
  {
    reportFailure(
        estimate_result.message,
        "候选坐标估计");

    const DangerObservationStatusCode status_code =
        estimate_result.status ==
                PipelineStatus::kQualityRejected
            ? DangerObservationStatusCode::kQualityRejected
            : DangerObservationStatusCode::
                  kCoordinateEstimationFailed;

    output_adapter_.publishInvalidDangerObservation(
        detection_frame.stamp,
        status_code,
        detection.confidence);

    return false;
  }

  // 点云坐标系球心仅用于调试，该输出失败不影响观测链路。
  if (config_.publish_debug)
  {
    const PipelineResult camera_publish_result =
        output_adapter_.publishCameraCandidate(
            estimate_result.candidate);

    if (camera_publish_result.status !=
        PipelineStatus::kSuccess)
    {
      reportFailure(
          camera_publish_result.message,
          "相机球心调试输出");
    }
  }

  // 保留原始相机坐标系候选点。
  // 仅在TF转换阶段允许使用等价的SLAM相机frame别名。
  Point3D tf_candidate = estimate_result.candidate;

  if (!config_.tf_source_frame_override.empty())
  {
    tf_candidate.frame_id =
        config_.tf_source_frame_override;
  }

  Point3D world_point;

  const PipelineResult tf_result =
      tf_transformer_.transform(
          tf_candidate,
          config_.target_frame,
          config_.tf_timeout,
          world_point);

  if (tf_result.status !=
      PipelineStatus::kSuccess)
  {
    reportFailure(
        tf_result.message,
        "TF转换");

    output_adapter_.publishInvalidDangerObservation(
        detection_frame.stamp,
        DangerObservationStatusCode::kTransformFailed,
        detection.confidence);

    return false;
  }

  const PipelineResult world_publish_result =
      output_adapter_.publishWorldCandidate(
          world_point);

  if (world_publish_result.status !=
      PipelineStatus::kSuccess)
  {
    reportFailure(
        world_publish_result.message,
        "world坐标候选点输出");

    // PointStamped是调试输出，DangerObservation仍可继续发布。
  }

  DangerObservationData observation;
  observation.center = world_point;
  observation.fitted_radius =
      estimate_result.fitted_radius;
  observation.sphere_rmse =
      estimate_result.sphere_rmse;
  observation.inlier_ratio =
      estimate_result.inlier_ratio;
  observation.roi_point_count =
      estimate_result.roi_point_count;
  observation.inlier_count =
      estimate_result.inlier_count;
  observation.detector_confidence =
      detection.confidence;

  const PipelineResult observation_result =
      output_adapter_.publishDangerObservation(
          observation);

  if (observation_result.status !=
      PipelineStatus::kSuccess)
  {
    reportFailure(
        observation_result.message,
        "DangerObservation输出");
    return false;
  }

  return true;
}

void DangerLocalizationNode::reportFailure(
    const std::string& message,
    const std::string& stage)
{
  ROS_ERROR_THROTTLE(5.0,
      "%s失败：%s",
      stage.c_str(),
      message.c_str());
}

}  // namespace vision_localization

int main(int argc, char** argv)
{
  ros::init(
      argc,
      argv,
      "danger_localization_node");

  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  vision_localization::DangerLocalizationNode node(
      node_handle,
      private_node_handle);

  if (!node.initialize())
  {
    ROS_FATAL(
        "danger_localization_node initialization failed.");

    return EXIT_FAILURE;
  }

  ROS_INFO(
      "danger_localization_node is running.");

  ros::spin();

  return EXIT_SUCCESS;
}
