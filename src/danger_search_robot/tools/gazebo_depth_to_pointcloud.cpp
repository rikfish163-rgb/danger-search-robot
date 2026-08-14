#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

class GazeboDepthToPointCloud {
 public:
  GazeboDepthToPointCloud(ros::NodeHandle& nh, const std::string& gazebo_topic,
                          const std::string& ros_topic,
                          const std::string& frame_id, double horizontal_fov)
      : frame_id_(frame_id),
        fx_(640.0 / (2.0 * std::tan(horizontal_fov / 2.0))),
        fy_(fx_),
        cx_(319.5),
        cy_(239.5) {
    publisher_ = nh.advertise<sensor_msgs::PointCloud2>(ros_topic, 2);
    gazebo_node_.reset(new gazebo::transport::Node());
    gazebo_node_->Init();
    subscriber_ = gazebo_node_->Subscribe(
        gazebo_topic, &GazeboDepthToPointCloud::OnImage, this);
  }

 private:
  void OnImage(
      const boost::shared_ptr<const gazebo::msgs::ImageStamped>& message) {
    const auto& image = message->image();
    const unsigned int width = image.width();
    const unsigned int height = image.height();
    const unsigned int step = image.step();
    if (width == 0 || height == 0 || step < width * sizeof(float) ||
        image.data().size() < step * height) {
      ROS_WARN_THROTTLE(2.0, "Unsupported Gazebo depth image layout");
      return;
    }

    sensor_msgs::PointCloud2 output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = frame_id_;
    output.height = height;
    output.width = width;
    output.is_bigendian = false;
    output.is_dense = false;
    output.point_step = 3 * sizeof(float);
    output.row_step = output.point_step * width;
    output.fields.resize(3);
    output.fields[0].name = "x";
    output.fields[0].offset = 0;
    output.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[0].count = 1;
    output.fields[1].name = "y";
    output.fields[1].offset = sizeof(float);
    output.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[1].count = 1;
    output.fields[2].name = "z";
    output.fields[2].offset = 2 * sizeof(float);
    output.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[2].count = 1;
    output.data.resize(output.row_step * height);

    for (unsigned int v = 0; v < height; ++v) {
      const auto* row = reinterpret_cast<const float*>(
          image.data().data() + static_cast<size_t>(v) * step);
      for (unsigned int u = 0; u < width; ++u) {
        const float depth = row[u];
        float x = std::numeric_limits<float>::quiet_NaN();
        float y = std::numeric_limits<float>::quiet_NaN();
        float z = std::numeric_limits<float>::quiet_NaN();
        if (std::isfinite(depth) && depth > 0.05f && depth < 8.0f) {
          z = depth;
          x = static_cast<float>((static_cast<double>(u) - cx_) * depth / fx_);
          y = static_cast<float>((static_cast<double>(v) - cy_) * depth / fy_);
        }
        auto* point = output.data.data() +
                      static_cast<size_t>(v) * output.row_step +
                      static_cast<size_t>(u) * output.point_step;
        std::memcpy(point, &x, sizeof(float));
        std::memcpy(point + sizeof(float), &y, sizeof(float));
        std::memcpy(point + 2 * sizeof(float), &z, sizeof(float));
      }
    }
    publisher_.publish(output);
  }

  std::string frame_id_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  ros::Publisher publisher_;
  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr subscriber_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_depth_to_pointcloud");
  if (argc != 5) {
    ROS_ERROR("usage: gazebo_depth_to_pointcloud GAZEBO_TOPIC ROS_TOPIC FRAME_ID HORIZONTAL_FOV_RAD");
    return 2;
  }
  gazebo::client::setup();
  ros::NodeHandle nh;
  GazeboDepthToPointCloud bridge(nh, argv[1], argv[2], argv[3],
                                 std::stod(argv[4]));
  ros::spin();
  gazebo::client::shutdown();
  return 0;
}
