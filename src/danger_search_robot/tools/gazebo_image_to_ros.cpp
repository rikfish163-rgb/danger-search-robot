#include <cstring>
#include <string>

#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

class GazeboImageToRos {
 public:
  GazeboImageToRos(ros::NodeHandle& nh, const std::string& gazebo_topic,
                   const std::string& ros_topic, const std::string& frame_id)
      : frame_id_(frame_id) {
    publisher_ = nh.advertise<sensor_msgs::Image>(ros_topic, 2);
    gazebo_node_.reset(new gazebo::transport::Node());
    gazebo_node_->Init();
    subscriber_ = gazebo_node_->Subscribe(
        gazebo_topic, &GazeboImageToRos::OnImage, this);
  }

 private:
  void OnImage(
      const boost::shared_ptr<const gazebo::msgs::ImageStamped>& message) {
    const auto& image = message->image();
    const unsigned int width = image.width();
    const unsigned int height = image.height();
    const unsigned int depth = image.step() / width;
    if (width == 0 || height == 0 || depth != 3 ||
        image.data().size() < image.step() * height) {
      ROS_WARN_THROTTLE(2.0, "Unsupported Gazebo image layout");
      return;
    }

    sensor_msgs::Image output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = frame_id_;
    output.width = width;
    output.height = height;
    output.encoding = "rgb8";
    output.is_bigendian = false;
    output.step = image.step();
    output.data.resize(output.step * output.height);
    std::memcpy(output.data.data(), image.data().data(), output.data.size());
    publisher_.publish(output);
  }

  std::string frame_id_;
  ros::Publisher publisher_;
  gazebo::transport::NodePtr gazebo_node_;
  gazebo::transport::SubscriberPtr subscriber_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "gazebo_image_to_ros");
  if (argc != 4) {
    ROS_ERROR("usage: gazebo_image_to_ros GAZEBO_TOPIC ROS_TOPIC FRAME_ID");
    return 2;
  }
  gazebo::client::setup();
  ros::NodeHandle nh;
  GazeboImageToRos bridge(nh, argv[1], argv[2], argv[3]);
  ros::spin();
  gazebo::client::shutdown();
  return 0;
}
