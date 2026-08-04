#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_vel_test");

    ros::NodeHandle nh;

    ros::Publisher pub =
        nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate rate(10);

    ROS_INFO("cmd_vel test started");

    while(ros::ok())
    {
        geometry_msgs::Twist cmd;

        cmd.linear.x = 0.2;
        cmd.angular.z = 0.0;

        pub.publish(cmd);

        ros::spinOnce();

        rate.sleep();
    }

    return 0;
}