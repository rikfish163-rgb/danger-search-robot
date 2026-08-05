#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>

#include <cmath>


double goal_x = 0.0;
double goal_y = 0.0;

bool has_goal = false;


// 接收目标点
void goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_x = msg->pose.position.x;
    goal_y = msg->pose.position.y;

    has_goal = true;

    ROS_INFO(
        "New goal received: %.2f %.2f",
        goal_x,
        goal_y
    );
}



int main(int argc,char** argv)
{
    ros::init(
        argc,
        argv,
        "simple_navigation"
    );


    ros::NodeHandle nh;


    // cmd_vel发布
    ros::Publisher cmd_pub =
        nh.advertise<geometry_msgs::Twist>(
            "/cmd_vel",
            10
        );


    // 目标点订阅
    ros::Subscriber goal_sub =
        nh.subscribe(
            "/goal_pose",
            10,
            goalCallback
        );


    tf::TransformListener listener;


    ros::Rate rate(10);



    while(ros::ok())
    {

        ros::spinOnce();


        geometry_msgs::Twist cmd;


        // 没有目标
        if(!has_goal)
        {
            cmd_pub.publish(cmd);
            rate.sleep();
            continue;
        }


        tf::StampedTransform transform;


        try
        {
            listener.lookupTransform(
                "odom",
                "base",
                ros::Time(0),
                transform
            );
        }

        catch(tf::TransformException &ex)
        {
            ROS_WARN(
                "%s",
                ex.what()
            );

            rate.sleep();

            continue;
        }



        double x =
            transform.getOrigin().x();


        double y =
            transform.getOrigin().y();



        double yaw =
            tf::getYaw(
                transform.getRotation()
            );



        double dx =
            goal_x-x;


        double dy =
            goal_y-y;



        double distance =
            sqrt(
                dx*dx+
                dy*dy
            );



        // 到达目标

        if(distance < 0.2)
        {

            cmd.linear.x = 0;

            cmd.angular.z = 0;


            cmd_pub.publish(cmd);


            ROS_INFO(
                "Goal reached"
            );


            has_goal=false;


            rate.sleep();

            continue;
        }




        double target_yaw =
            atan2(
                dy,
                dx
            );



        double yaw_error =
            target_yaw-yaw;



        while(yaw_error>M_PI)
            yaw_error-=2*M_PI;


        while(yaw_error<-M_PI)
            yaw_error+=2*M_PI;




        // P控制

        cmd.linear.x =
            0.3*distance;


        cmd.angular.z =
            1.0*yaw_error;



        // 限制

        if(cmd.linear.x>0.3)
            cmd.linear.x=0.3;


        cmd_pub.publish(cmd);



        rate.sleep();

    }


    return 0;
}
