#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <geometry_msgs/TransformStamped.h>

class odom_ecoder
{
    public:
        odom_ecoder();
        void velCB(const std_msgs::Float64& currant_vel);

    private:
        ros::Publisher odom_pub;
        tf::TransformBroadcaster odom_broadcaster;
        ros::Subscriber odom_sub;

        ros::NodeHandle n;

        double x,y,th,vx,vy,vth;

        ros::Time current_time, last_time;
};

odom_ecoder::odom_ecoder()
{
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    odom_pub = n.advertise<nav_msgs::Odometry>("/odom1", 50);  
    odom_sub = n.subscribe("/currant_vel", 1, &odom_ecoder::velCB, this); 
}

void odom_ecoder::velCB(const std_msgs::Float64& currant_vel)
{
    double vx=currant_vel.data;

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = vx * dt;
    double delta_y = 0 ;
    double delta_th = 0 ;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "/odom1";
    odom_trans.child_frame_id = "/base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "/odom1";
    odom.child_frame_id = "/base_footprint";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_ecoder");
    odom_ecoder _odom_ecoder;
    ros::spin();
    return 0;
}
