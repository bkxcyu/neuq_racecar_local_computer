//
// Created by Steven Zhang on 18-12-14.
// art racecar
//

#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel2pwm(geometry_msgs::Twist vel)
{
    geometry_msgs::Twist pwm;
    pwm.linear.x=(vel.linear.x+54.1109)/0.0348;
    pwm.linear.z=vel.linear.z;
    pwm.angular.z=vel.angular.z;
    return pwm;
}
void TwistCallback1(const geometry_msgs::Twist& twist)
{
    double angle;
    ROS_INFO("x= %f", twist.linear.x);
    ROS_INFO("z= %f", twist.angular.z);
    angle = 2500.0 - twist.angular.z * 2000.0 / 180.0;
    //ROS_INFO("angle= %d",uint16_t(angle));
    send_cmd(uint16_t(twist.linear.x),uint16_t(angle));
}

void TwistCallback(const geometry_msgs::Twist& twist)
{
    double angle;
    static double last_in_x;
    static double last_in_z;

    geometry_msgs::Twist _twist;
    _twist=vel2pwm(twist);

    // ROS_INFO("get x= %f", twist.linear.x);
    // ROS_INFO("get z= %f", twist.angular.z);

    //aviod noize

    if(_twist.linear.x==0&&last_in_x!=0)
    {
        ROS_INFO(" corrected output x= %f", last_in_x);
        ROS_INFO(" corrected output z= %f", last_in_z);
        angle = 2500.0 - last_in_z * 2000.0 / 180.0;
        send_cmd(uint16_t(last_in_x),uint16_t(angle));
        return;
    }


    angle = 2500.0 - _twist.angular.z * 2000.0 / 180.0;//2200    1500   770
    
    // last_in_x=twist.linear.x;
    // last_in_z=twist.angular.z;
    ROS_INFO("output x= %f", _twist.linear.x);
    ROS_INFO("output z= %f", _twist.angular.z);
    send_cmd(uint16_t(_twist.linear.x+_twist.linear.z*300),uint16_t(angle));
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(38400,data);
    ros::init(argc, argv, "art_driver");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/cmd_vel",1,TwistCallback);
    ros::Subscriber sub1 = n.subscribe("/car/cmd_vel",1,TwistCallback1);



    ros::spin();

}