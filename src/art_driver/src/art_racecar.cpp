//
// Created by Steven Zhang on 18-12-14.
// art racecar
//

#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#define PI 3.14159265358979
#include <ackermann_msgs/AckermannDriveStamped.h>

float fmap(float toMap, float in_min, float in_max, float out_min, float out_max)
{
  return(toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void TwistCallback(const geometry_msgs::Twist& twist)
{
    double angle;
    double vel;
    double angle_in_degree;
    ROS_INFO("x= %f", twist.linear.x);
    ROS_INFO("z= %f", twist.angular.z);

     if(twist.angular.z>=0)
        angle=fmap(twist.angular.z,0,0.3,1500,900);
    if(twist.angular.z<0)
        angle=fmap(twist.angular.z,0,-0.3,1500,2100);

    if(twist.linear.x>0)
        vel=fmap(twist.linear.x,0,1,1550,1600);
    if(twist.linear.x<0)
        vel=fmap(twist.linear.x,0,-1,1350,1300);
    if(twist.linear.x==0)
        vel=1500;
     ROS_INFO("angle= %d  vel=%d",uint16_t(angle),uint16_t(vel));
    send_cmd(uint16_t(vel),uint16_t(angle));
}


int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(38400,data);
    ros::init(argc, argv, "art_driver");
    ros::NodeHandle n;

    ros::Subscriber sub2 = n.subscribe("/cmd_vel",1,TwistCallback);
        

    ros::spin();

}