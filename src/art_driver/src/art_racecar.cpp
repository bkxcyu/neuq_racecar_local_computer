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

void TwistCallback(const geometry_msgs::Twist& twist)
{
    double angle;
    double vel;
    ROS_INFO("x= %f", twist.linear.x);
    ROS_INFO("z= %f", twist.angular.z);
    angle = 2500.0 - (twist.angular.z*180/PI+90) * 2000.0 / 180.0;
    if(twist.linear.x>0)
        vel = twist.linear.x*50+1500;
    if(twist.linear.x<0)
        vel = twist.linear.x*50+1500-150;
    // ROS_INFO("angle= %d  vel=%d",uint16_t(angle),uint16_t(vel));
    send_cmd(uint16_t(vel),uint16_t(angle));
}

void AckermannCallback(const ackermann_msgs::AckermannDriveStamped& Ackermann)
{
    double angle;
    double vel;
    //ROS_INFO("x= %f", twist.linear.x);
    //ROS_INFO("z= %f", twist.angular.z);
    angle = 2500.0 - (Ackermann.drive.steering_angle*180/PI+90)* 2000.0 / 180.0;
    if(Ackermann.drive.speed>0)
        vel = Ackermann.drive.speed*50+1550;
    if(Ackermann.drive.speed<0)
        vel = Ackermann.drive.speed*50+1550-300;

    // ROS_INFO("ACKM_angle= %f",(Ackermann.drive.steering_angle*180/PI+90));
    ROS_INFO("PWM_angle= %f, PWM_vel= %f",angle,vel);
    // send_cmd(uint16_t(vel),uint16_t(angle));
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(38400,data);
    ros::init(argc, argv, "art_driver");
    ros::NodeHandle n;


    ros::Subscriber sub1 = n.subscribe("/ackermann_cmd",1,AckermannCallback);
    ros::Subscriber sub2 = n.subscribe("/cmd_vel",1,TwistCallback);
        

    ros::spin();

}