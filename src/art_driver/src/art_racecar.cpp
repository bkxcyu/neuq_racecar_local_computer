//
// Created by Steven Zhang on 18-12-14.
// art racecar
//

#include "../include/art_racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

float fmap(float toMap, float in_min, float in_max, float out_min, float out_max)
{
  return(toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void TwistCallback(const geometry_msgs::Twist& twist)
{
    double angle;
    //ROS_INFO("x= %f", twist.linear.x);
    //ROS_INFO("z= %f", twist.angular.z);
    angle = 2500.0 - twist.angular.z * 2000.0 / 180.0;
    //ROS_INFO("angle= %d",uint16_t(angle));
    send_cmd(uint16_t(twist.linear.x),uint16_t(angle));
}

void AckermannCallback(const ackermann_msgs::AckermannDriveStamped& Ackermann)
{
    double angle;
    double vel;
    //ROS_INFO("x= %f", twist.linear.x);
    //ROS_INFO("z= %f", twist.angular.z);
    // angle = 2500.0 - Ackermann.drive.steering_angle* 2000.0 / 180.0;
    if(Ackermann.drive.steering_angle>=0)
        angle=fmap(Ackermann.drive.steering_angle,0,0.4,1500,2500);
    if(Ackermann.drive.steering_angle<0)
        angle=fmap(Ackermann.drive.steering_angle,-0.4,0,500,1500);
    if(Ackermann.drive.speed>0)
        vel=fmap(Ackermann.drive.speed,0,1,1565,1600);
    if(Ackermann.drive.speed<0)
        vel=fmap(Ackermann.drive.speed,-1,0,1400,1435)
    if(Ackermann.drive.speed=0)
        vel=1500;
    //ROS_INFO("angle= %d",uint16_t(angle));
    send_cmd(uint16_t(vel),uint16_t(angle));
}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(38400,data);
    ros::init(argc, argv, "art_driver");
    ros::NodeHandle n;


// ros::Subscriber sub = n.subscribe("/car/cmd_vel",1,TwistCallback);

    ros::Subscriber sub = n.subscribe("/ackermann_cmd",1,AckermannCallback);

        

    ros::spin();

}