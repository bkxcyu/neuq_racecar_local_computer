#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

class Teb_controller
{
public:
    Teb_controller();

private:
    /* data */
};

Teb_controller::Teb_controller()
{
    
}

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "Teb_Controller");
    Teb_controller Controller;
    ros::spin();
    return 0;
}