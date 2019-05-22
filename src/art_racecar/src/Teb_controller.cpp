
// wrote by bxy
// use fuzzy control method 

#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <teb_local_planner/FeedbackMsg.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include "rsband_local_planner/rsband_local_planner_ros.h"
#include "rsband_local_planner/fuzzy_ptc.h"


rsband_local_planner::FuzzyPTC rs_fuzzy("fuzzy_controller");

class Teb_controller
{
public:
    Teb_controller();
    
private:
    /* data */
    ros::NodeHandle n_;
    ros::Subscriber odom_sub, path_sub, goal_sub;
    ros::Publisher pub_;
    ros::Timer timer1, timer2;
    nav_msgs::Odometry odom;
    nav_msgs::Path map_path;
    geometry_msgs::Point odom_goal_pos;
    geometry_msgs::Twist cmd_vel;
    bool goal_received;
    bool goal_reached;
    tf::TransformListener tf_listener;
    int controller_freq;

    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
    void goalReachingCB(const ros::TimerEvent&);
    void controlLoopCB(const ros::TimerEvent&);
};

Teb_controller::Teb_controller()
{
    ros::NodeHandle p("~");

    p.param("controller_freq", controller_freq, 20);//控制频率

    odom_sub=n_.subscribe("/odometry/filtered",1,&Teb_controller::odomCB,this);
    path_sub=n_.subscribe("/planed_path",1,&Teb_controller::pathCB,this);
    goal_sub=n_.subscribe("move_base_simple/goal",1,&Teb_controller::goalCB,this);
    pub_=n_.advertise<geometry_msgs::Twist>("car/cmd_vel",1);

    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &Teb_controller::controlLoopCB, this); 
}

void Teb_controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}

void Teb_controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}

void Teb_controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    geometry_msgs::PoseStamped odom_goal;
    tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
    odom_goal_pos = odom_goal.pose.position;
    goal_received = true;
    goal_reached = false;
}

void Teb_controller::controlLoopCB(const ros::TimerEvent&)
{
    std::vector<geometry_msgs::PoseStamped> map_path_stdv;
    for (int i = 0; i < map_path.poses.size(); i++)
    {
        map_path_stdv[i]=map_path.poses[i];
    }

    rs_fuzzy.computeVelocityCommands(map_path_stdv,cmd_vel);
    pub_.publish(cmd_vel);
}

int main(int argc, char **argv)
{
    /* code for main function */
    ros::init(argc, argv, "Teb_Controller");
    
    Teb_controller controller;

    ros::spin();
    return 0;
}