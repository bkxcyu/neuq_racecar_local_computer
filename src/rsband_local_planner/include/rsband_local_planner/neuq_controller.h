#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Path.h"
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/Float64.h"
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_core/base_local_planner.h>
// costmap
// #include <costmap_2d/costmap_2d_ros.h>
// #include <costmap_converter/costmap_converter_interface.h>
// boost classes
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
//eigen
#include <tf_conversions/tf_eigen.h>
//rsband
#include "rsband_local_planner/RSBandPlannerConfig.h"


#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
namespace rsband_local_planner
{
    class L1Controller
    {
        public:
            L1Controller(std::string name);
            ~L1Controller();
            
            bool computeVelocityCommands(geometry_msgs::Twist& cmd);

            // void reconfigure(RSBandPlannerConfig& config);

            bool isGoalReached();

            void initMarker();
            bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
            bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
            float getCurrantVel();
            double getYawFromPose(const geometry_msgs::Pose& carPose);
            double getEta(const geometry_msgs::Pose& carPose);
            double getCar2GoalDist();
            double getL1Distance();
            double getSteeringAngle(double eta);
            double getGasInput(const float& current_v);
            float map(float value, float istart, float istop, float ostart, float ostop);
            std_msgs::Float64 computeSlowDownVel();
            std_msgs::Float64 computeIntegralErr();
            std_msgs::Float64 switchErrIntoVel(std_msgs::Float64 Err);
            geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
            visualization_msgs::Marker ObstacleMarker;
            void reconfigure(RSBandPlannerConfig& config);
            geometry_msgs::Twist pwm2vel(geometry_msgs::Twist pwm);

        private:
            ros::NodeHandle n_;
            ros::Subscriber odom_sub, path_sub, goal_sub,obst_sub;
            ros::Publisher pub_, marker_pub,err_pub,obst_marker_pub_;
            ros::Timer timer1, timer2;
            tf::TransformListener tf_listener;

            visualization_msgs::Marker points, line_strip, goal_circle;
            geometry_msgs::Twist cmd_vel;
            geometry_msgs::Twist last_cmd_vel;
            geometry_msgs::Twist currant_vel_from_odom;
            geometry_msgs::Point odom_goal_pos;
            nav_msgs::Odometry odom;
            nav_msgs::Path map_path, odom_path;
            base_local_planner::OdometryHelperRos odom_helper_;

            double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, u, v;
            double RUSH_VEL;
            double MAX_SLOW_DOWN,Lfw_gain;
            double KD,KP,KI;
            double qujian_max,qujian_min;
            double Gas_gain, baseAngle, Angle_gain, goalRadius;
            double last_error,err_sum;
            int controller_freq, baseSpeed;
            int TRAVERSAL_POINT;
            int RUSH_POINT;
            int CurrantPointNumber;
            int MaxPointNumber;
            int BLOOM_START_POINT;
            int BLOOM_START_VEL;
            bool foundForwardPt, goal_received, goal_reached;
            bool reset_flag,stop_flag;

            void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
            void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
            void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
            void obstCB(const visualization_msgs::Marker& obstMsg);
            void goalReachingCB(const ros::TimerEvent&);
            void controlLoopCB(const ros::TimerEvent&);
            bool ReadyToLastRush();
            bool JudgeLockedRotor();
            void BackOff();

    }; // end of class
}


