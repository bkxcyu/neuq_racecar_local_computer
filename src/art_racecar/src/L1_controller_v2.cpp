/*
Copyright (c) 2017, ChanYuan KUO, YoRu LU,
latest editor: HaoChih, LIN
All rights reserved. (Hypha ROS Workshop)

This file is part of hypha_racecar package.

hypha_racecar is free software: you can redistribute it and/or modify
it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published
by the Free Software Foundation, either version 3 of the License, or
any later version.

hypha_racecar is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU LESSER GENERAL PUBLIC LICENSE for more details.

You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
along with hypha_racecar.  If not, see <http://www.gnu.org/licenses/>.
*/

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


#define PI 3.14159265358979

/********************/
/* CLASS DEFINITION */
/********************/
class L1Controller
{
    public:
        L1Controller();
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
        int countCostOfARegion(geometry_msgs::Pose robotPose);
        float map(float value, float istart, float istop, float ostart, float ostop);
        std_msgs::Float64 computeSlowDownVel();
        std_msgs::Float64 computeIntegralErr();
        std_msgs::Float64 switchErrIntoVel(std_msgs::Float64 Err);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
        visualization_msgs::Marker ObstacleMarker;
        // costmap_2d::Costmap2DROS* costmap_ros;//！！！由于无法直接引用move_base创建的costmap对象 这里可能会出现问题
        // costmap_2d::Costmap2D* costmap_;

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
        double MAX_SLOW_DOWN;
        double KD,KP,KI;
        double qujian_max,qujian_min;
        double Gas_gain, baseAngle, Angle_gain, goalRadius;
        double last_error,err_sum;//////////////
        int controller_freq, baseSpeed;
        int TRAVERSAL_POINT;
        bool foundForwardPt, goal_received, goal_reached;

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void obstCB(const visualization_msgs::Marker& obstMsg);
        void goalReachingCB(const ros::TimerEvent&);
        void controlLoopCB(const ros::TimerEvent&);

}; // end of class


L1Controller::L1Controller()
{

    last_cmd_vel.linear.x=Vcmd;
    last_cmd_vel.angular.z=0;
    
    // costmap_ = costmap_ros->getCostmap(); 

    odom_helper_.setOdomTopic("/odometry/filtered");

    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter  参照群文件  HyphaROS_RaceCar_Project_released.pdf
    pn.param("L", L, 0.26);//前后车轮间距
    pn.param("Lrv", Lrv, 10.0);//后预瞄距离
    pn.param("Vcmd", Vcmd, 1.0);//期望速度
    pn.param("lfw", lfw, 0.13);//前锚距
    pn.param("lrv", lrv, 10.0);//后锚距      这些参数可能需要修改

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);//控制频率
    pn.param("AngleGain", Angle_gain, -1.0);//角度增益（系数）
 
    pn.param("KP", KP, -1.0);//角度增益（系数）
    pn.param("KD", KD, -1.0);//角度增益（系数）

    pn.param("GasGain", Gas_gain, 1.0);//电机输出增益（系数P）
    pn.param("baseSpeed", baseSpeed, 1470);//基速度
    pn.param("baseAngle", baseAngle, 90.0);//基角度
    pn.param("MAX_SLOW_DOWN", MAX_SLOW_DOWN, 40.0);
    pn.param("qujian_min", qujian_min, 7.0);
    pn.param("qujian_max", qujian_max, 20.0);
    pn.param("TRAVERSAL_POINT", TRAVERSAL_POINT, 100);

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odometry/filtered", 1, &L1Controller::odomCB, this);//订阅位置消息                       注意回调函数
    path_sub = n_.subscribe("/move_base_node/NavfnROS/plan", 1, &L1Controller::pathCB, this);//订阅导航堆栈信息         注意回调函数
    goal_sub = n_.subscribe("/move_base_simple/goal", 1, &L1Controller::goalCB, this);//订阅位置（目标位置）信息          注意回调函数
    // obst_sub = n_.subscribe("teb_markers", 1, &L1Controller::obstCB, this);
    // obst_marker_pub_ = n_.advertise<visualization_msgs::Marker>("obst_markers", 1000);
    marker_pub = n_.advertise<visualization_msgs::Marker>("car_path", 10);//创建发布控制命令的发布者
    pub_ = n_.advertise<geometry_msgs::Twist>("car/cmd_vel", 1);//角速度 先速度
    err_pub=n_.advertise<std_msgs::Float64>("car/err", 1);

    //Timer 定时中断
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &L1Controller::controlLoopCB, this); // Duration(0.05) -> 20Hz//根据实时位置信息和导航堆栈更新舵机角度和电机速度，存在cmd_vel话题里
    timer2 = n_.createTimer(ros::Duration((0.5)/controller_freq), &L1Controller::goalReachingCB, this); // Duration(0.05) -> 20Hz//判断是否到达目标位置
    
    //Init variables
    Lfw = goalRadius = getL1Distance();//获取预瞄距离  期望速度越快 预瞄距离越大
    foundForwardPt = false;//是否存在可行航迹点
    goal_received = false;//目标是否获取到（目标是否发布）
    goal_reached = false;//是否到达目标
    cmd_vel.linear.x = 1500; // 1500 for stop
    cmd_vel.angular.z = baseAngle;

    //Show info
    ROS_INFO("[param] baseSpeed: %d", baseSpeed);
    ROS_INFO("[param] baseAngle: %f", baseAngle);
    ROS_INFO("[param] AngleGain: %f", Angle_gain);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    //Visualization Marker Settings 可视化标记初始化
    initMarker();

    //往下翻 找到  controlLoopCB  接着看
}



void L1Controller::initMarker()
{
    points.header.frame_id = line_strip.header.frame_id = goal_circle.header.frame_id = "odom";
    points.ns = line_strip.ns = goal_circle.ns = "Markers";
    points.action = line_strip.action = goal_circle.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = goal_circle.pose.orientation.w = 1.0;
    points.id = 0;
    line_strip.id = 1;
    goal_circle.id = 2;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    goal_circle.type = visualization_msgs::Marker::CYLINDER;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    //LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    goal_circle.scale.x = goalRadius;
    goal_circle.scale.y = goalRadius;
    goal_circle.scale.z = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    //goal_circle is yellow
    goal_circle.color.r = 1.0;
    goal_circle.color.g = 1.0;
    goal_circle.color.b = 0.0;
    goal_circle.color.a = 0.5;
}


void L1Controller::obstCB(const visualization_msgs::Marker& obstMsg)
{
    ObstacleMarker = obstMsg;
    obst_marker_pub_.publish(ObstacleMarker);
}

void L1Controller::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    odom = *odomMsg;
}


void L1Controller::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    map_path = *pathMsg;
}


void L1Controller::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;

        /*Draw Goal on RVIZ*/
        goal_circle.pose = odom_goal.pose;
        marker_pub.publish(goal_circle);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

double L1Controller::getYawFromPose(const geometry_msgs::Pose& carPose)
{   //从车的姿态（四元数）计算固有转向角（舵机打角）
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);//四元数转姿态角 分别为 绕x y z v这里只需要转向角（绕z）

    return yaw;
}

bool L1Controller::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{   //输入航路点的坐标和车的坐标 返回前方是否有航路点 即是否能到达该坐标
    float car2wayPt_x = wayPt.x - carPose.position.x;//从车到航路点的x y轴向距离
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);//从车的姿态（四元数）计算固有转向角（舵机打角）

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}


bool L1Controller::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{   //输入航路点和车坐标 返回是否能直接到达
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);//车到航路点的直线距离

    if(dist < Lfw)//如果这个距离小于预瞄距离（转弯半径） 则视为不能到达
        return false;
    else if(dist >= Lfw)
        return true;
}

geometry_msgs::Point L1Controller::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{   //输入车坐标（以及规划的路径信息）返回iu可行的目标向量
    geometry_msgs::Point carPose_pos = carPose.position;//车的位置重映射
    double carPose_yaw = getYawFromPose(carPose);//从车的姿态（四元数）计算固有转向角（舵机打角）
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){
        for(int i =0; i< map_path.poses.size(); i++)
        {
            geometry_msgs::PoseStamped map_path_pose = map_path.poses[i];//路径信息重映射
            geometry_msgs::PoseStamped odom_path_pose;

            try
            {
                tf_listener.transformPose("odom", ros::Time(0) , map_path_pose, "map" ,odom_path_pose);//将map坐标系下的路径信息（含有戳记的坐标数组）转换到odom坐标系下，存入odom_path_pose
                geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;//规划的路径坐标信息（航路点）存入odom_path_wayPt
                bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);//输入航路点的坐标和车的坐标 返回前方是否有航路点 即是否能到达该坐标

                if(_isForwardWayPt)//如果没有可行航路点怎么办？？？  航路点太超前怎么办？？？
                {
                    bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);//输入航路点和车坐标 返回是否能直接到达（是否在转弯的盲区）
                    if(_isWayPtAwayFromLfwDist)
                    {
                        forwardPt = odom_path_wayPt;//将可行的航路点存入forwardPt
                        foundForwardPt = true;
                        break;
                    }
                }
            }
            catch(tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }
        }

    }
    else if(goal_reached)//到达目标
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }

    /*Visualized Target Point on RVIZ*/
    /*Clear former target point Marker*/
    //下面是rviz可视化的内容
    points.points.clear();
    line_strip.points.clear();

    if(foundForwardPt && !goal_reached)//前方有可行航路点而且未到达目标位置
    {
        points.points.push_back(carPose_pos);
        points.points.push_back(forwardPt);
        line_strip.points.push_back(carPose_pos);
        line_strip.points.push_back(forwardPt);
    }

    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    return odom_car2WayPtVec;
}


double L1Controller:: getEta(const geometry_msgs::Pose& carPose)
{   //输入车坐标（四元数）  返回方位角（与x轴） 单位是弧度
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);//输入车坐标（以及规划的路径信息）返回iu可行的目标向量

    double eta = atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);//根据目标向量返回方位角（与x轴） 单位是弧度
    return eta;
}


double L1Controller::getCar2GoalDist()
{   //获取车到目标点的距离
    geometry_msgs::Point car_pose = odom.pose.pose.position;
    double car2goal_x = odom_goal_pos.x - car_pose.x;
    double car2goal_y = odom_goal_pos.y - car_pose.y;

    double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);

    return dist2goal;
}

double L1Controller::getL1Distance()
{   //获取预瞄距离  期望速度越快 预瞄距离越大
    
    float v_el =  getCurrantVel();
    double L1 = 0.8;
    v = fabs(v_el);
    // if(v >= 0 || v <= 3)
    // L1 = 0.22*(v*v-6*v+9)+1;
     if(v <= 1.6)
         L1 = 3 / 3.0;
     else if(v > 1.6 && v < 4.8)
         L1 = 1.25*v-1;
    //  if(v>=0||v<=3)
    //     L1 = 1.07*v + 0.8;
     else if(v >= 4.8)
         L1 = 5;
        // ROS_INFO("L1 = %.2f",L1);
    return L1;
}

double L1Controller::getSteeringAngle(double eta)
{   //根据方位角计算舵机打角
    double steeringAnge = -atan2((L*sin(eta)),(Lfw/2+lfw*cos(eta)))*(180.0/PI);
    //ROS_INFO("Steering Angle = %.2f", steeringAnge);
    return steeringAnge;
}

double L1Controller::getGasInput(const float& current_v)
{   //P控制
    double u = (Vcmd - current_v)*Gas_gain;
    //ROS_INFO("velocity = %.2f\tu = %.2f",current_v, u);
    return u;
}


void L1Controller::goalReachingCB(const ros::TimerEvent&)
{   //判断车是否到达目标

    if(goal_received)//取得目标
    {
        double car2goal_dist = getCar2GoalDist();//获取车到目标点的距离
        //相对位置小于预瞄距离 则认为到达
        if(car2goal_dist < goalRadius)
        {
            goal_reached = true;
            goal_received = false;
            ROS_INFO("Goal Reached !");
        }
    }
}

void L1Controller::controlLoopCB(const ros::TimerEvent&)
{   //根据实时位置信息和导航堆栈更新舵机角度和电机速度，存在cmd_vel话题里
    geometry_msgs::Pose carPose = odom.pose.pose;//话题消息重映射 位置
    geometry_msgs::Twist carVel = odom.twist.twist;//速度
    cmd_vel.linear.x = 1500;
    cmd_vel.angular.z = baseAngle;

    
/*>>>>>>>>>>>>>>>>>>>>   Lfw_REMAP   >>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
    // getCurrantVel();
    // ROS_INFO("SPEED_X:%f Z:%f",currant_vel_from_odom.linear.x ,currant_vel_from_odom.angular.z );
    Lfw = goalRadius = getL1Distance();
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
    if(goal_received)//取得目标
    {
        /*Estimate Steering Angle*///估计转向角
        double eta = getEta(carPose); //基于车体的动力学模型和导航堆栈计算出转向角    这部分参考群里两篇论文 ：KuwataTCST09.pdf   KuwataGNC08.pdf
/*>>>>>>>>>>>>>>>>>>   SLOW_down feature >>>>>>>>>>>>>>>>>>>>>>>>*/
        std_msgs::Float64 slow_down_vel;
        slow_down_vel=computeSlowDownVel();
        if(slow_down_vel.data<0)
        {
            slow_down_vel.data=0;
            ROS_ERROR("slow_down_vel<0,CHECK!");
        }
/*<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<*/
        if(foundForwardPt)//是否存在可行航路点
        {
          //  cmd_vel.angular.z = baseAngle + getSteeringAngle(eta)*Angle_gain;//将转向角存入cmd-vel
            //   cmd_vel.angular.z = baseAngle + KP*getSteeringAngle(eta)+KD*(getSteeringAngle(eta)-last_error);
           //    last_error=getSteeringAngle(eta);
           err_sum=err_sum+baseAngle;
            cmd_vel.angular.z = baseAngle + KP*getSteeringAngle(eta)+KD*(getSteeringAngle(eta)-last_error)+KI*err_sum;
            last_error=getSteeringAngle(eta);
            /*Estimate Gas Input*/
            if(!goal_reached)//如果沒有到達目標點则继续以基速度行驶
            {
                //double u = getGasInput(carVel.linear.x);
                //cmd_vel.linear.x = baseSpeed - u;
                cmd_vel.linear.x = baseSpeed-slow_down_vel.data;
                //  ROS_INFO("\nGas = %.2f\nSteering angle = %.2f",cmd_vel.linear.x,cmd_vel.angular.z);
            }
        }
    }
    last_cmd_vel=cmd_vel;
    pub_.publish(cmd_vel);//发布控制指令 ：包含转向角和 期望速度
}
/*---------------------------------------------------------------------------------------*/

int L1Controller::countCostOfARegion(geometry_msgs::Pose robot_pose_of_odom)
{
    int cost=0;
    // geometry_msgs::Pose carPoseOfMapFrame;
    // tf_listener.transformPose("map", ros::Time(0) , robot_pose_of_odom, "odom" ,carPoseOfMapFrame);

    // Eigen::Vector2d robot_orient = robot_pose_of_odom.orientationUnitVec();

    // for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    // {
    //   for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
    //   {

    //     if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
    //     {
    //         geometry_msgs::PoseStamped obs_of_map;
    //         geometry_msgs::PoseStamped obs_of_car;
    //         geometry_msgs::PoseStamped obst_markers;
            
    //         costmap_->mapToWorld(i,j,obs_of_map.pose.position.x, obs_of_map.pose.position.y);
    //         ROS_INFO("obs_of_map:(%.1f,%.1f)\n",obs_of_map.pose.position.x,obs_of_map.pose.position.y);

    //         tf_listener.transformPose("base_footprint", ros::Time(0) , obs_of_map, "map" ,obs_of_car);
    //         ROS_INFO("obs_of_car:(%.1f,%.1f)\n",obs_of_car.pose.position.x,obs_of_car.pose.position.y);

    //         Eigen::Vector2d obs_dir(obs_of_car.pose.position.x,obs_of_car.pose.position.y);
    //         Eigen::Vector2d car_dir(0,1);
    //         float theta;
    //         theta=std::acos(obs_dir.dot(car_dir)/(obs_dir.norm()*car_dir.norm()));
    //         ROS_INFO("theta:%.1f\n",theta);

    //         if (std::fabs(theta)<0.3)
    //         {
    //             cost++;
    //             ROS_INFO("cost:%d\n\n",cost);
    //             obst_markers.pose.position.x=obs_of_map.pose.position.x;
    //             obst_markers.pose.position.y=obs_of_map.pose.position.y;
    //             tf_listener.transformPose("odom", ros::Time(0) , obs_of_map, "map" ,obst_markers);
    //             points.points.push_back(obst_markers.pose.position);
    //         }     
    //     }
    //   }
    // }

    // marker_pub.publish(points);
    return cost;
}









float L1Controller::getCurrantVel()
{
    tf::Stamped<tf::Pose> robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);
    currant_vel_from_odom.linear.x = robot_vel_tf.getOrigin().getX();
    currant_vel_from_odom.linear.y = robot_vel_tf.getOrigin().getY();
    currant_vel_from_odom.angular.z = tf::getYaw(robot_vel_tf.getRotation());
    return currant_vel_from_odom.linear.x;
}


float L1Controller::map(float value, float istart, float istop, float ostart, float ostop)
{
	return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

std_msgs::Float64 L1Controller::switchErrIntoVel(std_msgs::Float64 Err)
{
    std_msgs::Float64 vel;
    // double k=0.5;
    // /*---------------------------------------------------------------------------------------*/


    // /*---------------------------------------------------------------------------------------*/
    // vel.data=fabs(k*Err.data);
    // if(vel.data>30)
    //     vel.data=30;
    //vel.data= -0.0027*fabs(Err.data)*fabs(Err.data)*fabs(Err.data)+0.1736*fabs(Err.data)*fabs(Err.data)-0.6398*fabs(Err.data)+3;
    double mediate;
    double fErr = fabs(Err.data);
     mediate = -pow(fErr,1.5)+12;
     vel.data = MAX_SLOW_DOWN/(1+exp(mediate));
    
    //work at 6.11 p.m.10:35 without error,but have not been test;
    // double a,b,c;
    // a = -1;
    // b = (qujian_min*qujian_min-qujian_max*qujian_max-MAX_SLOW_DOWN)/(qujian_min-qujian_max);
    // c = (qujian_min*qujian_min*qujian_max-qujian_min*MAX_SLOW_DOWN-qujian_min*qujian_max*qujian_max)/(qujian_max-qujian_min);
    // if(0.5*b>=qujian_min && 0.5*b<=qujian_max)
    //     a = -0.5*b/qujian_max;
    //     else
    //     a = -1;
    // if (fErr <= qujian_min)
    //  vel.data = 0;
    // if (fErr > qujian_min && fErr < qujian_max)
    //  vel.data = a*fErr*fErr + b*fErr +c;
        
    // else
    //  vel.data = MAX_SLOW_DOWN;
    //test test test test test test test
    
    if(std::isnan(vel.data)||std::isinf(vel.data))
    {
        ROS_ERROR("The caculated vel is nan or inf,something wrong,check it") ;
        Err.data=MAX_SLOW_DOWN;
        return vel;
    }

    // ROS_INFO("\nslow down vel=%f",vel.data);
    return vel;
}




std_msgs::Float64 L1Controller::computeIntegralErr()
{
    std_msgs::Float64 Err;
    Err.data=0;
    //坐标转换
    geometry_msgs::PoseStamped carPoseSt;//odom坐标系下车的坐标
    carPoseSt.pose=odom.pose.pose;
    carPoseSt.header=odom.header;
    geometry_msgs::PoseStamped carPoseOfCarFrame;//车坐标系下 车的坐标
    geometry_msgs::PoseStamped map_pathOfCarFrame;//车坐标系下 路径的坐标
    int path_point_number=TRAVERSAL_POINT;
    std_msgs::Float64 path_x;
    std_msgs::Float64 path_y;
    std_msgs::Float64 path_x_max;
    path_x_max.data=0;

    try
    {
        tf_listener.transformPose("base_footprint", ros::Time(0) , carPoseSt, "odom" ,carPoseOfCarFrame);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s,at L1 line 639",ex.what());
        ros::Duration(1.0).sleep();
    }
    while(path_point_number>map_path.poses.size())
    {
        path_point_number--;
        // ROS_INFO("\npath_point_number now is big than the size of path,so cut down it to %d",path_point_number);
        if(path_point_number==0)
        {
           ROS_WARN("\n---------------\npath_point_number=0 now,we maybe we've arrived,if not,please check");
           Err.data=0;
           return Err;
        }
    }
    /*------------------------------------算法实现部分----------------------------------------*/
    for(int i=0;i<path_point_number;i++)
    {   
        try
        {
            tf_listener.transformPose("base_footprint", ros::Time(0) , map_path.poses[i], "map" ,map_pathOfCarFrame);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s,at L1 line 655",ex.what());
            ros::Duration(1.0).sleep();
        }

        if(map_pathOfCarFrame.pose.position.x<0)
        {
            // ROS_INFO("The path_x<0  now ignore it and jump to next") ;
            continue;
        }
        path_x.data=map_pathOfCarFrame.pose.position.x;
        path_y.data=map_pathOfCarFrame.pose.position.y;
        path_x_max.data=std::max(path_x.data,path_x_max.data);//x坐标最大值
        Err.data+=path_y.data;

        // ROS_INFO("\nmap_pathOfCarFrame.x=%f map_pathOfCarFrame.y=%f ",map_pathOfCarFrame.pose.position.x,map_pathOfCarFrame.pose.position.y);
        // ROS_INFO("\nAdd %d times\n now err=%f",(int)i,Err.data);
        // ROS_INFO("\nmax_x =%f",path_x_max.data);
    }
    Err.data=Err.data/path_x_max.data;
    /*---------------------------------------------------------------------------------------*/

     try
    {
    points.points.clear();
    geometry_msgs::PoseStamped map_pathOfOdomFrame;
    tf_listener.transformPose("odom", ros::Time(0) , map_path.poses[path_point_number], "map" ,map_pathOfOdomFrame);
    points.points.push_back(map_pathOfOdomFrame.pose.position);
    marker_pub.publish(points);
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s,in L1,at line 685",ex.what());
        ros::Duration(1.0).sleep();
    }

    if(std::isnan(Err.data)||std::isinf(Err.data))
    {
        ROS_ERROR("The IntegralErr is nan or inf,something wrong,check it") ;
        Err.data=0;
        return Err;
    }


    // ROS_INFO("\n --- loop once finallly output ---\n err=%f ",Err.data);

    err_pub.publish(Err);
    return Err;
}


std_msgs::Float64 L1Controller::computeSlowDownVel()
{
    std_msgs::Float64 slow_down_vel;
    std_msgs::Float64 Err;

    Err=computeIntegralErr();
    slow_down_vel=switchErrIntoVel(Err);

    return slow_down_vel;
}




/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "L1Controller_v2");
    L1Controller controller;
    ros::spin();//循环执行
    return 0;
}

