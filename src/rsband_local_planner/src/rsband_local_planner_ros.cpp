/*********************************************************************
* build depend on rsband by George Kouros
* Author:  bkxcyu
*********************************************************************/

#include "rsband_local_planner/rsband_local_planner_ros.h"
#include <base_local_planner/goal_functions.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <tf_conversions/tf_eigen.h>
#include"rsband_local_planner/pose_se2.h"
PLUGINLIB_DECLARE_CLASS(rsband_local_planner, RSBandPlannerROS,
  rsband_local_planner::RSBandPlannerROS, nav_core::BaseLocalPlanner);

namespace rsband_local_planner
{

  RSBandPlannerROS::RSBandPlannerROS() : initialized_(false)
  {
  }


  RSBandPlannerROS::~RSBandPlannerROS()
  {
  }


  void RSBandPlannerROS::initialize(std::string name,
    tf::TransformListener* tfListener, costmap_2d::Costmap2DROS* costmapROS)
  {
    if (initialized_)
    {
      ROS_WARN("Planner already initialized. Should not be called more than "
        "once");
      return;
    }

    // store tflistener and costmapROS
    tfListener_ = tfListener;
    costmapROS_ = costmapROS;
    costmap_ = costmapROS_->getCostmap();

    ros::NodeHandle pnh("~/" + name);

    L1_ = boost::shared_ptr<L1Controller>(new L1Controller(name));

    // create and initialize dynamic reconfigure
    drs_.reset(new drs(pnh));
    drs::CallbackType cb =
      boost::bind(&RSBandPlannerROS::reconfigureCallback, this, _1, _2);
    drs_->setCallback(cb);

    // set initilized
    initialized_ = true;

    //创建链表
    whosyourdaddy.warning_point = whosyourdaddy.creatlist();

    ROS_DEBUG("Local Planner Plugin Initialized!");
  }
  
  void RSBandPlannerROS::reconfigureCallback(RSBandPlannerConfig& config,
    uint32_t level)
  {
    xyGoalTolerance_ = config.xy_goal_tolerance;
    yawGoalTolerance_ = config.yaw_goal_tolerance;

    if (L1_)
      L1_->reconfigure(config);
    else
      ROS_ERROR("Reconfigure CB called before path tracking controller "
        "initialization!");
  }


  bool RSBandPlannerROS::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& globalPlan)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner must be initialized before setPlan is called!");
      return false;
    }

    globalPlan_ = globalPlan;
    return true;
  }


  bool RSBandPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd)
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner must be initialized before computeVelocityCommands "
        "is called!");
      return false;
    }

    if (!L1_->computeVelocityCommands(cmd))
    {
      ROS_ERROR("Path tracking controller failed to produce command");
      return false;
    }
    
    double _adjust_angular;
    _adjust_angular=re_adjust_servo();
    // cmd.angular.z+=_adjust_angular;

    return true;
  }


  bool RSBandPlannerROS::isGoalReached()
  {
    if (!initialized_)
    {
      ROS_ERROR("Planner must be initialized before isGoalReached is called!");
      return false;
    }

    tf::Stamped<tf::Pose> robotPose;
    if (!costmapROS_->getRobotPose(robotPose))
    {
      ROS_ERROR("Could not get robot pose!");
      return false;
    }

    geometry_msgs::PoseStamped goal = globalPlan_.back();

    double dist = base_local_planner::getGoalPositionDistance(
      robotPose, goal.pose.position.x, goal.pose.position.y);
    double yawDiff = base_local_planner::getGoalOrientationAngleDifference(
      robotPose, tf::getYaw(goal.pose.orientation));

    if (dist < xyGoalTolerance_ && fabs(yawDiff) < yawGoalTolerance_)
    {
      ROS_INFO("Goal Reached!");
      return true;
    }

    return false;
  }


  double RSBandPlannerROS::re_adjust_servo()
  {
      double adjust_angular;
      //get robot pose 
      PoseSE2 robot_pose_;
      tf::Stamped<tf::Pose> robot_pose;
      if (!costmapROS_->getRobotPose(robot_pose))
      {
        ROS_ERROR("Could not get robot pose!");
        return false;
      }
      robot_pose_ = PoseSE2(robot_pose);
      //get robot orientation vector
      Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

      whosyourdaddy.clearlist(whosyourdaddy.warning_point);
      //scan local costmap to find obstacle point
      for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
      {
        for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
        { 
          //find one of the obstacles
          if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
          {
            //transform obstacle  of costmap frame into local_map(/odom) frame and get it's direction
            Eigen::Vector2d obs;
            costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            Eigen::Vector2d obs_dir = obs-robot_pose_.position();
           //obs_dir是odom坐标系下 扫描到的障碍物与机器人位置的差向量
           //robot_orient是机器人的方向向量
          /*---------------------------------------------------*/
            //get distance between obstacle and robot
            //get angular between robot orientation and obstacle direction
            float dis,ang;
            dis=obs_dir.norm();
            ang=acos(obs_dir.dot(robot_orient)/(obs_dir.norm()*robot_orient.norm()));
            if(dis<0.2)
            {
              whosyourdaddy.append(whosyourdaddy.warning_point,dis,ang);
              // show_obst(obs.coeffRef(0),obs.coeffRef(1));//odom
            }
          /*---------------------------------------------------*/
          }
        }
      }

      //finally output a vector called whosyourdaddy.out_point ? how to transform it into adjust_angular?
      whosyourdaddy.sortlist(whosyourdaddy.warning_point);
      whosyourdaddy.last_point = whosyourdaddy.getlastnode(whosyourdaddy.warning_point);
      if(whosyourdaddy.last_point->distance == LINK_HEAD_D || whosyourdaddy.last_point->distance == LINK_HEAD_A) 
      	printf("null\n");
      else
      {
      	whosyourdaddy.v_vector(whosyourdaddy.last_point);
      }

      return adjust_angular;
      
  }

}  // namespace rsband_local_planner
