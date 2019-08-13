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

    obst_pub = _n_.advertise<visualization_msgs::Marker>("POINT", 1);

    L1_ = boost::shared_ptr<L1Controller>(new L1Controller(name));

    // create and initialize dynamic reconfigure
    drs_.reset(new drs(pnh));
    drs::CallbackType cb =
      boost::bind(&RSBandPlannerROS::reconfigureCallback, this, _1, _2);
    drs_->setCallback(cb);

    // set initilized
    initialized_ = true;
    initMarker();

    //创建链表
    // whosyourdaddy=boost::shared_ptr<point_list>(new point_list());
    whosyourdaddy.warning_point = whosyourdaddy.creatlist();

    ROS_DEBUG("Local Planner Plugin Initialized!");
  }
  void RSBandPlannerROS::initMarker()
  {
    //initpoint
    points.header.frame_id =line_strip.header.frame_id = "odom";
    points.ns=line_strip.ns = "Markers";
    points.action=line_strip.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w=line_strip.pose.orientation.w =1.0;
    points.id= 0;
    line_strip.id = 1;
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    // POINTS markers use x and y scale for width/height respectively

    points.scale.x = 0.1;
    points.scale.y = 0.1;
    // Points are green
    points.color.r = 1.0;
    points.color.g = 0.0;
    points.color.b = 0.0;
    points.color.a = 0.8;

    line_strip.scale.x = 0.05;
    line_strip.scale.y = 0.05;
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;
  }
  void RSBandPlannerROS::show_obst(float x,float y,const PoseSE2& carPose)
  {
    geometry_msgs::Point POINT;
    geometry_msgs::Point carpoint;
    //visualization_msgs::Marker points;
    //toPoseMsg(geometry_msgs::Pose& pose)
    geometry_msgs::Pose _carPose;
    carPose.toPoseMsg(_carPose);
    carpoint.x = _carPose.position.x;
    carpoint.y = _carPose.position.y;
    carpoint.z = 0.0;
    POINT.x = x;
    POINT.y = y;
    POINT.z = 0.0;
    points.points.clear();
    points.points.push_back(carpoint);
    points.points.push_back(POINT);

    // ROS_INFO("get point");
    obst_pub.publish(points);
    //obst_pub = _n_.advertise<visualization_msgs::Marker>("POINT", 10);
  }
  void RSBandPlannerROS::show_obst()
  {
    // geometry_msgs::Point POINT;
    // geometry_msgs::Point carpoint;
    // //visualization_msgs::Marker points;
    // carpoint.x = x;
    // carpoint.y = y;
    // carpoint.z = 0.0;
    // POINT.x = x;
    // POINT.y = y;
    // POINT.z = 0.0;
    // points.points.clear();
    // points.points.push_back(carpoint);
    // points.points.push_back(POINT);

    // ROS_INFO("get point");
    obst_pub.publish(points);
    obst_pub.publish(line_strip);
    //obst_pub = _n_.advertise<visualization_msgs::Marker>("POINT", 10);
  }
  void RSBandPlannerROS::addVizPoint(float x,float y)
  {
    geometry_msgs::Point vis_obst;
    vis_obst.x=x;
    vis_obst.y=y;   
    points.points.push_back(vis_obst);
  }
  void RSBandPlannerROS::addVizLine(float xstart,float ystart,float xend,float yend)
  {
    line_strip.points.clear();
    geometry_msgs::Point startPoint;
    startPoint.x=xstart;
    startPoint.y=ystart;
    geometry_msgs::Point endPoint;
    endPoint.x=xend;
    endPoint.y=yend;
    line_strip.points.push_back(startPoint);
    line_strip.points.push_back(endPoint);
  }
  
  void RSBandPlannerROS::reconfigureCallback(RSBandPlannerConfig& config,
    uint32_t level)
  {
    // xyGoalTolerance_ = config.xy_goal_tolerance;
    // yawGoalTolerance_ = config.yaw_goal_tolerance;
    whosyourdaddy.gain_angle=config.gain_angle;
    whosyourdaddy.unit_distance=config.unit_distance;
    whosyourdaddy.warning_distance=config.warning_distance;
    whosyourdaddy.limit_distance=config.limit_distance;
    whosyourdaddy.angle_max=config.angle_max;
    whosyourdaddy.angle_min=config.angle_min;

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

    // if (!L1_->computeVelocityCommands(cmd))
    // {
    //   ROS_ERROR("Path tracking controller failed to produce command");
    //   return false;
    // }
    
    double _rectified_angular;
    _rectified_angular=rectifyAngularVel();
    cmd.angular.z=_rectified_angular;////
    ROS_INFO("output pwm=%.2f",cmd.angular.z);

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

  float map(float value, float istart, float istop, float ostart, float ostop)
  {
	  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
  }
  double RSBandPlannerROS::rectifyAngularVel()
  {
      double rectified_angular;
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

      whosyourdaddy.clearlist( whosyourdaddy.warning_point);
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

            // show_obst(obs.coeffRef(0),obs.coeffRef(1),robot_pose_);//odom
            
           //obs_dir是odom坐标系下 扫描到的障碍物与机器人位置的差向量
           //robot_orient是机器人的方向向量
          /*---------------------------------------------------*/
            //switch  data in rectangular coordinates to polar coordinates
            float dis,ang;
            float alph,beta;
            alph=atan2(robot_orient.coeffRef(1),robot_orient.coeffRef(0));
            beta=atan2(obs.coeffRef(1),obs.coeffRef(0));
            ang=beta-alph;
            dis=obs_dir.norm();
            //ROS_INFO("in dis=%.2f,ang=%.2f",dis,ang);
            
            if(dis<whosyourdaddy.warning_distance && ang>-0.85 && ang<0.85)
            {
              whosyourdaddy.append(whosyourdaddy.warning_point,dis,ang);
              addVizPoint(obs.coeffRef(0),obs.coeffRef(1));
              //whosyourdaddy.simple_vec(ang);
              //ROS_INFO("in dis=%.2f,ang=%.2f",dis,ang);
            }
          /*---------------------------------------------------*/
          }
        }
      }
      //visualization
      geometry_msgs::PoseStamped point_of_odomframe;
      point_of_odomframe.header.frame_id="odom";
      geometry_msgs::PoseStamped car_of_odom_frame;
      car_of_odom_frame.header.frame_id="odom";

      geometry_msgs::PoseStamped point_of_carframe;
      point_of_carframe.header.frame_id="base_footprint";
      geometry_msgs::PoseStamped car_of_carframe;
      car_of_carframe.header.frame_id="base_footprint";

      point_of_odomframe.header.stamp=car_of_odom_frame.header.stamp=point_of_carframe.header.stamp=car_of_carframe.header.stamp=ros::Time::now();  
      try
      {
          point_of_carframe.pose.position.x=1*cos(whosyourdaddy.out_point.ang);//the point in car_frame that should be visualized
          point_of_carframe.pose.position.y=1*sin(whosyourdaddy.out_point.ang);//
          point_of_carframe.pose.orientation.w=1.0;
          tf_listener.transformPose("odom", ros::Time(0) , point_of_carframe, "base_footprint" ,point_of_odomframe);
          
          car_of_carframe.pose.position.x=0;//the point in car_frame that should be visualized
          car_of_carframe.pose.position.y=0;//
          car_of_carframe.pose.orientation.w=1.0;
          tf_listener.transformPose("odom", ros::Time(0) , car_of_carframe, "base_footprint" ,car_of_odom_frame);
          
      }
      catch(tf::TransformException &ex)
      {
          ROS_ERROR("%s tf error in rsband",ex.what());
          ros::Duration(1.0).sleep();
      }
      addVizLine(car_of_odom_frame.pose.position.x,car_of_odom_frame.pose.position.y,point_of_odomframe.pose.position.x,point_of_odomframe.pose.position.y);
      // addVizPoint(point_of_odomframe.pose.position.x,point_of_odomframe.pose.position.y);
      show_obst();
      points.points.clear();

    //finally output a vector called whosyourdaddy.out_point ? how to transform it into adjust_angular?
      whosyourdaddy.sortlist( whosyourdaddy.warning_point);
      whosyourdaddy.last_point = whosyourdaddy.getlastnode( whosyourdaddy.warning_point);
      if( whosyourdaddy.last_point->distance == LINK_HEAD_D || whosyourdaddy.last_point->distance == LINK_HEAD_A) 
       {
         printf("null\n");whosyourdaddy.out_point.ang=0;
       }
      else
      {
        whosyourdaddy.v_vector( whosyourdaddy.last_point);
      }
      //whosyourdaddy.output( whosyourdaddy.warning_point);
      
      ROS_INFO("output angle=%.2f",whosyourdaddy.out_point.ang);
      float out_ang=whosyourdaddy.out_point.ang;
      rectified_angular=map(out_ang,-1.58,1.58,45,145);
      
      return rectified_angular;
      
  }

}  // namespace rsband_local_planner
