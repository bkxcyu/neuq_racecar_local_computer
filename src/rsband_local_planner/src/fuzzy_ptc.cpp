/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kouros
*********************************************************************/

#include "rsband_local_planner/fuzzy_ptc.h"

#include <ros/package.h>
#include <math.h>
#include <tf/tf.h>
#include <boost/foreach.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
  double rad2deg(double rad)
  {
    return rad * 180.0 / M_PI;
  }

  double deg2rad(double deg)
  {
    return deg * M_PI / 180.0;
  }
}

namespace rsband_local_planner
{

  FuzzyPTC::FuzzyPTC(std::string name) : initialized_(false)
  {
    pnh_ = new ros::NodeHandle("~/" + name);
    initialize();
  }


  FuzzyPTC::~FuzzyPTC()
  {
    delete engine_;
    // no need to delete fuzzy variables and ruleblock, since it is done in
    // the destructor of the engine
  }


  void FuzzyPTC::initialize()
  {
    if (initialized_)
    {
      ROS_ERROR("Controller already initialized!");
      return;
    }

    // load fuzzy rules
    if (pnh_->hasParam("fuzzy_rules"))  // load from param server if available
    {
      if (pnh_->getParam("fuzzy_rules/speed_rules", speedRules_))
      {
        ROS_FATAL("Failed to load speed fuzzy rules!");
        exit(EXIT_FAILURE);
      }

      if (pnh_->getParam("fuzzy_rules/steering_rules",
          SteeringRules_))
      {
        ROS_FATAL("Failed to load front steering fuzzy rules!");
        exit(EXIT_FAILURE);
      }

      if (pnh_->getParam("fuzzy_rules/Lfw_rules",
          Lfw_rules_))
        ROS_WARN("Failed to load rear steering deviation fuzzy rules!");
    }
    else  // load from yaml file in pkg
    {
      std::string pkgPath = ros::package::getPath("rsband_local_planner");
      YAML::Node config = YAML::LoadFile(
        pkgPath + "/cfg/path_tracking_controller_fuzzy_rules.yaml");

      speedRules_ =
        config["fuzzy_rules"]["speed_rules"].as< std::vector<std::string> >();
      SteeringRules_ =
        config["fuzzy_rules"]["steering_rules"].as<
        std::vector<std::string> >();
      Lfw_rules_ =
        config["fuzzy_rules"]["Lfw_rules"].as<
        std::vector<std::string> >();
    }

    // initialize sub goal msg and publisher
    // subGoal_.type = visualization_msgs::Marker::SPHERE;
    // subGoal_.scale.x = subGoal_.scale.y = subGoal_.scale.z = 0.1;
    // subGoal_.color.b = 1.0f; subGoal_.color.a = 1;
    // subGoalPub_ = pnh_->advertise<visualization_msgs::Marker>("sub_goal", 1);
    
    initialized_ = true;
     
  }


  void FuzzyPTC::reconfigure(RSBandPlannerConfig& config)
  {

       SMOOTHNESS = config.SMOOTHNESS;

    // maxSteeringAngle_ = config.max_steering_angle;
    maxSpeed_ = config.max_speed;
    // xyGoalTolerance_ = config.xy_goal_tolerance;
    // yawGoalTolerance_ = config.yaw_goal_tolerance;
    // latDevTolerance_ = config.lateral_deviation_tolerance;
    // updateSubGoalDistThreshold_ = config.update_sub_goal_dist_threshold;
    // goalDistThreshold_ = config.goal_dist_threshold;
    // displayControllerIO_ = config.display_controller_io;
    // stop_ = config.stop;
    // rearSteeringMode_ =
    //   static_cast<RearSteeringMode>(config.rear_steering_mode);

    // reinitialize fuzzy engine with the updated parameters
    ROS_INFO("begin to initializeFuzzyEngine...............");
    initializeFuzzyEngine();
  }


  void FuzzyPTC::initializeFuzzyEngine()
  {
    if (!initialized_)
    {
      ROS_ERROR("Fuzzy Engine Initialization attempted before controller "
        "initialization");
      return;
    }
    ROS_INFO("-----------   1  ------------");
    // initialize FLC engine
    engine_ = new fl::Engine();
    engine_->setName("fuzzy_path_tracking_controller");

    //========================= INPUT VARIABLES ================================
ROS_INFO("-----------   2  ------------");
    // direction input variable
    smoothness_ = new fl::InputVariable;
    smoothness_->setEnabled(true);
    smoothness_->setName("Smoothness");
    smoothness_->setRange(0.0, 3.0);
    smoothness_->addTerm(new fl::Trapezoid("Smooth", 0.0, 0.0, 0.9, 1.1));
    smoothness_->addTerm(new fl::Trapezoid("Normal", 0.9, 1.1, 1.9, 2.1));
    smoothness_->addTerm(new fl::Trapezoid("Rough",  1.9, 2.1, 3.0, 3.0));
    engine_->addInputVariable(smoothness_);
ROS_INFO("-----------   3  ------------");
    // angle deviation error fuzzy input variable initialization
    angularDeviationError_ = new fl::InputVariable;
    angularDeviationError_->setEnabled(true);
    angularDeviationError_->setName("AngleErr");
    angularDeviationError_->setRange(-90, 90);
    angularDeviationError_->addTerm(new fl::Trapezoid("LHigh_a",   -90.0, -90.0, -64.0, -44.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("LMedium_a", -64.0, -44.0, -28.0, -20.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("LLow_a",    -28.0, -20.0,   0.0,   0.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("RLow_a",     0.0,    0.0,  20.0,  28.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("RMedium_a",  20.0,  28.0,  44.0,  64.0));
    angularDeviationError_->addTerm(new fl::Trapezoid("RHigh_a",    44.0,  64.0,  90.0,  90.0));
    engine_->addInputVariable(angularDeviationError_);
ROS_INFO("-----------   4  ------------");
    orientationError__ = new fl::InputVariable;
    orientationError__->setEnabled(true);
    orientationError__->setName("OrientationErr_");
    orientationError__->setRange(-180.0, 180.0);
    orientationError__->addTerm(new fl::Trapezoid("LHigh_o_",   -180.0, -180.0, -120.0, -90.0));
    orientationError__->addTerm(new fl::Trapezoid("LMedium_o_", -120.0,  -90.0,  -35.0, -25.0));
    orientationError__->addTerm(new fl::Trapezoid("LLow_o_",     -35.0,  -25.0,    0.0,   0.0));
    orientationError__->addTerm(new fl::Trapezoid("RLow_o_",       0.0,    0.0,   25.0,  35.0));
    orientationError__->addTerm(new fl::Trapezoid("RMedium_o_",   25.0,   35.0,   90.0, 120.0));
    orientationError__->addTerm(new fl::Trapezoid("RHigh_o_",     90.0,  120.0,  180.0, 180.0));
    engine_->addInputVariable(orientationError__);
ROS_INFO("-----------   5  ------------");
    // orientation error input variable initialization
    orientationError_ = new fl::InputVariable;
    orientationError_->setEnabled(true);
    orientationError_->setName("OrientationErr");
    orientationError_->setRange(0.0, 180.0);
    orientationError_->addTerm(new fl::Trapezoid("Low_o",     0.0,  0.0, 20.0, 25.0));
    orientationError_->addTerm(new fl::Trapezoid("Medium_o", 20.0, 25.0,   55,   60));
    orientationError_->addTerm(new fl::Trapezoid("High_o",     55,   60,  180,  180));
    engine_->addInputVariable(orientationError_);
ROS_INFO("-----------   6  ------------");
    // Integrall error input variable initialization
    integrallError_ = new fl::InputVariable;
    integrallError_->setEnabled(true);
    integrallError_->setName("IntegrallErr");
    integrallError_->setRange(0, 100.0);
    integrallError_->addTerm(new fl::Trapezoid("Low_i",    0.0, 0.0, 5.0, 7.0));
    integrallError_->addTerm(new fl::Trapezoid("Medium_i", 7.0, 10,  20,  25));
    integrallError_->addTerm(new fl::Trapezoid("High_i",    20, 25, 100, 100));
    engine_->addInputVariable(integrallError_);
ROS_INFO("-----------   7  ------------");
    // currantSpeed_ error input variable initialization
    currantSpeed_ = new fl::InputVariable;
    currantSpeed_->setEnabled(true);
    currantSpeed_->setName("currantSpeed");
    currantSpeed_->setRange(0, 5.0);
    currantSpeed_->addTerm(new fl::Trapezoid("LLLow_c", 0.0, 0.0, 0.5, 0.9));
    currantSpeed_->addTerm(new fl::Trapezoid("LLow_c",  0.5, 0.9, 1.2, 1.6));
    currantSpeed_->addTerm(new fl::Trapezoid("Low_c",   1.2, 1.6, 1.9, 2.3));
    currantSpeed_->addTerm(new fl::Trapezoid("Medium_c",1.9, 2.3, 2.6, 3.5));
    currantSpeed_->addTerm(new fl::Trapezoid("Fast_c",  2.6, 3.5, 3.7, 4.0));
    currantSpeed_->addTerm(new fl::Trapezoid("FFast_c", 3.7, 4.0, 4.2, 4.4));
    currantSpeed_->addTerm(new fl::Trapezoid("FFFast_c",4.2, 4.4, 5.0, 5.0));
    engine_->addInputVariable(currantSpeed_);

    //========================= OUTPUT VARIABLES ===============================
ROS_INFO("-----------   8  ------------");
    // front steering angle output variable initialization
    steeringAngle_ = new fl::OutputVariable;
    steeringAngle_->setEnabled(true);
    steeringAngle_->setName("steeringAngle");
    double baseAngle=95.0;
    steeringAngle_->setRange(baseAngle-30,baseAngle+30);
    steeringAngle_->fuzzyOutput()->setAggregation(fl::null);
    steeringAngle_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    steeringAngle_->setDefaultValue(baseAngle);
    steeringAngle_->setLockPreviousValue(true);
    steeringAngle_->setLockValueInRange(false);
    steeringAngle_->addTerm(new fl::Constant("RHH", baseAngle-60.0));
    steeringAngle_->addTerm(new fl::Constant("RH",  baseAngle-30.0));
    steeringAngle_->addTerm(new fl::Constant("RM",  baseAngle-14.0));
    steeringAngle_->addTerm(new fl::Constant("RL",  baseAngle-7.0));
    steeringAngle_->addTerm(new fl::Constant("Z",   baseAngle));
    steeringAngle_->addTerm(new fl::Constant("LL",  baseAngle+7.0));
    steeringAngle_->addTerm(new fl::Constant("LM",  baseAngle+14.0));
    steeringAngle_->addTerm(new fl::Constant("LH",  baseAngle+30.0));
    steeringAngle_->addTerm(new fl::Constant("LHH", baseAngle+60.0));
    engine_->addOutputVariable(steeringAngle_);
ROS_INFO("-----------   9  ------------");
    // rear steering angle output variable initialization
    Lfw_ = new fl::OutputVariable;
    Lfw_->setEnabled(true);
    Lfw_->setName("Lfw");
    Lfw_->setRange(1, 6);
    Lfw_->fuzzyOutput()->setAggregation(fl::null);
    Lfw_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    Lfw_->setDefaultValue(1.0);
    Lfw_->setLockPreviousValue(true);
    Lfw_->setLockValueInRange(false);
    Lfw_->addTerm(new fl::Constant("SShort_L", 0.5));
    Lfw_->addTerm(new fl::Constant("Short_L",  1.2));
    Lfw_->addTerm(new fl::Constant("Medium_L", 1.8));
    Lfw_->addTerm(new fl::Constant("Long_L",   2.5));
    Lfw_->addTerm(new fl::Constant("LLong_L",  3.0));
    engine_->addOutputVariable(Lfw_);
ROS_INFO("-----------   10  ------------");
    // speed output variable initialization
    speed_ = new fl::OutputVariable;
    speed_->setEnabled(true);
    speed_->setName("Speed");
    speed_->setDefaultValue(0.0);
    // maxSpeed_=3;
    speed_->setRange(0.0, maxSpeed_);
    speed_->fuzzyOutput()->setAggregation(fl::null);
    speed_->setDefuzzifier(new fl::WeightedAverage("TakagiSugeno"));
    speed_->addTerm(new fl::Constant("LLLow",    maxSpeed_ / 7));
    speed_->addTerm(new fl::Constant("LLow",   2*maxSpeed_ / 7));
    speed_->addTerm(new fl::Constant("Low",    3*maxSpeed_ / 7));
    speed_->addTerm(new fl::Constant("Medium", 4*maxSpeed_ / 7));
    speed_->addTerm(new fl::Constant("Fast",   5*maxSpeed_ / 7));
    speed_->addTerm(new fl::Constant("FFast",  6*maxSpeed_ / 7));
    speed_->addTerm(new fl::Constant("FFFast",       maxSpeed_));
    engine_->addOutputVariable(speed_);


    //=============================== RULES ====================================
ROS_INFO("-----------   11  ------------");
    // rule block initialization
    ruleBlock_ = new fl::RuleBlock;
ROS_INFO("-----------   12  ------------");
    // steering rules
    BOOST_FOREACH(std::string rule, SteeringRules_)
      ruleBlock_->addRule(fl::Rule::parse(rule, engine_));
ROS_INFO("-----------   13  ------------");
    // rear steering rules
    BOOST_FOREACH(std::string rule, Lfw_rules_)
      ruleBlock_->addRule(fl::Rule::parse(rule, engine_));
ROS_INFO("-----------   14  ------------");
    // speed rules
    BOOST_FOREACH(std::string rule, speedRules_)
      ruleBlock_->addRule(fl::Rule::parse(rule, engine_));
ROS_INFO("-----------   15  ------------");
    engine_->addRuleBlock(ruleBlock_);
    engine_->configure(
      "Minimum", "Maximum", "Minimum", "Maximum", "WeightedAverage", "General");
ROS_INFO("-----------   16  ------------");
    std::string status;
    if (!engine_->isReady(&status))
        throw fl::Exception("Engine not ready. "
          "The following errors were encountered:\n" + status, FL_AT);
    ROS_INFO("fuzzy is alredy");
  }


  bool FuzzyPTC::computeVelocityCommands(
    const double& CURRANT_SPEED,
    double& output_Lfw)
  {   

    smoothness_->setValue(SMOOTHNESS);
    currantSpeed_->setValue(CURRANT_SPEED);

    engine_->process();

    output_Lfw = Lfw_->getValue();  

    return true;
  }

  bool FuzzyPTC::computeVelocityCommands(
    const double& ANGULAR_ERR,const double& ORIENTATION_ERR,const double& INTEGRALL_ERR,
    double& output_vel)
  {   

    // smoothness_->setValue(SMOOTHNESS);
    angularDeviationError_->setValue(ANGULAR_ERR);
    orientationError_->setValue(fabs(ORIENTATION_ERR));
    orientationError__->setValue(ORIENTATION_ERR);
    integrallError_->setValue(fabs(INTEGRALL_ERR));
    // currantSpeed_->setValue(CURRANT_SPEED);

    // ROS_INFO("input/nANGULAR_ERR=%.2f  ORIENTATION_ERR=%.2f INTEGRALL_ERR=%.2f   CURRANT_SPEED=%.2f",ANGULAR_ERR,ORIENTATION_ERR,INTEGRALL_ERR,CURRANT_SPEED);
   
    engine_->process();

    output_vel= speed_->getValue();  
    // double output_angle = steeringAngle_->getValue(); 
    // output_Lfw = Lfw_->getValue();  

    // ROS_INFO("output_vel=%.2f  output_angle=%.2f output_lfw=%.2f",output_vel,output_angle,output_Lfw);
   

    
    // if (std::isnan(cmd.linear.x) or std::isnan(cmd.linear.y))
    // {
    //   ROS_ERROR("Speed=Nan. Something went wrong!");
    //   cmd.linear.x = 0.0;
    //   cmd.linear.y = 0.0;
    //   return false;
    // }
    // if (std::isnan(cmd.angular.z))
    // {
    //   ROS_ERROR("RotVel=Nan. Something went wrong!");
    //   cmd.angular.z = 0.0;
    //   return false;
    // }

    // if (stop_)
    // {
    //   cmd.linear.x = 0.0;
    //   cmd.linear.y = 0.0;
    //   cmd.angular.z = 0.0;
    // }

    return true;
  }


  bool FuzzyPTC::isGoalReached(
    const std::vector<geometry_msgs::PoseStamped>& path)
  {
    bool positionReached =
      calcPositionError(path, path.size()-1) < xyGoalTolerance_;
    bool orientationReached =
      fabs(calcOrientationError(path, path.size()-1)) < yawGoalTolerance_;
    return positionReached && orientationReached;
  }


  double FuzzyPTC::calcAngularDeviationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return atan2(path[subGoalIdx].pose.position.y,
      path[subGoalIdx].pose.position.x);
  }


  double FuzzyPTC::calcOrientationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return tf::getYaw(path[subGoalIdx].pose.orientation);
  }


  double FuzzyPTC::calcPositionError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return hypot(path[subGoalIdx].pose.position.x, path[subGoalIdx].pose.position.y);
  }


  double FuzzyPTC::calcLateralDeviationError(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int subGoalIdx)
  {
    return calcPositionError(path, subGoalIdx)
      * sin(calcAngularDeviationError(path, subGoalIdx));
  }


  double FuzzyPTC::calcDistance(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int idx1, unsigned int idx2)
  {
    double x0, x1, y0, y1;
    x0 = path[idx1].pose.position.x;
    y0 = path[idx1].pose.position.y;
    x1 = path[idx2].pose.position.x;
    y1 = path[idx2].pose.position.y;
    return hypot(x1 - x0, y1 - y0);
  }


  unsigned int FuzzyPTC::findSubGoal(
    const std::vector<geometry_msgs::PoseStamped>& path)
  {
    unsigned int subGoalIdx = 0;
    double distance = 0;

    while (subGoalIdx < path.size()-1 && distance < updateSubGoalDistThreshold_)
    {
      distance += calcDistance(path, subGoalIdx, subGoalIdx+1);
      subGoalIdx++;

      if (isCuspPoint(path, subGoalIdx)
        && calcPositionError(path, subGoalIdx) > xyGoalTolerance_)
      {
        break;
      }
    }

    subGoal_.header = path.front().header;
    subGoal_.pose = path[subGoalIdx].pose;
    subGoalPub_.publish(subGoal_);

    return subGoalIdx;
  }


  bool FuzzyPTC::isCuspPoint(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int idx)
  {
    if (idx < 1 || idx >= path.size() - 2)
      return false;

    geometry_msgs::Point a, b;
    a.x = path[idx].pose.position.x - path[idx-1].pose.position.x;
    a.y = path[idx].pose.position.y - path[idx-1].pose.position.y;
    b.x = path[idx].pose.position.x - path[idx+1].pose.position.x;
    b.y = path[idx].pose.position.y - path[idx+1].pose.position.y;

    double angle = acos((a.x * b.x + a.y * b.y) / hypot(a.x, a.y)
      / hypot(b.x, b.y));

    return !std::isnan(angle) && (fabs(angle) < 1.0);// && (fabs(angle) > 0.085);
  }

}
