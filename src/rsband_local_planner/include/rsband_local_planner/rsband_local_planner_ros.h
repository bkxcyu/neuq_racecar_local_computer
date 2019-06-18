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

#ifndef RSBAND_LOCAL_PLANNER_RSBAND_LOCAL_PLANNER_ROS_H
#define RSBAND_LOCAL_PLANNER_RSBAND_LOCAL_PLANNER_ROS_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

#include <nav_core/base_local_planner.h>
#include "eband_local_planner/eband_local_planner.h"
#include "rsband_local_planner/reeds_shepp_planner.h"
#include "rsband_local_planner/fuzzy_ptc.h"

#include <dynamic_reconfigure/server.h>
#include "rsband_local_planner/RSBandPlannerConfig.h"

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include "rsband_local_planner/neuq_controller.h"

namespace rsband_local_planner
{

  /**
   * @class RSBandPlannerROS
   * @brief A local planner plugin for  move_base of ROS, for car like robots
   */
  class RSBandPlannerROS : public nav_core::BaseLocalPlanner
  {
    public:

      /**
       * @brief Constructor
       */
      RSBandPlannerROS();

      /**
       * @brief Destructor
       */
      ~RSBandPlannerROS();

      /**
       * @brief Initializes planner
       * @param name: The name of the planner
       * @param tfListener: ptr to a tf transform listener
       * @param costmapROS: ptr to a costmap ros wrapper
       */
      void initialize(std::string name, tf::TransformListener* tfListener,
        costmap_2d::Costmap2DROS* costmapROS);

      /**
       * @brief Sets the global plan to be followed by this planner
       * @param globalPlan: The plan to follow
       * @return true if plan was set successfully
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& globalPlan);

      /**
       * @brief Computes the velocity command for the robot base
       * @param cmd: The computed command container
       * @return true if a cmd was computed successfully
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd);

      /**
       * @brief Checks whether the goal is reached
       * @return true if goal is reached
       */
      bool isGoalReached();


      //! eband to rs planning strategy enum
      enum EbandToRSStrategy
      {
        startToEnd,
        startToNext,
        pointToPoint,
        skipFailures,
        startToRecedingEnd
      };

    private:

      /**
       * @brief Interpolates pose orientations of the given plan
       * @param plan: The plan, for which to interpolate the poses
       */
      void interpolateOrientations(
        std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief Updates eband
       * @details Prunes the eband and adds new target frames
       * @return true if eband was updated successfully
       */
      bool updateEBand();

      /**
       * @brief Attempts an emergency plan in case actual plan failed
       * @param ebandPlan: the eband plan
       * @param emergencyPlan: the emergency plan container
       * @return true if emergency planning succeeds
       */
      bool emergencyPlan(std::vector<geometry_msgs::PoseStamped>& ebandPlan,
        std::vector<geometry_msgs::PoseStamped>& emergencyPlan);

      /**
       * @brief Reconfigures node parameters
       * @param config: The dynamic reconfigure node configuration
       * @param level: reconfiguration level
       */
      void reconfigureCallback(RSBandPlannerConfig& config, uint32_t level);

      typedef dynamic_reconfigure::Server<
        rsband_local_planner::RSBandPlannerConfig> drs;
      //! dynamic reconfigure server ptr
      boost::shared_ptr<drs> drs_;

      //! tf transform listener ptr
      tf::TransformListener* tfListener_;
      //! costmap ROS wrapper ptr
      costmap_2d::Costmap2DROS* costmapROS_;

      //! eband planner ptr
      boost::shared_ptr<eband_local_planner::EBandPlanner> ebandPlanner_;
      //! reeds shepp planner ptr
      boost::shared_ptr<ReedsSheppPlanner> rsPlanner_;
      //! path tracking controller ptr
      boost::shared_ptr<FuzzyPTC> ptc_;
      boost::shared_ptr<L1Controller> L1_;

      //! distance to goal tolerance
      double xyGoalTolerance_;
      //! angular deviation from goal pose tolerance
      double yawGoalTolerance_;
      //! dist threshold used when updating sub goal
      double updateSubGoalDistThreshold_;

      //! eband to reeds shepp band conversion strategy
      EbandToRSStrategy ebandToRSStrategy_;

      //!< determines whether emergency planning will be used in case of failure
      bool emergencyPlanning_;
      //!< emergency mode
      bool emergencyMode_;
      //!< emergency plan poses
      std::vector<geometry_msgs::PoseStamped> emergencyPoses_;


      //! global plan publisher
      ros::Publisher globalPlanPub_;
      //! local plan publisher
      ros::Publisher localPlanPub_;
      //! eband plan publisher
      ros::Publisher ebandPlanPub_;
      //! rs plan publisher
      ros::Publisher rsPlanPub_;

      //! global plan
      std::vector<geometry_msgs::PoseStamped> globalPlan_;
      //! transformed plan
      std::vector<geometry_msgs::PoseStamped> transformedPlan_;

      //! eband plan start and end indexes regarding global plan
      std::vector<int> planStartEndCounters_;

      bool initialized_;
  };

}  // namespace rsband_local_planner

#endif  // RSBAND_LOCAL_PLANNER_RSBAND_LOCAL_PLANNER_ROS_H
