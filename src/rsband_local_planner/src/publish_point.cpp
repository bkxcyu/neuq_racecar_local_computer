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



    class OBSTR
    {
    public:
        OBSTR();
        // ~OBSTR();
        void initMarker();
        
    
    private:
       void obstrCB(const geometry_msgs::PointStamped& obstrMsg);
       ros::NodeHandle _n;
       ros::Subscriber obstr_sub;
       ros::Publisher obstr_marker;
       geometry_msgs::PointStamped obstr;
       visualization_msgs::Marker Obstr;
    };
    OBSTR::OBSTR()
    {
        obstr_sub = _n.subscribe("/clicked_point", 20, &OBSTR::obstrCB, this);  
        // obst_marker_pub_ = _n.advertise<visualization_msgs::Marker>("obst_markers", 20);
        initMarker();
    }
   
    void OBSTR::initMarker()
    {
        Obstr.header.frame_id =  "odom";
        Obstr.ns =  "Markers";
        Obstr.action = visualization_msgs::Marker::ADD;
        Obstr.pose.orientation.w  = 1.0;
        Obstr.id = 0;
        Obstr.type = visualization_msgs::Marker::CUBE;
        // obstr markers use x and y scale for width/height respectively
        Obstr.scale.x = 0.4;
        Obstr.scale.y = 0.4; 
        // obstr are green
        Obstr.color.g = 1.0f;
        Obstr.color.a = 1.0;

    }
    void OBSTR::obstrCB(const geometry_msgs::PointStamped& obstrMsg)
    {
        obstr = obstrMsg;
        // obstr_marker_pub_.publish(ObstacleMarker);
        //可视化在这里实现
        Obstr.pose.position = obstr.point;
        obstr_marker.publish(Obstr);
    }
    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "point");
        OBSTR obstrcte;
        ros::NodeHandle _n;
        // ros::Subscriber sub = _n.subscribe("/clicked_point",20, obstrCB);
        ros::spin();
    }

