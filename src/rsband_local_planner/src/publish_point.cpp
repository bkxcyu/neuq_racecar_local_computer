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
       void obstrCB(const geometry_msgs::PointStamped::ConstPtr& obstrMsg);
       ros::NodeHandle _n;
       ros::Subscriber obstr_sub;
       ros::Publisher obst_marker_pub_;
       geometry_msgs::PointStamped obstr;
       visualization_msgs::Marker each_Obstr_marker;
       visualization_msgs::MarkerArray obst_markers_array;
       
    };
    OBSTR::OBSTR()
    {
        obstr_sub = _n.subscribe("/clicked_point", 20, &OBSTR::obstrCB, this);  
        obst_marker_pub_ = _n.advertise<visualization_msgs::MarkerArray>("obst_markers", 20);
        initMarker();
    }
   
    void OBSTR::initMarker()
    {
        // int k =0;
        // while (k<20)
        // {
            each_Obstr_marker.header.frame_id =  "odom";
            each_Obstr_marker.ns =  "Markers";
            each_Obstr_marker.action = visualization_msgs::Marker::ADD;
            each_Obstr_marker.pose.orientation.w  = 1.0;
            each_Obstr_marker.id = 0;
            each_Obstr_marker.type = visualization_msgs::Marker::POINTS;
            // obstr markers use x and y scale for width/height respectively
            each_Obstr_marker.scale.x = 4;
            each_Obstr_marker.scale.y = 4;
            // each_Obstr_marker.scale.z=4; 
            // obstr are green
            each_Obstr_marker.color.g = 1.0f;
            each_Obstr_marker.color.a = 1.0;
            // k++;
        // }

    }
    void OBSTR::obstrCB(const geometry_msgs::PointStamped::ConstPtr& obstrMsg)
    {
        obstr = *obstrMsg;
        ROS_INFO("RECEIVE:%.2f %.2f %.2f",obstr.point.x,obstr.point.y,obstr.point.z);
        // obstr_marker_pub_.publish(ObstacleMarker);
        //可视化在这里实现
        each_Obstr_marker.points.clear();
        each_Obstr_marker.points.push_back(obstr.point);
        each_Obstr_marker.id++;
        obst_markers_array.markers.push_back(each_Obstr_marker);
        obst_marker_pub_.publish(obst_markers_array);
    }
    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "point");
        OBSTR obstrcte;
        ros::NodeHandle _n;
        // ros::Subscriber sub = _n.subscribe("/clicked_point",20, obstrCB);
        ros::spin();
    }

