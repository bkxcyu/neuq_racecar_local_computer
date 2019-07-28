#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  obst_sub = nh.subscribe("obst_markers", 1, &SimpleLayer::obstCB, this);//visualization_msgs::Marker

}



void SimpleLayer::obstCB(const visualization_msgs::MarkerArray& _obst)
{
  // std::vector<geometry_msgs::Point> obst_buffer;
  // obst_buffer=_obst.points;
  for(visualization_msgs::Marker each_marker:_obst.markers)
  {
    geometry_msgs::PointStamped each_pointSt;
    each_pointSt.header.frame_id=each_marker.header.frame_id;;
    each_pointSt.header.stamp = ros::Time();
    each_pointSt.point=each_marker.points.front();
    // try
    // {
    // geometry_msgs::PointStamped map_point;
    // tf_listener.transformPoint("/map", each_pointSt, map_point);
    obst.push_back(each_pointSt);
    // }
    // catch(tf::TransformException& ex)
    // {
    //   ROS_ERROR("tf err in simple layer: %s", ex.what());
    // }
  }
}



void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  mark_x_ = robot_x + 60*cos(robot_yaw);
  mark_y_ = robot_y + 60*sin(robot_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  // unsigned int mx;
  // unsigned int my;
  // if(master_grid.worldToMap(mark_x_, mark_y_, mx, my))
  //   master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  for(geometry_msgs::PointStamped each_point:obst)
    master_grid.setCost(each_point.point.x, each_point.point.y, LETHAL_OBSTACLE);
  


}

} // end namespace
