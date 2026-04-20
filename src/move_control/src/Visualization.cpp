/**
 * @file Visualization.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-01-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <move_control/Visualization.hpp>

namespace visualization {

Visualization::Visualization(const ros::NodeHandle &nh) {
  // 实例化
  nh_ = nh;
  marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}

void Visualization::visualizePoint(double pos_x, double pos_y, double pos_z,
                                   ShapeType shape, double scale_x,
                                   double scale_y, double scale_z, float r,
                                   float g, float b, float a,
                                   const std::string &ns, const int id,
                                   const std::string &frame_id) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = toMarkerType(shape);
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pos_x;
  marker.pose.position.y = pos_y;
  marker.pose.position.z = pos_z;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = scale_x;
  marker.scale.y = scale_y;
  marker.scale.z = scale_z;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;

  //   设置为永久
  marker.lifetime = ros::Duration(0);

  marker_pub_.publish(marker);
}

//   移除某个点
void Visualization::removeMarker(const std::string &ns, const int id) {
  visualization_msgs::Marker del;
  del.ns = ns;
  del.id = id;
  del.action = visualization_msgs::Marker::DELETE;
  marker_pub_.publish(del);
}

//   移除命名空间内的所有Marker
void Visualization::removeMarkers(const std::string &ns) {
  visualization_msgs::Marker del_all;
  del_all.ns = ns;
  del_all.action = visualization_msgs::Marker::DELETEALL;
  marker_pub_.publish(del_all);
}

} // namespace visualization