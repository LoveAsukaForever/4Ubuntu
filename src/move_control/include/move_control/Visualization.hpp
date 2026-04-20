/**
 * @file Visualization.hpp
 * @author Keten (2863861004@qq.com)
 * @brief 调用rviz接口可视化
 * @version 0.1
 * @date 2026-01-21
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace visualization {

enum class ShapeType {
  Sphere,
  Cube,
  Cylinder,
  Arrow,
  Point, // 小点
  Text,
};
class Visualization {
  // 可视化类型

public:
  //   Visualization();
  Visualization(const ros::NodeHandle &nh);
  ~Visualization() = default;

  //   永久显示点
  void visualizePoint(double pos_x, double pos_y, double pos_z, ShapeType shape,
                      double scale_x = 0.3, double scale_y = 0.3,
                      double scale_z = 0.3, float r = 1.0f, float g = 0.0f,
                      float b = 0.0f, float a = 1.0f,
                      const std::string &ns = "", const int id = 0,
                      const std::string &frame_id = "world");
  //   移除某个点
  void removeMarker(const std::string &ns = "", const int id = 0);

  //   移除命名空间内的所有Marker
  void removeMarkers(const std::string &ns = "");

private:
  inline uint8_t toMarkerType(ShapeType type) {
    switch (type) {
    case ShapeType::Sphere:
      return visualization_msgs::Marker::SPHERE;
    case ShapeType::Cube:
      return visualization_msgs::Marker::CUBE;
    case ShapeType::Cylinder:
      return visualization_msgs::Marker::CYLINDER;
    case ShapeType::Arrow:
      return visualization_msgs::Marker::ARROW;
    case ShapeType::Point:
      return visualization_msgs::Marker::SPHERE; // 小球模拟点
    case ShapeType::Text:
      return visualization_msgs::Marker::TEXT_VIEW_FACING;
    default:
      return visualization_msgs::Marker::SPHERE;
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
};

} // namespace visualization