/**
 * @file pointcloud2_filter_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 滤除自身之外的点云
 * @version 0.1
 * @date 2026-01-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

class VehiclePointCloudFilter {
public:
  VehiclePointCloudFilter() : tf_listener_(nh_) {
    ros::NodeHandle nh_private("~");
    // 读取盒子尺寸
    nh_private.param<double>("min_x", min_x_, -0.4);
    nh_private.param<double>("max_x", max_x_, 0.4);
    nh_private.param<double>("min_y", min_y_, -0.3);
    nh_private.param<double>("max_y", max_y_, 0.3);
    nh_private.param<double>("min_z", min_z_, -0.1);
    nh_private.param<double>("max_z", max_z_, 0.6);
    nh_private.param<bool>("negative", negative_, true);
    nh_private.param<std::string>("target_frame", target_frame_, "base_link");

    ROS_INFO("Vehicle filter box: x[%.2f ~ %.2f], y[%.2f ~ %.2f], z[%.2f ~ "
             "%.2f], negative=%s, target_frame=%s",
             min_x_, max_x_, min_y_, max_y_, min_z_, max_z_,
             negative_ ? "true" : "false", target_frame_.c_str());

    //  订阅输入点云
    sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
        "/livox/pointcloud2", 1, &VehiclePointCloudFilter::cloudCallback, this);

    // 发布滤除之后的点云
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "/livox/pointcloud2_filtered", 1);

    // 发布滤除盒
    marker_pub_ =
        nh_.advertise<visualization_msgs::Marker>("crop_box_marker", 10);

    timer_ = nh_.createTimer(ros::Duration(1.0),
                             &VehiclePointCloudFilter::publishMarker, this);
  }

private:
  void publishMarker(const ros::TimerEvent &) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "vehicle_body";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (max_x_ + min_x_) / 2.0;
    marker.pose.position.y = (max_y_ + min_y_) / 2.0;
    marker.pose.position.z = (max_z_ + min_z_) / 2.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_x_ - min_x_;
    marker.scale.y = max_y_ - min_y_;
    marker.scale.z = max_z_ - min_z_;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.4;

    marker.lifetime = ros::Duration(0);

    marker_pub_.publish(marker);
  }

  // 点云回调
  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input_msg) {
    // 直接在原始坐标系（雷达 mid360）下滤波

    pcl::PCLPointCloud2::Ptr cloud_in(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*input_msg, *cloud_in);

    // CropBox 滤波（参数就是相对于雷达的偏移）
    pcl::CropBox<pcl::PCLPointCloud2> crop;
    crop.setInputCloud(cloud_in);

    Eigen::Vector4f min_pt, max_pt;
    min_pt << min_x_, min_y_, min_z_, 1.0;
    max_pt << max_x_, max_y_, max_z_, 1.0;

    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.setNegative(negative_);

    pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
    crop.filter(*cloud_filtered);

    // 输出保持原 frame_id（mid360）
    sensor_msgs::PointCloud2 output_msg;
    pcl_conversions::fromPCL(*cloud_filtered, output_msg);
    output_msg.header = input_msg->header; // frame_id 还是 mid360

    pub_.publish(output_msg);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::Publisher marker_pub_;
  ros::Timer timer_;
  tf::TransformListener tf_listener_;

  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  bool negative_;
  std::string target_frame_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vehicle_pointcloud_filter");
  VehiclePointCloudFilter filter;
  ros::spin();
  return 0;
}