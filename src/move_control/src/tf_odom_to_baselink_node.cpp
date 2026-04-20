// clang-format off
/**
 * @file tf_odom_to_baselink_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 对slam模块输出做处理，使其可以满足导航模块需求
 * @version 0.1
 * @date 2026-01-27
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : fast-lio
 *         baselink_to_mid360 等效于odom_to_lidarodom
 *         这里订阅话题/lidar_odom 其实发布的是lidarodom_to_lidar，即/lidar在/lidarodom下的位姿表达，因此可以计算odom_to_lidar
 *         odom_to_lidar = odom_to_lidarodom * lidarodom_to_lidar
 *         拿到odom_to_lidar之后
 *         就可以用lidar_to_baselink,这个其实就是mid360_to_baselink,也就是baselink_to_mid360.inverse
 *         即odom_to_baselink = odom_to_lidar * lidar_to_baselink
 *         因此就可以用odom_to_baselink发布/odom 到 /base_link 的坐标变换
 *         还有就是雷达点云的变换，现在输入的/current_pcd 是/lidar_odom下的点云
 *         因此需要转到/odom下（todo）
 * @versioninfo :
 */
// clang-format on

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <visualization_msgs/Marker.h>

class OdomToTfNode {
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber lidar_odom_sub_;

  // 订阅输入点云
  ros::Subscriber pcd_sub_;
  ros::Publisher output_pcd_pub_;
  ros::Publisher odom_pub_;

  /* relative tf transform */
  tf2::Transform tf_base_to_lidar_;
  tf2::Transform tf_odom_to_lidarodom_;
  tf2::Transform tf_odom_to_lidar_;
  tf2::Transform tf_lidarodom_to_lidar_;
  tf2::Transform tf_odom_to_base_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  bool tf_initialized_ = false;

  // 参数
  std::string lidar_frame_;
  std::string base_frame_;

  // 可视化
  ros::Timer timer_;
  ros::Publisher marker_pub_;
  double radius_r_;
  std::string robot_frame_;

public:
  OdomToTfNode() : nh_private_("~") {
    // 从参数服务器读取 frame 名字
    nh_private_.param<std::string>("lidar_frame", lidar_frame_, "mid360");
    nh_private_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_private_.param<std::string>("robot_frame", robot_frame_, "odom");
    nh_private_.param<double>("radius", radius_r_, 0.44);

    ROS_INFO("OdomToTf node started: lidar_frame=%s, base_frame=%s",
             lidar_frame_.c_str(), base_frame_.c_str());

    // 订阅 /lidar_odom（雷达里程计）
    lidar_odom_sub_ = nh_.subscribe("/lidar_odom", 10,
                                    &OdomToTfNode::lidarodomCallback, this);
    pcd_sub_ =
        nh_.subscribe("/current_pcd", 10, &OdomToTfNode::pcdCallback, this);
    // 发布output点云
    output_pcd_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/output_pcd", 10);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

    // 可视化话题发布
    marker_pub_ =
        nh_.advertise<visualization_msgs::Marker>("robot_box_marker", 10);
    timer_ =
        nh_.createTimer(ros::Duration(1.0), &OdomToTfNode::publishMarker, this);

    /* tf2 init */
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>();

    // 启动时查询一次静态变换 lidar → base_link
    initializeLidarToBaseTf();
  }

  // 启动时一次性查询静态变换，检查 baselink -> lidar
  void initializeLidarToBaseTf() {

    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
          base_frame_, lidar_frame_, ros::Time(0), ros::Duration(5.0));
      tf2::fromMsg(tf_stamped.transform, tf_base_to_lidar_);
      tf_odom_to_lidarodom_ = tf_base_to_lidar_;
      tf_initialized_ = true;
      ROS_INFO("Successfully initialized static transform: %s → %s",
               base_frame_.c_str(), lidar_frame_.c_str());

    } catch (tf::TransformException &ex) {
      ROS_ERROR("Failed to lookup static transform %s → %s: %s",
                base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
      ROS_ERROR("Please ensure static transform is published (e.g. via "
                "static_transform_publisher in launch)");
    }
  }

  // lidar_odom 回调
  void lidarodomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    if (tf_initialized_) {
      // lidarodom_to_lidar
      tf2::fromMsg(msg->pose.pose, tf_lidarodom_to_lidar_);
      // 拿到 odom-> lidar
      tf_odom_to_lidar_ = tf_odom_to_lidarodom_ * tf_lidarodom_to_lidar_;
      // 拿到 odom -> base_link
      tf_odom_to_base_ = tf_odom_to_lidar_ * tf_base_to_lidar_.inverse();

      // 发布/odom到/base_link的坐标变换
      geometry_msgs::TransformStamped transform_msg;
      transform_msg.header.stamp = msg->header.stamp;
      transform_msg.header.frame_id = "/odom";
      transform_msg.child_frame_id = base_frame_;
      transform_msg.transform = tf2::toMsg(tf_odom_to_base_);
      br_->sendTransform(transform_msg);

      nav_msgs::Odometry odom_msg;
      odom_msg.header.stamp = msg->header.stamp;
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";
      odom_msg.pose.pose.position.x = tf_odom_to_base_.getOrigin().x();
      odom_msg.pose.pose.position.y = tf_odom_to_base_.getOrigin().y();
      odom_msg.pose.pose.position.z = tf_odom_to_base_.getOrigin().z();
      // 设置姿态（从 tf_odom_to_base_ 提取旋转部分）
      odom_msg.pose.pose.orientation =
          tf2::toMsg(tf_odom_to_base_.getRotation());
      odom_pub_.publish(odom_msg);

    } else {
      ROS_ERROR_THROTTLE(0.5, "[tf_odom_to_baselink] static tf is not sure!");
    }
  }

  void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    if (tf_initialized_) {
      pcl::PointCloud<pcl::PointXYZ> pcl_cloud_in, pcl_cloud_out;
      pcl::fromROSMsg(*msg, pcl_cloud_in);

      sensor_msgs::PointCloud2 output;
      geometry_msgs::Transform tf_lidar_to_lidarodom =
          tf2::toMsg(tf_lidarodom_to_lidar_.inverse());

      pcl_ros::transformPointCloud(pcl_cloud_in, pcl_cloud_out,
                                   tf_lidar_to_lidarodom);
      pcl_cloud_out.header.frame_id = "mid360";

      pcl::toROSMsg(pcl_cloud_out, output);

      output_pcd_pub_.publish(output);
    } else {
      ROS_ERROR_THROTTLE(0.5, "[tf_odom_to_baselink] static tf is not sure!");
    }
  }

  void publishMarker(const ros::TimerEvent &) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = robot_frame_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot_body";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE; // 设置为球体
    marker.action = visualization_msgs::Marker::ADD;
    // 设置球体的位置
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0; // 球体中心高度
    marker.pose.orientation.w = 1.0;

    // 设置球体的大小（直径）
    marker.scale.x = radius_r_; // x 方向直径
    marker.scale.y = radius_r_; // y 方向直径
    marker.scale.z = radius_r_; // z 方向直径

    // 设置球体的颜色
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0; // 蓝色
    marker.color.a = 1.0; // 不透明

    // 设置生命周期（0 表示永久显示）
    marker.lifetime = ros::Duration(0);

    // 发布 Marker
    marker_pub_.publish(marker);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "tf_odom_to_baselink_node");
  OdomToTfNode node;
  ros::spin();
  return 0;
}