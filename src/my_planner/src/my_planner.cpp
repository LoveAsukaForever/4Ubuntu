/**
 * @file my_planner.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-01-31
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : TEMP 宏定义用于临时使用，现在还是需要跟踪到目标姿态
 * @versioninfo :
 */

#include "my_planner/my_planner.h"

PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseLocalPlanner)

#define GLOBAL_REACHED_THRESHOLD 5

namespace my_planner {

// 不可以做ros提供的发布订阅等依赖于节点创建的操作，因为构造函数很可能先于节点的创建调用
MyPlanner::MyPlanner() {}

// 实时替换掉路径
geometry_msgs::Quaternion
MyPlanner::calculateRobotPoseToTarget(double robot_x, double robot_y,
                                      double target_x, double target_y) {
  // 计算目标点相对于机器人的偏移量
  double dx = target_x - robot_x;
  double dy = target_y - robot_y;

  // 计算目标点相对于机器人的方向角（yaw）
  double target_yaw = atan2(dy, dx);

  // 生成新的姿态四元数
  tf2::Quaternion q;
  q.setRPY(0, 0, target_yaw);

  // 填充机器人新的姿态
  geometry_msgs::Pose pose;

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose.orientation;
}

geometry_msgs::PoseStamped MyPlanner::getUserGoalPose() {
  if (global_plan_.empty()) {
    ROS_ERROR("global_plan_ is empty, no goal available");
    return geometry_msgs::PoseStamped();
  }

  // 最后一个点通常就是用户发送的目标点（或接近）
  return global_plan_.back();
}

void MyPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                           costmap_2d::Costmap2DROS *costmap_ros) {
  ROS_INFO("It's time to go!");

  tf_listener_ = std::make_unique<tf::TransformListener>();
  costmap_ros_ = costmap_ros;

  // 读取参数服务器
  private_nh_ = ros::NodeHandle("~" + name);
  private_nh_.param("max_linear_vel_x", max_linear_vel_x_, 0.5);
  private_nh_.param("max_linear_vel_y", max_linear_vel_y_, 0.5);
  private_nh_.param("max_angular_vel_z", max_angular_vel_z_, 0.5);
  ROS_INFO_STREAM("Sussceefully config the max parameters -- max_vx: "
                  << max_linear_vel_x_);
  ROS_INFO_STREAM("Sussceefully config the max parameters -- max_vy: "
                  << max_linear_vel_y_);
  ROS_INFO_STREAM("Sussceefully config the max parameters -- max_vz: "
                  << max_angular_vel_z_);

  // 读取pid参数设置
  double lx_p, lx_i, lx_d;
  private_nh_.param("linear_x_pid/p", lx_p, 2.0);
  private_nh_.param("linear_x_pid/i", lx_i, 0.0);
  private_nh_.param("linear_x_pid/d", lx_d, 0.5);

  double ly_p, ly_i, ly_d;
  private_nh_.param("linear_y_pid/p", ly_p, 2.0);
  private_nh_.param("linear_y_pid/i", ly_i, 0.0);
  private_nh_.param("linear_y_pid/d", ly_d, 0.5);

  double az_p, az_i, az_d;
  private_nh_.param("angular_z_pid/p", az_p, 2.0);
  private_nh_.param("angular_z_pid/i", az_i, 0.0);
  private_nh_.param("angular_z_pid/d", az_d, 0.5);

  // 设置pid
  linear_x_pid_ = std::make_unique<PID>(lx_p, lx_i, lx_d);
  linear_y_pid_ = std::make_unique<PID>(ly_p, ly_i, ly_d);
  angular_pid_ = std::make_unique<PID>(az_p, az_i, az_d);

  ROS_INFO_STREAM("Sussceefully config the pid parameters -- linear_x P: "
                  << lx_p << " I: " << lx_i << " D: " << lx_d);
  ROS_INFO_STREAM("Sussceefully config the pid parameters -- linear_y P: "
                  << ly_p << " I: " << ly_i << " D: " << ly_d);
  ROS_INFO_STREAM("Sussceefully config the pid parameters -- angular_z P: "
                  << az_p << " I: " << az_i << " D: " << az_d);

  private_nh_.param<std::string>("robot_frame", robot_frame_, "/base_link");
  private_nh_.param<std::string>("odom_frame", odom_frame_, "/odom");
  ROS_INFO_STREAM("Successfully config the robot frame -- "
                  << robot_frame_.c_str());
  ROS_INFO_STREAM("Successfully config the odom frame -- "
                  << odom_frame_.c_str());

  private_nh_.param<double>("forward_path_num", forward_path_num_, 10);
  private_nh_.param("goal_reached_threshold", goal_reached_threshold_, 5);
  private_nh_.param<double>("yaw_threshold", yaw_threshold_, 0.15);

  // Dyanmic_reconfigure 回调
  dsrv_ = std::make_unique<
      dynamic_reconfigure::Server<my_planner::MyPlannerConfig>>(private_nh_);
  dynamic_reconfigure::Server<my_planner::MyPlannerConfig>::CallbackType cb =
      [this](auto &config, auto level) { reconfigureCB(config, level); };
  dsrv_->setCallback(cb);

  last_cmd_time_ = ros::Time(0);
}

bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  target_index_ = 0; // 目标索引，只在发布新路径时重置
  global_plan_ = plan;
  // 拿到该次规划后的最后一个坐标
  final_target_ = getUserGoalPose();
  ROS_INFO("Received a plan with %lu poses.", global_plan_.size());
  return true;
}

bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  /* 获取代价地图的数据 */
  costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  unsigned char *map_data = costmap->getCharMap(); // 获取代价地图的栅格值数组
  unsigned int size_x =
      costmap->getSizeInCellsX(); // 获取代价地图的x方向的栅格数
  unsigned int size_y =
      costmap->getSizeInCellsY(); // 获取代价地图的y方向的栅格数

  // 使用opencv绘制代价地图
  /* 颜色规定：
      代价值  颜色     含义
      0      灰色     自由通行
      254    黑色     障碍物区域，机器人底盘投影不可进入
      253    浅蓝色   膨胀禁行区域，机器人底盘投影中心不可进入
      1~252  红蓝渐变  代价递减区域，机器人途径此区域会计算代价

     因此，只要点的代价值 >= 253 ,就是判断为该点有障碍物
   */

  // 在代价地图上遍历导航路径点
  for (int i = 0; i < global_plan_.size(); i++) {
    geometry_msgs::PoseStamped pose_odom;
    global_plan_[i].header.stamp = ros::Time(0);
    tf_listener_->transformPose(odom_frame_, global_plan_[i], pose_odom);
    double odom_x = pose_odom.pose.position.x;
    double odom_y = pose_odom.pose.position.y;

    /* 区分好：局部代价地图原点和底盘中心，局部代价地图中心点即 底盘中心的位置
     */

    double origin_x =
        costmap->getOriginX(); // 获取局部代价地图的原点在全局map中的坐标
    double origin_y = costmap->getOriginY();
    double local_x = odom_x - origin_x;
    double local_y = odom_y - origin_y;
    int x = local_x / costmap->getResolution();
    int y = local_y / costmap->getResolution();

    /* 检测前方路径点是否在禁行区域或者障碍物里 */
    if (i >= target_index_ &&
        i < target_index_ + forward_path_num_) // + 10 是什么意思
    {
      // 其实就是下一个路径点和往后的第十个路径点，中间所有路径点都会经过检查
      // 这段距离是多少呢？其实就是 10 * 地图分辨率
      // gmapping 建图的分辨率是 0.05
      // ，所以这里这么写就是说：给与半米的检测距离来做避障
      int map_index = y * size_x + x;
      unsigned char cost = map_data[map_index];
      if (cost >= 253)
        return false; // 返回false则表示局部规划器找不到可行的方案
    }
  }

  /* 速度规划器 */
  geometry_msgs::PoseStamped target_pose;
  for (int i = target_index_; i < global_plan_.size(); i++) {
    geometry_msgs::PoseStamped pose_base;
    global_plan_[i].header.stamp = ros::Time(0);
    tf_listener_->transformPose(robot_frame_, global_plan_[i], pose_base);
    double dx = pose_base.pose.position.x;
    double dy = pose_base.pose.position.y;
    double dist = sqrt(dx * dx + dy * dy);

    if (dist > 0.2) // 如果还没达到可以转换的位置
    {
      target_index_ = i;
      target_pose = pose_base;
      ROS_WARN_THROTTLE(
          0.5, "Select the %d point as the temp target,dist: %.2f", i, dist);
      break;
    }

    if (global_plan_.size() - 1 == i) // 如果已经到了最后一个点
    {
      target_pose = pose_base;
      ROS_WARN_THROTTLE(
          0.5, "Select the %d point as the final target,dist: %.2f", i, dist);
    }
  }

  // 计算步长
  ros::Time now = ros::Time::now();
  double dt = 0.0;
  if (first_compute_) {
    dt = 0.1; // 第一次用一个合理默认值（避免除零或初始积分爆炸）
    first_compute_ = false;
  } else {
    dt = (now - last_cmd_time_).toSec();
    if (dt <= 0.0 || dt > 1.0) { // 防止异常值（比如时间跳变或节点卡死）
      dt = 0.1;
      ROS_WARN_THROTTLE(1.0, "dt error (%.4f),use 0.1s", dt);
    }
  }
  // 更新上一次时间
  last_cmd_time_ = now;
  // 拿到当前机器人的位姿
  geometry_msgs::PoseStamped cur_odom;
  if (!costmap_ros_->getRobotPose(cur_odom)) {
    ROS_ERROR_STREAM("Could not get robot's odom!");
    return false;
  }

  double dx = target_pose.pose.position.x;
  double dy = target_pose.pose.position.y;

  double final_dx = final_target_.pose.position.x - cur_odom.pose.position.x;
  double final_dy = final_target_.pose.position.y - cur_odom.pose.position.y;

#ifndef TEMP

  double final_target_yaw = std::atan2(final_dy, final_dx);
  // 用 tf2::Quaternion 设置新的朝向（只改 yaw，roll/pitch 保持 0）
  tf2::Quaternion new_q;
  new_q.setRPY(0.0, 0.0, final_target_yaw); // 只设置 yaw

#else

  tf2::Quaternion tar_q(
      final_target_.pose.orientation.x, final_target_.pose.orientation.y,
      final_target_.pose.orientation.z, final_target_.pose.orientation.w);
  double tar_roll, tar_pitch, tar_yaw;
  tf2::Matrix3x3(tar_q).getRPY(tar_roll, tar_pitch, tar_yaw);
  double final_target_yaw = tar_yaw;

#endif

  double dist = sqrt(dx * dx + dy * dy);

  //   获取当前机器人yaw
  tf2::Quaternion cur_q(
      cur_odom.pose.orientation.x, cur_odom.pose.orientation.y,
      cur_odom.pose.orientation.z, cur_odom.pose.orientation.w);

  double cur_roll, cur_pitch, cur_yaw;
  tf2::Matrix3x3(cur_q).getRPY(cur_roll, cur_pitch, cur_yaw);

  // 计算姿态差距
  double yaw_error = final_target_yaw - cur_yaw;

  yaw_error = atan2(sin(yaw_error), cos(yaw_error)); // 归一化到[-pi,pi]

  double v_x, v_y, v_yaw;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    v_x = linear_x_pid_->compute(dx, dt);
    v_y = linear_y_pid_->compute(dy, dt);
    v_yaw = angular_pid_->compute(yaw_error, dt);
  }

  v_x = std::max(std::min(v_x, max_linear_vel_x_), -max_linear_vel_x_);
  v_y = std::max(std::min(v_y, max_linear_vel_y_), -max_linear_vel_y_);
  v_yaw = std::max(std::min(v_yaw, max_angular_vel_z_), -max_angular_vel_z_);

  // 速度下发
  int remaining_points = global_plan_.size() - target_index_;
  if (remaining_points > goal_reached_threshold_) {
    // 在剩余路径前,正常赋值
    cmd_vel.linear.x = v_x;
    cmd_vel.linear.y = v_y;
    if_pos_reached_ = false;

  } else {
    // 如果已经到差不多点的位置，就直接停止位置的跟踪
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    if_pos_reached_ = true;
  }

  // 误差绝对值死区判断
  if (std::abs(yaw_error) < yaw_threshold_) {
    // 误差够小，截断yaw输出
    cmd_vel.angular.z = 0;
    if_pose_reached_ = true;
  } else {
    // 输出修正
    cmd_vel.angular.z = v_yaw;
    if_pose_reached_ = false;
  }

  return true;
}

bool MyPlanner::isGoalReached() {
  // 边界检查
  if (global_plan_.empty() ||
      target_index_ >= static_cast<int>(global_plan_.size())) {
    return true; // 路径已空，直接认为到达
  }

  // 如果导航已经到达
  if (if_pose_reached_ && if_pos_reached_) {
    if_pose_reached_ = false;
    if_pos_reached_ = false;
    // 清空pid输出
    linear_x_pid_->clearIntergral();
    linear_y_pid_->clearIntergral();
    angular_pid_->clearIntergral();

    ROS_INFO_STREAM("REACH THE GOAL!");

    return true;
  }

  return false;
}

void MyPlanner::reconfigureCB(my_planner::MyPlannerConfig &config,
                              uint32_t level) {

  if (first_reconfigure_) {
    first_reconfigure_ = false;
    return;
  }
  default_config_ = config;
  ROS_INFO_STREAM("Sussceefully config the pid parameters -- linear_x P: "
                  << default_config_.linear_x_pid_p
                  << " I: " << default_config_.linear_x_pid_i
                  << " D: " << default_config_.linear_x_pid_d);
  ROS_INFO_STREAM("Sussceefully config the pid parameters -- linear_y P: "
                  << default_config_.linear_y_pid_p
                  << " I: " << default_config_.linear_y_pid_i
                  << " D: " << default_config_.linear_y_pid_d);
  ROS_INFO_STREAM("Sussceefully config the pid parameters -- angular_z P: "
                  << default_config_.angular_z_pid_p
                  << " I: " << default_config_.angular_z_pid_i
                  << " D: " << default_config_.angular_z_pid_d);
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    // pid 调节
    linear_x_pid_->setKp(default_config_.linear_x_pid_p);
    linear_x_pid_->setKi(default_config_.linear_x_pid_i);
    linear_x_pid_->setKd(default_config_.linear_x_pid_d);
    linear_y_pid_->setKp(default_config_.linear_y_pid_p);
    linear_y_pid_->setKi(default_config_.linear_y_pid_i);
    linear_y_pid_->setKd(default_config_.linear_y_pid_d);
    angular_pid_->setKp(default_config_.angular_z_pid_p);
    angular_pid_->setKi(default_config_.angular_z_pid_i);
    angular_pid_->setKd(default_config_.angular_z_pid_d);
    // 最大速度
    max_linear_vel_x_ = default_config_.max_linear_vel_x;
    max_linear_vel_y_ = default_config_.max_linear_vel_y;
    max_angular_vel_z_ = default_config_.max_angular_vel_z;

    // 规划参数
    goal_reached_threshold_ = default_config_.goal_reached_threshold;
    yaw_threshold_ = default_config_.yaw_threshold;
    forward_path_num_ = default_config_.forward_path_num;
    // 清空pid输出
    linear_x_pid_->clearIntergral();
    linear_y_pid_->clearIntergral();
    angular_pid_->clearIntergral();
  }
}

} // namespace my_planner