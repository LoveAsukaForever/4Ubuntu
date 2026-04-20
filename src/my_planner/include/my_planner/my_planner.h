/**
 * @file my_planner.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2024-12-10
 *
 * @copyright Copyright (c) 2024
 *
 * @attention :
 * @note :自定义局部规划器初尝试
 * @versioninfo :
 */
#pragma once
#include <memory>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/highgui/highgui.hpp> // 显示图片窗口
#include <opencv2/imgproc/imgproc.hpp> // 绘图
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <mutex>
#include <my_planner/MyPlannerConfig.h>

namespace my_planner {

class PID {
public:
  // PID()
  PID(double kp, double ki, double kd)
      : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;
    return output;
  }

  // 清除积分，停止规划时以及更改参数时都需要清除积分
  void clearIntergral() { integral_ = 0; }

  // 设置Kp
  void setKp(double _kp) { kp_ = _kp; }

  // 设置Ki
  void setKi(double _ki) { ki_ = _ki; }

  // 设置Kd
  void setKd(double _kd) { kd_ = _kd; }

private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
};

class MyPlanner : public nav_core::BaseLocalPlanner {
public:
  MyPlanner();
  ~MyPlanner() = default; // 规划器被销毁

  /**
   * @brief 初始化
   *
   * @param name
   * @param tf
   * @param costmap_ros 存放代价地图的对象指针
   */
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  /**
   * @brief Set the Plan object 发布全局地图
   *
   * @tutorial: 该函数会在规划器被激活时调用,发布全局地图
   *            注意看数据类型，这里是一个vector，里面存放的是PoseStamped类型的数据
   *            PoseStamped- header
   *                       - pose - position 位置信息
   *                              - orientation 姿态信息
   * @param plan
   * @return true
   * @return false
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

  /**
   * @brief 输出速度控制信息
   *        不需要在这里面发布速度，只需要将规划的速度填入即可
   *
   * @tutorials: 这个函数会被以一个频率循环调用，直到到达目的地
   *             该频率可以通过move_base的参数controller_frequency设置，default：20Hz
   *             可以在.launch 中直接调用
   *              <param name="controller_frequency" value="20.0"/>
   *             来直接修改
   *
   * @param cmd_vel
   * @return true
   * @return false
   */
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  /**
   * @brief 判断是否到达终点的阈值设置
   *
   * @tutorial: 该函数会被以一个频率循环调用，直到到达目的地
   *            跟随computeVelocityCommands函数一样，以与其相同的频率调用
   *
   * @return true 认为达到目的地
   * @return false 认为还没到目的地
   */
  bool isGoalReached();

private:
  /**
   * @brief Get the User Goal Pose object
   *
   * @return geometry_msgs::PoseStamped
   */
  geometry_msgs::PoseStamped getUserGoalPose();

  /**
   * @brief 计算机器人朝向目标点的姿态
   *
   * @param robot_x
   * @param robot_y
   * @param target_x
   * @param target_y
   * @return geometry_msgs::Quaternion
   */
  geometry_msgs::Quaternion calculateRobotPoseToTarget(double robot_x,
                                                       double robot_y,
                                                       double target_x,
                                                       double target_y);

  void reconfigureCB(my_planner::MyPlannerConfig &config, uint32_t level);

private:
  /* ros relative */
  ros::NodeHandle private_nh_;

  /* ros reconfigure */
  std::unique_ptr<dynamic_reconfigure::Server<my_planner::MyPlannerConfig>>
      dsrv_;
  my_planner::MyPlannerConfig default_config_;
  std::mutex config_mutex_;
  bool first_reconfigure_{true};

  std::vector<geometry_msgs::PoseStamped> global_plan_; // 保存一下全局路线
  geometry_msgs::PoseStamped final_target_;

  std::unique_ptr<tf::TransformListener> tf_listener_; // 用于监听tf变换

  int target_index_;

  costmap_2d::Costmap2DROS *costmap_ros_; // 代价地图

  std::unique_ptr<PID> linear_x_pid_;
  std::unique_ptr<PID> linear_y_pid_;
  std::unique_ptr<PID> angular_pid_;

  std::string robot_frame_, odom_frame_;

  double max_linear_vel_x_, max_linear_vel_y_, max_angular_vel_z_;
  double forward_path_num_;
  int goal_reached_threshold_;
  double yaw_threshold_;

  ros::Time last_cmd_time_;  // 上一次计算 cmd_vel 的时间
  bool first_compute_{true}; // 第一次计算时不累积积分

  bool if_pos_reached_{false};
  bool if_pose_reached_{false};
};

} // namespace my_planner