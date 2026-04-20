/**
 * @file test_fsm_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-01-28
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <actionlib/client/simple_action_client.h>
#include <condition_variable>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_control/ActionMsg.h>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <thread>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

ros::Publisher delta_pub;
ros::Subscriber delta_sub;
ros::Subscriber board_action_sub;
ros::Publisher action_pub;
std::unique_ptr<MoveBaseClient> ac_ptr;
std::condition_variable goal_send_cv;
std::mutex goal_send_mutex;
std::thread fsm_thread;
// 定义状态枚举
enum class State { IDLE, PLANNING, CHECKING };
State current_state = State::IDLE;

std_msgs::Float32 delta;

void deltaCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  // 如果z是-1（浮点数判断）,不存
  if (std::fabs(msg->point.z + 1.0) < 0.0001) {
    return;
  }
  delta.data = msg->point.x;
  delta_pub.publish(delta);
}

void boardActionCallback(const std_msgs::UInt8::ConstPtr &msg) {
  /* MCU PICK_ACTIVE = 20 (see uart_cmd_rx / ActionTask status codes). */
  if (msg->data == 20U) {
    std::unique_lock<std::mutex> lock(goal_send_mutex);
    current_state = State::PLANNING; // 切换到规划状态
    // 条件变量释放
    goal_send_cv.notify_one();
  }
}

// 检查是否到达目标点
bool checkGoalReached() {
  if (ac_ptr->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Goal reached!");
    return true;
  }
  return false;
}

void fsmThreadLoop() {
  while (ros::ok()) {
    std::unique_lock<std::mutex> lock(goal_send_mutex);

    // 等待进入 PLANNING 状态
    goal_send_cv.wait(
        lock, [] { return current_state == State::PLANNING || !ros::ok(); });

    if (!ros::ok())
      break;

    // 进入 PLANNING 状态
    if (current_state == State::PLANNING) {
      ROS_INFO("Planning goal...");

      // 设置目标点
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.header.frame_id = "world";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = 1.0; // 示例目标点
      goal.target_pose.pose.position.y = 1.0;
      goal.target_pose.pose.orientation.w = 1.0;

      ac_ptr->sendGoal(goal);
      current_state = State::CHECKING; // 切换到 CHECKING 状态
    }

    lock.unlock();

    // 检查目标点是否到达
    while (current_state == State::CHECKING) {
      if (checkGoalReached()) {
        // 发布到达目标点的消息
        move_control::ActionMsg reach_msg;
        reach_msg.action_flag = 2.0; // 示例数据
        action_pub.publish(reach_msg);

        // 回到 IDLE 状态
        std::unique_lock<std::mutex> lock(goal_send_mutex);
        current_state = State::IDLE;
        break;
      }

      // 等待一段时间后再次检查
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_fsm_node");

  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start(); // 将回调机制设置到后台

  delta_pub = nh.advertise<std_msgs::Float32>("delta", 10);

  action_pub = nh.advertise<move_control::ActionMsg>("/action", 10);
  delta_sub = nh.subscribe<geometry_msgs::PointStamped>(
      "/basketball_detection_system/middle_ball", 10, &deltaCallback);
  board_action_sub =
      nh.subscribe<std_msgs::UInt8>("board_action", 10, &boardActionCallback);

  ac_ptr = std::make_unique<MoveBaseClient>("move_base", true);
  ROS_INFO("Waiting for move_base action server...");
  ac_ptr->waitForServer();
  ROS_INFO("Connected to move_base action server");

  std::unique_lock<std::mutex> lock(goal_send_mutex);
  // goal_send_cv.wait(lock)

  // fsm_thread = std::thread(&fsmThreadLoop); // 发送线程

  ros::waitForShutdown();

  // 等待线程结束
  // if (fsm_thread.joinable()) {
  //   fsm_thread.join();
  // }

  return 0;
}