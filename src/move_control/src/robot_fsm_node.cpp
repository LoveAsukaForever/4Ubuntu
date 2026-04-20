// clang-format off
/**
 * @file robot_fsm_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 状态机执行
 * @version 0.1
 * @date 2026-01-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 上电位于HAND_CTRL状态，等待触发信号开始进入AUTO_READY
 *         FIND_BALL 将开启自转模式，开始搜寻范围之内的球
 *         TAKE_BALL 通知机构进行拾球(在距离目标点距离足够近时触发，此时机器人将会直接开启摄像头跟踪锁定)
 *         FIND_POINT 寻找期望发射球的位置，可以写死在地图中定位的点
 *         SHOOT_BALL 通知机构进行发射动作
 * @versioninfo :
 */
// clang-format on
#include <atomic>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <move_control/BadmintonRobotFSM.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_fsm_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start(); // 将回调机制设置到后台

  /* 实例化handler */
  badminton_robot_fsm::BadmintonRobotFSM handler;

  if (!handler.init(nh)) {
    ROS_ERROR("fsm handler init failed! ");
    return -1;
  }

  ROS_INFO("start to the terminal APP");

  ros::Publisher fsm_state_pub =
      nh.advertise<std_msgs::String>("/fsm_state", 10);

  std::atomic<bool> stop_fsm_state_pub{false};
  std::thread fsm_state_pub_thread([&]() {
    // Use wall time so /fsm_state keeps publishing even with /use_sim_time and no /clock.
    ros::WallRate rate(10);
    while (ros::ok() && !stop_fsm_state_pub) {
      std_msgs::String msg;
      msg.data = handler.getStateName();
      fsm_state_pub.publish(msg);
      rate.sleep();
    }
  });

  // 这里会阻塞本线程，执行终端任务，持续接收用户输入
  handler.terminalApp();

  stop_fsm_state_pub = true;
  if (fsm_state_pub_thread.joinable()) {
    fsm_state_pub_thread.join();
  }

  ros::waitForShutdown();
  return 0;
}
