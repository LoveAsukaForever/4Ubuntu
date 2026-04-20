/**
 * @file serial_hw_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-01-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <ros/ros.h>
#include <serial_hw/SerialHW.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_hw_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(0);
  spinner.start(); // 将回调机制设置到后台
  serial_hw::SerialHW handle;

  if (!handle.init(nh)) {
    ROS_ERROR("failed to init the serial hw handler!");
    return -1;
  }

  ros::waitForShutdown();
  return 0;
}