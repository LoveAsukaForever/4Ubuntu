/**
 * @file send_goal.cpp
 * @author Keten (2863861004@qq.com)
 * @brief  发布坐标点
 * @version 0.1
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note ：从终端中读取输入流，可以不断发布而覆盖掉上一次发布的坐标点
 * @versioninfo :
 */
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Quaternion.h"
#include "iostream"
#include "move_base_msgs/MoveBaseAction.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

int main(int argc, char **argv) {
  ros::init(argc, argv, "send_goal");

  MoveBaseClient ac("move_base", true);

  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  while (ros::ok()) {
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    std::cout << "waiting for input: [x y yaw] ";
    double x = 0, y = 0, yaw = 0;
    std::cin >> x >> y >> yaw;
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
    quaternionTFToMsg(q, goal.target_pose.pose.orientation);
    ROS_INFO("ready to send goal!");
    ac.sendGoal(goal);
    ROS_INFO("Send [x]: %f [y]: %f", goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y);
  }

  return 0;
}