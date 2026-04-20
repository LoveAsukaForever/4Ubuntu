/**
 * @file send_goal.h
 * @author Keten (2863861004@qq.com)
 * @brief 将发送目标点的功能封装成行为树节点
 * @version 0.1
 * @date 2025-01-18
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/PoseStamped.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "ros/ros.h"

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace send_goal_process {

class SendGoal : public BT::StatefulActionNode {
public:
  SendGoal(const std::string &name, const BT::NodeConfiguration &config)
      : BT::StatefulActionNode(name, config), ac_("move_base", true) {
    // 连接到服务器
    while (!ac_.waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }
  }
  ~SendGoal() = default;

  static BT::PortsList providedPorts() {
    return {BT::InputPort<geometry_msgs::PoseStamped>("goal_pose")};
  }

  virtual BT::NodeStatus onStart() override {
    auto goal_pose = getInput<geometry_msgs::PoseStamped>("goal_pose");
    if (!goal_pose) {
      throw BT::RuntimeError("missing required input [goal_pose]: ",
                             goal_pose.error());
    }
    goal_pose_ = goal_pose.value();
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose_;
    ac_.sendGoal(goal);

    return BT::NodeStatus::RUNNING;
  }

  virtual BT::NodeStatus onRunning() override {
    actionlib::SimpleClientGoalState state = ac_.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("reach the goal!");
      return BT::NodeStatus::SUCCESS;
    } else if (state == actionlib::SimpleClientGoalState::ABORTED ||
               state == actionlib::SimpleClientGoalState::REJECTED) {
      ROS_WARN("failed to reach the goal!");
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING;
  }

  virtual void onHalted() override {
    if (ac_.getState() == actionlib::SimpleClientGoalState::ACTIVE ||
        ac_.getState() == actionlib::SimpleClientGoalState::PENDING) {
      ROS_WARN("行为树被中断，取消目标。");
      ac_.cancelGoal();
    }
  }

private:
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  geometry_msgs::PoseStamped goal_pose_;
};

} // namespace send_goal_process
