/**
 * @file BadmintonRobotFSM.hpp
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
#pragma once

#include <mutex>
#include <ros/ros.h>
#include <thread>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

#include <actionlib/client/simple_action_client.h>
#include <cmath> // cos, sin
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <move_control/ActionMsg.h>
#include <readline/history.h>
#include <readline/readline.h>

#include <memory>
#include <move_control/Visualization.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <queue>
#include <string>
#include <signal_handler/SignalHandler.hpp>
#include <std_msgs/Float32.h>

namespace badminton_robot_fsm {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

class PID {
public:
  PID(double kp, double ki, double kd)
      : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

  double compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prev_error_ = error;
    return output;
  }

private:
  double kp_, ki_, kd_;
  double prev_error_;
  double integral_;
};

class BadmintonRobotFSM {
  enum class RobotState {
    HAND_CTRL = 0,
    AUTO_READY = 1,
    FIND_BALL = 2,
    TAKE_BALL = 3,
    FIND_POINT = 4,
    SHOOT_BALL = 5,
  };

  enum class FindBallState {
    CALCULATE = 0,
    CONFIRM = 1,
    GO_TO = 2,
    APPROACH = 3, // 到达附近
  };

public:
  /** 仅创建 action client；是否在 init() 中连上 move_base 见 move_base_ready_ */
  BadmintonRobotFSM() : ac_("move_base", true), state_(RobotState::HAND_CTRL) {}
  ~BadmintonRobotFSM() = default;
  bool init(const ros::NodeHandle &nh);

  //   终端交互
  bool terminalApp();

  void fsmThreadLoop();

  void cameraDataCallback(const geometry_msgs::PointStamped &msg);

  void odomDataCallback(const nav_msgs::Odometry &msg);

  void ballMiddleDeltaCallback(const geometry_msgs::PointStamped &msg);

  void handCtrlAction();

  void autoReadyAction();

  void findBallAction();

  void findPointAction();

  void takeBallAction();

  void shootBallAction();

  /** 当前主状态名（用于外部定时发布 /fsm_state 等） */
  std::string getStateName() const;

  inline const char *stringCurState(RobotState view_state) const {
    switch (view_state) {
    case RobotState::HAND_CTRL: {
      return "HAND_CTRL";
    }
    case RobotState::AUTO_READY: {
      return "AUTO_READY";
    }
    case RobotState::FIND_BALL: {
      return "FIND_BALL";
    }
    case RobotState::TAKE_BALL: {
      return "TAKE_BALL";
    };
    case RobotState::FIND_POINT: {
      return "FIND_POINT";
    }
    case RobotState::SHOOT_BALL: {
      return "SHOOT_BALL";
    };
    }
    return "UNKNOWN";
  }

  void handleSignal(int /* signum */);

private:
  void enterState(RobotState state);

  void exitState(RobotState state);

  // Request a main-state transition. Uses a dedicated mutex so terminal input
  // can't deadlock with state reads/writes in the FSM thread.
  //
  // Lock ordering (must be consistent everywhere):
  // 1) state_mutex_
  // 2) next_state_mutex_
  inline void requestState(RobotState s) {
    std::lock_guard<std::mutex> lock_state(state_mutex_);
    std::lock_guard<std::mutex> lock_next(next_state_mutex_);
    next_state_ = s;
  }

  // 发送导航目标
  bool sendGoalToMoveBase(double goal_x, double goal_y, double yaw);

  // 导航到球
  bool navigateToBall();

  // 取消导航
  bool cancelNavigation();

  // 计算坐标
  bool calculate_target_self_to_global();

  // 获取导航状态
  actionlib::SimpleClientGoalState getNavigationState();

  // 等待导航完成
  bool waitForNavigationResult();

  //
  void processInput(const std::string &line);

  // 选择下一步
  void chooseNext(RobotState state);

  // 选择上一步
  void chooseBack(RobotState state);

  std::string trim(const std::string &str);
  void
  debug_calculate_target_self_to_global(double r_x, double r_y, double &g_x,
                                        double &g_y,
                                        const nav_msgs::Odometry &robo_odom);

  // 发布目标位置的tf
  void tfBroadcast(const geometry_msgs::TransformStamped &tf);

private:
  /* ros relative */
  ros::NodeHandle nh_;
  ros::Subscriber camera_sub_;  // 相机数据接收
  ros::Subscriber ball_middle_; // 球在中央点的位置
  ros::Subscriber odom_sub_;    // 接收一下机器人当前位置
  ros::Publisher cmd_vel_pub_;  // 速度发布者
  ros::Publisher action_pub_;   // 动作发布者
  ros::Publisher delta_pub_;    // 差值发布者
  geometry_msgs::Twist cmd_vel_msg_;
  move_control::ActionMsg action_msg_;
  std_msgs::Float32 delta_msg_;

  nav_msgs::Odometry cur_odom_;
  std::mutex read_odom_mutex_;

  geometry_msgs::PointStamped self_target_pos_; // 目标位置点(相对自身)
  std::mutex read_target_mutex_;
  geometry_msgs::PointStamped
      global_target_pos_; // 目标位置点(相对全局map，可以直接发布到movebase)
  bool has_target_{false};
  geometry_msgs::TransformStamped ball_tf_;       // 球的tf变换
  geometry_msgs::PointStamped ball_middle_delta_; // 球的中心位置
  std::mutex read_delta_mutex_;
  bool send_delta_{false};

  /* movebase relative */
  MoveBaseClient ac_;
  /** init() 内 waitForServer 超时后为 false，调用 ac_ 前须检查 */
  bool move_base_ready_{false};

  /* state */
  RobotState state_;
  RobotState next_state_;
  mutable std::mutex state_mutex_;
  mutable std::mutex next_state_mutex_;
  FindBallState find_ball_state_;
  std::mutex find_ball_state_mutex_;
  bool if_enter_HAND_CTRL_{false};
  bool if_enter_AUTO_READY_{false};
  bool if_enter_FIND_BALL_{false};
  bool if_enter_TAKE_BALL_{false};
  bool if_enter_FIND_POINT_{false};
  bool if_enter_SHOOT_BALL_{false};

  std::mutex terminal_app_mutex_;
  int wait_cnt_; // 用于延时
  /** AUTO_READY 内部子步骤（原 static flag；进入状态时清零） */
  int auto_ready_flag_{0};
  bool first_send_goal_{true};
  bool find_point_goal_sent_{false};

  /* 记录用于发送羽毛球的位置以及姿态 */
  // geometry_msgs::PointStamped shoot_position_;
  geometry_msgs::PoseStamped shoot_position_;

  /* thread relative */
  bool is_running_;
  std::thread fsm_thread_;

  /* for debug */
  std::unique_ptr<visualization::Visualization> visualizer_ptr_; // 创建可视化器
  double input_x_, input_y_;
  double output_x_, output_y_;
  nav_msgs::Odometry debug_odom_;
};

} // namespace badminton_robot_fsm