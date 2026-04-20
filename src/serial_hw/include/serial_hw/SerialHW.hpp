/**
 * @file SerialHW.hpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-01-18
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note : 速度3个float + 1个目标偏移量的float+
 * 1个动作位：0空闲，1规划中，2捡球，3射球（与下位机 RobotCommand_t.action_flag 对齐）
 * 4*4+1 = 17个Bytes --- 17个uint8_t
 * @versioninfo :
 */

#pragma once

// clang-format off

// ros origin package
#include <XmlRpc.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// other ros package
#include <signal_handler/SignalHandler.hpp>
#include <move_control/ActionMsg.h>
#include <any_node/ThreadedPublisher.hpp>
#include <serial_hw/Ringbuffer.h>


// system
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <errno.h>

// clang-format on

namespace serial_hw {

struct SerialMessage {
  std::vector<uint8_t> data;
  ros::Time timestamp;
};

class SerialHW {

  enum class UnpackStatus {
    CHECK_HEAD = 0,
    CHECK_ID = 1,
    CHECK_LENGTH = 2,
    GET_DATA = 3,
    CHECK_CRC = 4,
    CHECK_FOOTER
  };

public:
  SerialHW() = default;
  ~SerialHW();

  bool init(ros::NodeHandle &nh);
  void read(const ros::Time &time);

  void sendThreadLoop();
  void readThreadLoop();
  void unpackThreadLoop();

  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void actionCallback(const move_control::ActionMsg::ConstPtr &msg);
  void deltaCallback(const std_msgs::Float32::ConstPtr &msg);
  void imuCallback(const nav_msgs::Odometry::ConstPtr &msg);

  void handleSignal(int /* signum */);

private:
  void unpack(const uint8_t *buf, size_t len);
  void processMessage(const uint8_t *data, size_t len);

  bool checkCRC(const uint8_t *data, size_t len, uint16_t received_crc);

  // receive msg
  uint8_t board_action_; // 1个整数

  /* ros relative */
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber action_sub_;
  ros::Subscriber delta_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher action_pub_;
  ros::Publisher serial_status_pub_;
  ros::Publisher comm_latency_pub_;
  ros::Publisher imu_debug_pub_;

  /* thread relative */
  std::thread send_thread_;
  std::atomic<bool> is_running_{false};
  std::thread read_thread_;
  std::thread unpack_thread_;

  // 消息队列和同步工具
  std::queue<SerialMessage> cmd_vel_queue_;
  std::queue<SerialMessage> action_queue_;
  std::queue<SerialMessage> delta_queue_;
  std::queue<SerialMessage> imu_queue_;

  std::mutex cmd_vel_mutex_;
  std::mutex action_mutex_;
  std::mutex delta_mutex_;
  std::mutex imu_mutex_;
  std::mutex send_cv_mutex_;
  std::mutex rb_mutex_;

  std::condition_variable msg_cv_;
  std::condition_variable unpack_cv_;

  SerialMessage cmd_msg_;
  SerialMessage action_msg_;
  SerialMessage delta_msg_;
  SerialMessage imu_msg_;

  /* serial relative */
  serial::Serial serial_{};

  /* ringbuffer */
  RingBuffer *rb_{nullptr};
};

} // namespace serial_hw
