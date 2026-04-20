/**
 * @file SerialHW.cpp
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

#include <serial_hw/SerialHW.hpp>

namespace serial_hw {

namespace {

// CRC-16/CCITT-FALSE
uint16_t calculateCRC16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

void fillMessageCRC(SerialMessage &msg) {
  if (msg.data.size() < 8) {
    return;
  }
  const size_t payload_len = static_cast<size_t>(msg.data[3]);
  const size_t crc_hi_idx = 4 + payload_len;
  const size_t crc_lo_idx = crc_hi_idx + 1;
  if (crc_lo_idx >= msg.data.size()) {
    return;
  }
  const uint16_t crc = calculateCRC16(msg.data.data(), crc_hi_idx);
  msg.data[crc_hi_idx] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  msg.data[crc_lo_idx] = static_cast<uint8_t>(crc & 0xFF);
}

} // namespace

// 析构，释放线程
SerialHW::~SerialHW() {
  // 释放发送线程
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (unpack_thread_.joinable()) {
    unpack_thread_.join();
  }
  // 关闭串口资源
  if (serial_.isOpen()) {
    serial_.close();
  }

  delete rb_;
}

bool SerialHW::init(ros::NodeHandle &nh) {

  nh_ = nh;

  std::string serial_port;
  if (!nh_.getParam("serial_port", serial_port)) {
    ROS_ERROR("[serial_port] not set!");
    return false;
  }

  //  初始化serial库
  if (!serial_.isOpen()) {

    serial_.setPort(serial_port);
    serial_.setBaudrate(115200);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(10);
    serial_.setTimeout(time_out);
    serial_.open();
    serial_.flushInput();

    // 速度消息id为1
    cmd_msg_.data.resize(20, 0); // 20 = 2+2+1+1+2+12
    // 填充消息头
    cmd_msg_.data[0] = 0xFC; // header1
    cmd_msg_.data[1] = 0xFB; // header2
    cmd_msg_.data[2] = 0x01; // ID
    cmd_msg_.data[3] = 12;   // Data length = 4*3

    // 填充消息尾
    cmd_msg_.data[16] = 0x00; // CRC placeholder
    cmd_msg_.data[17] = 0x00; // CRC placeholder
    cmd_msg_.data[18] = 0xFD; // footer1
    cmd_msg_.data[19] = 0xFE; // footer2

    // 动作消息id为2
    action_msg_.data.resize(9, 0); // 8 + 1
    // 填充消息头
    action_msg_.data[0] = 0xFC; // header1
    action_msg_.data[1] = 0xFB; // header2
    action_msg_.data[2] = 0x02; // ID
    action_msg_.data[3] = 1;    // Data length = 1

    // 填充消息尾
    action_msg_.data[5] = 0x00; // CRC placeholder
    action_msg_.data[6] = 0x00; // CRC placeholder
    action_msg_.data[7] = 0xFD; // footer1
    action_msg_.data[8] = 0xFE; // footer2

    // delta消息id为3
    delta_msg_.data.resize(12, 0); // 8+4
    // 填充消息头
    delta_msg_.data[0] = 0xFC; // header1
    delta_msg_.data[1] = 0xFB; // header2
    delta_msg_.data[2] = 0x03; // ID
    delta_msg_.data[3] = 4;    // Data length = 4*1

    // 填充消息尾
    delta_msg_.data[8] = 0x00;  // CRC placeholder
    delta_msg_.data[9] = 0x00;  // CRC placeholder
    delta_msg_.data[10] = 0xFD; // footer1
    delta_msg_.data[11] = 0xFE; // footer2

    // imu消息id为4
    imu_msg_.data.resize(20, 0); // 8+4+4+4
    // 填充消息头
    imu_msg_.data[0] = 0xFC; // header1
    imu_msg_.data[1] = 0xFB; // header2
    imu_msg_.data[2] = 0x04; // ID
    imu_msg_.data[3] = 4;    // Data length = 4*1

    // 填充消息尾
    imu_msg_.data[16] = 0x00; // CRC placeholder
    imu_msg_.data[17] = 0x00; // CRC placeholder
    imu_msg_.data[18] = 0xFD; // footer1
    imu_msg_.data[19] = 0xFE; // footer2

    // 注册订阅者（与全栈统一：速度指令走 /robot2/cmd_vel）
    cmd_vel_sub_ =
        nh_.subscribe("/robot2/cmd_vel", 10, &SerialHW::cmdVelCallback, this);
    action_sub_ = nh_.subscribe("/action", 10, &SerialHW::actionCallback, this);
    delta_sub_ = nh_.subscribe("/delta", 10, &SerialHW::deltaCallback, this);
    imu_sub_ = nh_.subscribe("/odom", 10, &SerialHW::imuCallback, this);

    // 注册发布者
    /* UInt8 so status 255 (FAULT) is not mis-encoded as Int8 -1. */
    action_pub_ = nh_.advertise<std_msgs::UInt8>("/board_action", 10);
    serial_status_pub_ = nh_.advertise<std_msgs::UInt8>("/serial_status", 10);
    comm_latency_pub_ = nh_.advertise<std_msgs::Float64>("/comm_latency", 10);
    imu_debug_pub_ = nh_.advertise<std_msgs::Float32>("/debug_imu", 10);

    /* ringbuffer */
    rb_ = new RingBuffer(128); // 128byte

    /* signal handler */
    signal_handler::SignalHandler::bindAll(&SerialHW::handleSignal, this);

    /* thread init */
    is_running_ = true;
    send_thread_ = std::thread(&SerialHW::sendThreadLoop, this); // 发送线程
    read_thread_ = std::thread(&SerialHW::readThreadLoop, this); // 读取线程
    unpack_thread_ = std::thread(&SerialHW::unpackThreadLoop, this);

    ROS_INFO("SerialHW initialized successfully");

    return true;
  } else
    return false;
}

void SerialHW::sendThreadLoop() {
  while (is_running_) {
    SerialMessage send_msg;
    bool has_msg = false;

    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      if (!cmd_vel_queue_.empty()) {
        send_msg = cmd_vel_queue_.front();
        cmd_vel_queue_.pop();
        has_msg = true;
      }
    }

    // cmd_vel为空时，检查action
    if (!has_msg) {
      std::lock_guard<std::mutex> lock(action_mutex_);
      if (!action_queue_.empty()) {
        send_msg = action_queue_.front();
        action_queue_.pop();
        has_msg = true;
      }
    }

    if (!has_msg) {
      std::lock_guard<std::mutex> lock(delta_mutex_);
      if (!delta_queue_.empty()) {
        send_msg = delta_queue_.front();
        delta_queue_.pop();
        has_msg = true;
      }
    }

    if (!has_msg) {
      std::lock_guard<std::mutex> lock(imu_mutex_);
      if (!imu_queue_.empty()) {
        send_msg = imu_queue_.front();
        imu_queue_.pop();
        has_msg = true;
      }
    }

    if (has_msg) {
      // 发送消息，在try-catch块中完成
      try {
        serial_.flushOutput();
        serial_.write(send_msg.data.data(), send_msg.data.size());
        ROS_DEBUG("Message sent successfully at time: %.3f",
                  send_msg.timestamp.toSec());
        // cmd_vel 帧 ID=0x01：测量从回调入队时刻到 write 完成（上位机侧串口发送延迟）
        if (send_msg.data.size() > 2 && send_msg.data[2] == 0x01) {
          std_msgs::Float64 lat;
          lat.data = (ros::Time::now() - send_msg.timestamp).toSec();
          comm_latency_pub_.publish(lat);
        }
      } catch (const std::exception &e) {
        ROS_ERROR("Failed to write to serial port: %s", e.what());
      }
    } else {
      // 两个队列都空，等待唤醒（超时1s防止意外卡住）
      std::unique_lock<std::mutex> lock(send_cv_mutex_);
      msg_cv_.wait_for(lock, std::chrono::milliseconds(1000),
                       [this] { return is_running_ == false; });
    }
  }
}

void SerialHW::readThreadLoop() {
  uint8_t buf[128];
  while (is_running_) {
    try {
      if (serial_.waitReadable()) {
        // 有数据才读，避免cpu空转
        size_t n = serial_.read(buf, sizeof(buf));
        if (n > 0) {
          // 写入环形缓冲区
          size_t written = rb_->write(buf, n);
          unpack_cv_.notify_one(); // 通知解包线程
          if (written < n) {
            // 缓冲区写满了，只写了部分数据
            std::cerr << "ringbuffer is full!" << std::endl;
          }
        }
      } else {
        // skip
      }

    } catch (const serial::IOException &e) {
      std::cerr << "[serial_hw]: serial port error!" << e.what() << std::endl;
    } catch (const std::exception &e) {
      std::cerr << "[serial_hw]: other error!" << e.what() << std::endl;
    }
  }
}

void SerialHW::unpackThreadLoop() {
  uint8_t buf[128];
  while (is_running_) {
    std::unique_lock<std::mutex> lock(rb_mutex_);
    unpack_cv_.wait(lock,
                    [this]() { return rb_->available() > 0 || !is_running_; });

    if (!is_running_)
      break;
    size_t available = rb_->available();
    if (available > 0) {
      size_t len = std::min(available, sizeof(buf));
      rb_->read(buf, len);

      // 解包
      unpack(buf, len);
    }
  }
}

void SerialHW::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  SerialMessage new_msg = cmd_msg_;
  new_msg.timestamp = ros::Time::now();

  float linear_x = static_cast<float>(msg->linear.x);
  float linear_y = static_cast<float>(msg->linear.y);
  float angular_z = static_cast<float>(msg->angular.z);

  memcpy(new_msg.data.data() + 4, &linear_x, 4);
  memcpy(new_msg.data.data() + 8, &linear_y, 4);
  memcpy(new_msg.data.data() + 12, &angular_z, 4);
  fillMessageCRC(new_msg);

  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    cmd_vel_queue_.push(new_msg);
  }
  msg_cv_.notify_one(); // 立即唤醒发送线程
}

void SerialHW::actionCallback(const move_control::ActionMsg::ConstPtr &msg) {
  SerialMessage new_msg = action_msg_;
  new_msg.timestamp = ros::Time::now();

  uint8_t action_flag = static_cast<uint8_t>(msg->action_flag);
  memcpy(new_msg.data.data() + 4, &action_flag, 1);
  fillMessageCRC(new_msg);

  {
    std::lock_guard<std::mutex> lock(action_mutex_);
    action_queue_.push(new_msg);
  }
  msg_cv_.notify_one(); // 立即唤醒发送线程
}

void SerialHW::deltaCallback(const std_msgs::Float32::ConstPtr &msg) {
  SerialMessage new_msg = delta_msg_;
  new_msg.timestamp = ros::Time::now();

  float delta = msg->data;
  memcpy(new_msg.data.data() + 4, &delta, 4);
  fillMessageCRC(new_msg);

  {
    std::lock_guard<std::mutex> lock(delta_mutex_);
    delta_queue_.push(new_msg);
  }
  msg_cv_.notify_one(); // 立即唤醒发送线程
}

void SerialHW::imuCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  SerialMessage new_msg = imu_msg_;
  tf2::Quaternion cur_q(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  double cur_roll, cur_pitch, cur_yaw;
  tf2::Matrix3x3(cur_q).getRPY(cur_roll, cur_pitch, cur_yaw);

  float yaw = static_cast<float>(cur_yaw);
  std_msgs::Float32 debug_imu;
  debug_imu.data = yaw;
  imu_debug_pub_.publish(debug_imu);

  float pos_x = static_cast<float>(msg->pose.pose.position.x);
  float pos_y = static_cast<float>(msg->pose.pose.position.y);

  memcpy(new_msg.data.data() + 4, &yaw, 4);
  memcpy(new_msg.data.data() + 8, &pos_x, 4);
  memcpy(new_msg.data.data() + 12, &pos_y, 4);
  fillMessageCRC(new_msg);

  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_queue_.push(new_msg);
  }
  msg_cv_.notify_one(); // 唤醒发送线程
}

void SerialHW::handleSignal(int /* signum */) {
  is_running_ = false;
  msg_cv_.notify_one();
  unpack_cv_.notify_one();
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
  if (unpack_thread_.joinable()) {
    unpack_thread_.join();
  }

  if (serial_.isOpen()) {
    serial_.close();
  }

  ros::shutdown();
  ROS_INFO("exit serial_hw node!");
}

// 解析数据状态机
void SerialHW::unpack(const uint8_t *buf, size_t len) {

  static UnpackStatus state = UnpackStatus::CHECK_HEAD;

  static std::vector<uint8_t> packet; // 用于存储当前正在解析的数据包
  static uint8_t msg_id = 0;
  static uint8_t data_length = 0;
  static uint16_t crc = 0;

  for (size_t i = 0; i < len; ++i) {
    uint8_t byte = buf[i];
    switch (state) {
    case UnpackStatus::CHECK_HEAD: {
      if (byte == 0xFC && i + 1 < len && buf[i + 1] == 0xFB) {
        packet.clear();
        packet.push_back(byte);
        packet.push_back(buf[++i]);
        state = UnpackStatus::CHECK_ID;
      }
      break;
    }
    case UnpackStatus::CHECK_ID: {
      // 读取消息id
      msg_id = byte;
      packet.push_back(byte);
      state = UnpackStatus::CHECK_LENGTH;
      break;
    }
    case UnpackStatus::CHECK_LENGTH: {
      // 读取消息长度
      data_length = byte;
      packet.push_back(byte);
      state = UnpackStatus::GET_DATA;
      break;
    }
    case UnpackStatus::GET_DATA: {
      packet.push_back(byte);
      if (packet.size() == 4 + data_length) {
        // 收够数据了，进入check crc
        state = UnpackStatus::CHECK_CRC;
      }
      break;
    }
    case UnpackStatus::CHECK_CRC: {
      packet.push_back(byte);
      if (packet.size() == 4 + data_length + 2) { // +2 = CRC
        crc = (packet[4 + data_length] << 8) | packet[4 + data_length + 1];
        if (checkCRC(packet.data(), 4 + data_length, crc)) {
          state = UnpackStatus::CHECK_FOOTER;
        } else {
          // CRC 校验失败，丢弃数据包
          state = UnpackStatus::CHECK_HEAD;
        }
      }
      break;
    }
    case UnpackStatus::CHECK_FOOTER: {
      // 读取消息尾
      packet.push_back(byte);
      if (packet.size() == 4 + data_length + 4) {
        if (packet[packet.size() - 2] == 0xFD &&
            packet[packet.size() - 1] == 0xFE) {
          // 完整消息解析成功
          processMessage(packet.data(), packet.size());
        }
        state = UnpackStatus::CHECK_HEAD;
      }
      break;
    }
    default:
      state = UnpackStatus::CHECK_HEAD;
      break;
    }
  }
}

// 根据id分发消息
void SerialHW::processMessage(const uint8_t *data, size_t len) {
  uint8_t msg_id = data[2];
  const uint8_t *payload = data + 4;
  size_t payload_len = len - 8;
  switch (msg_id) {
  case 0x00: {

    break;
  }
  case 0x01: {

    break;
  }
  case 0x02: {
    // 状态标志位（MCU uint8：10/20/30/31/255 等）
    std_msgs::UInt8 msg;
    board_action_ = *payload;
    msg.data = board_action_;
    action_pub_.publish(msg);
    serial_status_pub_.publish(msg);
    ROS_INFO_STREAM_THROTTLE(0.5, "receive the msg: " << msg.data);
    break;
  }
  default:
    break;
  }
}

bool SerialHW::checkCRC(const uint8_t *data, size_t len,
                        uint16_t received_crc) {
  const uint16_t calc_crc = calculateCRC16(data, len);
  return calc_crc == received_crc;
}

} // namespace serial_hw