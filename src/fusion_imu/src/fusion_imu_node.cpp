/**
 * @file fusion_imu_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2026-02-01
 *
 * @copyright Copyright (c) 2026
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <Fusion.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdbool.h>

FusionBias bias;
FusionAhrs ahrs;

// IMU 数据回调函数
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
  // 从 IMU 消息中提取角速度
  FusionVector gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y,
                            msg->angular_velocity.z};

  // 加速度计数据
  FusionVector accelerometer = {msg->linear_acceleration.x,
                                msg->linear_acceleration.y,
                                msg->linear_acceleration.z};

  // 如果有磁力计数据，可以提取
  FusionVector magnetometer = {0.0f, 0.0f, 0.0f}; // 默认无磁力计数据
  if (!std::isnan(msg->orientation_covariance[0])) {
    magnetometer.axis.x = msg->orientation_covariance[0];
    magnetometer.axis.y = msg->orientation_covariance[1];
    magnetometer.axis.z = msg->orientation_covariance[2];
  }

  // 校准数据
  gyroscope = FusionBiasUpdate(&bias, gyroscope);

  // 计算时间间隔
  static ros::Time previous_time = ros::Time::now();
  ros::Time current_time = ros::Time::now();
  float delta_time = (current_time - previous_time).toSec();
  previous_time = current_time;

  // 更新 AHRS 算法
  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta_time);

  // 获取四元数姿态
  FusionQuaternion quaternion = FusionAhrsGetQuaternion(&ahrs);

  // 发布四元数姿态
  static ros::Publisher quaternion_pub =
      ros::NodeHandle().advertise<geometry_msgs::Quaternion>("/ahrs_imu", 10);
  geometry_msgs::Quaternion quat_msg;
  quat_msg.x = quaternion.element.x;
  quat_msg.y = quaternion.element.y;
  quat_msg.z = quaternion.element.z;
  quat_msg.w = quaternion.element.w;
  quaternion_pub.publish(quat_msg);

  // 打印姿态信息（可选）
  FusionEuler euler = FusionQuaternionToEuler(quaternion);
  ROS_INFO("Roll: %.2f, Pitch: %.2f, Yaw: %.2f", euler.angle.roll,
           euler.angle.pitch, euler.angle.yaw);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_fusion_node");
  ros::NodeHandle nh;

  // 初始化 Fusion 库
  const float sample_rate = 200.0f; // 替换为实际采样率
  FusionBiasInitialise(&bias, sample_rate);
  FusionAhrsInitialise(&ahrs);

  // 设置 AHRS 参数
  FusionAhrsSettings settings = {
      .convention = FusionConventionNwu, // NWU 坐标系
      .gain = 0.5f,                      // 默认增益
      .gyroscopeRange = 2000.0f,         // 陀螺仪量程
      .accelerationRejection = 15.0f,    // 加速度拒绝阈值
      .magneticRejection = 10.0f,        // 磁场拒绝阈值
      .recoveryTriggerPeriod =
          5 * sample_rate, // 5 秒恢复（假设采样率为 100Hz）
  };
  FusionAhrsSetSettings(&ahrs, &settings);

  // 订阅 IMU 数据
  ros::Subscriber imu_sub =
      nh.subscribe<sensor_msgs::Imu>("/livox/imu", 10, imuCallback);

  // 进入 ROS 循环
  ros::spin();

  return 0;
}