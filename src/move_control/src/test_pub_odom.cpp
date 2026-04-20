#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

std::string msg_;

void cmdCallback(const std_msgs::String::ConstPtr &msg) { msg_ = msg->data; }

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_pub_odom");
  ros::NodeHandle nh;

  // 创建一个Publisher，发布到 /odom 话题
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

  ros::Subscriber cmd_sub_ =
      nh.subscribe<std_msgs::String>("/cmd", 10, &cmdCallback);

  ros::Rate loop_rate(10); // 设置发布频率为10Hz

  // 创建TF广播器
  tf2_ros::TransformBroadcaster tf_broadcaster;

  while (ros::ok()) {
    nav_msgs::Odometry odom_msg;

    // 填充header
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // 设置虚假的位置
    if (msg_ == "Reach") {
      odom_msg.pose.pose.position.x = 1.0; // x坐标
      odom_msg.pose.pose.position.y = 2.0; // y坐标
      odom_msg.pose.pose.position.z = 0.0; // z坐标
    } else {
      odom_msg.pose.pose.position.x = 1.0; // x坐标
      odom_msg.pose.pose.position.y = 2.0; // y坐标
      odom_msg.pose.pose.position.z = 0.0; // z坐标
    }
    // 设置虚假的四元数（表示方向）
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.88); // 假设yaw角为1.57弧度（90度）
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 设置虚假的线速度和角速度
    odom_msg.twist.twist.linear.x = 0.0;  // x方向线速度
    odom_msg.twist.twist.linear.y = 0.0;  // y方向线速度
    odom_msg.twist.twist.angular.z = 0.0; // z方向角速度

    // 发布消息
    odom_pub.publish(odom_msg);

    // 发布TF变换
    geometry_msgs::TransformStamped odom_tf;
    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = 1.0;
    odom_tf.transform.translation.y = 2.0;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    tf_broadcaster.sendTransform(odom_tf);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}