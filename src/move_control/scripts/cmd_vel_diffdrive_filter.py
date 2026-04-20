#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def cb(msg: Twist):
    out = Twist()
    out.linear.x = msg.linear.x
    out.linear.y = 0.0
    out.linear.z = 0.0
    out.angular.x = 0.0
    out.angular.y = 0.0
    out.angular.z = msg.angular.z
    pub.publish(out)

rospy.init_node("cmd_vel_diffdrive_filter")
pub = rospy.Publisher("/cmd_vel_nav", Twist, queue_size=10)
rospy.Subscriber("/robot2/cmd_vel", Twist, cb, queue_size=10)
rospy.spin()
