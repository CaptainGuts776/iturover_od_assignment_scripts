#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

def callback(data):
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

  
    if len(data.data) >= 3:
        odom.pose.pose.position.x = data.data[0]
        odom.pose.pose.position.y = data.data[1]
        odom.pose.pose.position.z = data.data[2]
    else:
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0

    odom_pub.publish(odom)

rospy.init_node('float64_to_odom', anonymous=True)
odom_pub = rospy.Publisher('/converted_odom', Odometry, queue_size=10)
rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, callback)
rospy.spin()

