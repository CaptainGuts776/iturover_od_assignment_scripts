#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
from visualization_msgs.msg import Marker

def callback(data):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "euler_angles"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

 
    marker.scale.x = 0.5 
    marker.scale.y = 0.1  
    marker.scale.z = 0.1 

   
    marker.color.a = 1.0 
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    
    marker.pose.orientation.x = data.x
    marker.pose.orientation.y = data.y
    marker.pose.orientation.z = data.z
    marker.pose.orientation.w = 1.0  

  
    marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('euler_marker_publisher', anonymous=True)

    marker_pub = rospy.Publisher('/euler_marker', Marker, queue_size=10)
    rospy.Subscriber('/imu_euler', Vector3, callback)

    rospy.spin()
