#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

def imu_callback(data):
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    )

    
    euler = tf.transformations.euler_from_quaternion(quaternion)

   
    euler_msg = Vector3()
    euler_msg.x = euler[0]  # Roll
    euler_msg.y = euler[1]  # Pitch
    euler_msg.z = euler[2]  # Yaw


    euler_pub.publish(euler_msg)

if __name__ == '__main__':
    rospy.init_node('imu_to_euler', anonymous=True)


    rospy.Subscriber('/imu1/data', Imu, imu_callback)


    euler_pub = rospy.Publisher('/imu_euler', Vector3, queue_size=10)

    rospy.spin()
