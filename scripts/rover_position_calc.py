#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math


wheel_diameter = 0.135  
track_width = 0.890  
wheelbase = 0.83866      
current_time = rospy.Time.now()
dt = (current_time - last_time).to_sec()
last_time = current_time
 


x, y, theta = 0.0, 0.0, 0.0


left_front_rpm = 0.0
left_rear_rpm = 0.0
right_front_rpm = 0.0
right_rear_rpm = 0.0


odom_pub = None

def calculate_velocity_and_yaw(left_velocity, right_velocity, track_width):
   
    avg_velocity = (left_velocity + right_velocity) / 2.0
    
   
    if right_velocity != left_velocity:
        turning_radius = (track_width / 2.0) * (right_velocity + left_velocity) / (right_velocity - left_velocity)
        yaw_rate = avg_velocity / turning_radius
    else:
        yaw_rate = 0.0

    return avg_velocity, yaw_rate

def calculate_position(x, y, theta, velocity, yaw_rate, dt):
   
    if yaw_rate != 0:
        turning_radius = velocity / yaw_rate
        x_new = x + turning_radius * (math.sin(theta + yaw_rate * dt) - math.sin(theta))
        y_new = y - turning_radius * (math.cos(theta + yaw_rate * dt) - math.cos(theta))
    else:
        x_new = x + velocity * dt * math.cos(theta)
        y_new = y + velocity * dt * math.sin(theta)
    
    theta_new = theta + yaw_rate * dt
    
   
    theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
    
    return x_new, y_new, theta_new

def left_callback(data):
    global left_front_rpm, left_rear_rpm
    left_front_rpm = data.data[0]
    left_rear_rpm = data.data[1]

def right_callback(data):
    global right_front_rpm, right_rear_rpm
    right_front_rpm = data.data[0]
    right_rear_rpm = data.data[1]

def publish_odometry(x, y, theta):
    # Odometry mesajı oluştur
    odom_msg = Odometry()

    
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

   
    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0

   
    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(theta / 2.0)
    quaternion.w = math.cos(theta / 2.0)
    
    
    odom_msg.pose.pose.orientation = quaternion

   
    odom_msg.twist.twist.linear.x = 0.0
    odom_msg.twist.twist.linear.y = 0.0
    odom_msg.twist.twist.linear.z = 0.0
    odom_msg.twist.twist.angular.x = 0.0
    odom_msg.twist.twist.angular.y = 0.0
    odom_msg.twist.twist.angular.z = 0.0

   
    odom_pub.publish(odom_msg)

def update_position(event):
    global x, y, theta
    
  
    left_velocity = (left_front_rpm + left_rear_rpm) / 2.0 * (math.pi * wheel_diameter / 60.0)
    right_velocity = (right_front_rpm + right_rear_rpm) / 2.0 * (math.pi * wheel_diameter / 60.0)
    
    
    avg_velocity, yaw_rate = calculate_velocity_and_yaw(left_velocity, right_velocity, track_width)
    
   
    x, y, theta = calculate_position(x, y, theta, avg_velocity, yaw_rate, time_elapsed)
    
 
    publish_odometry(x, y, theta)

if __name__ == '__main__':
    rospy.init_node('position_calculator', anonymous=True)
    
   
    rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, left_callback)
    
    
    rospy.Subscriber('/drive_system_right_motors_feedbacks', Float64MultiArray, right_callback)
    
    
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    
   
    rospy.Timer(rospy.Duration(time_elapsed), update_position)
    
    rospy.spin()
