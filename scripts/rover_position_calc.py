#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import math


wheel_diameter = 0.135 
track_width = 0.890     
wheelbase = 0.83866      
time_elapsed = 1.0   


x, y, theta = 0.0, 0.0, 0.0


left_front_rpm = 0.0
left_rear_rpm = 0.0
right_front_rpm = 0.0
right_rear_rpm = 0.0

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
    
    rospy.loginfo("Yeni Pozisyon: x=%.2f, y=%.2f, theta=%.2f" % (x_new, y_new, theta_new))
    
    return x_new, y_new, theta_new

def left_callback(data):
    global left_front_rpm, left_rear_rpm
    left_front_rpm = data.data[0]
    left_rear_rpm = data.data[1]

def right_callback(data):
    global right_front_rpm, right_rear_rpm
    right_front_rpm = data.data[0]
    right_rear_rpm = data.data[1]

def update_position(event):
    global x, y, theta
    
    
    left_velocity = (left_front_rpm + left_rear_rpm) / 2.0 * (math.pi * wheel_diameter / 60.0)
    right_velocity = (right_front_rpm + right_rear_rpm) / 2.0 * (math.pi * wheel_diameter / 60.0)
    
   
    avg_velocity, yaw_rate = calculate_velocity_and_yaw(left_velocity, right_velocity, track_width)
    
    
    x, y, theta = calculate_position(x, y, theta, avg_velocity, yaw_rate, time_elapsed)
if abs(x - previous_x) > position_threshold or abs(y - previous_y) > position_threshold or abs(theta - previous_theta) > theta_threshold:
        rospy.loginfo("Yeni Pozisyon: x=%.2f, y=%.2f, theta=%.2f" % (x, y, theta))
        previous_x, previous_y, previous_theta = x, y, theta
if __name__ == '__main__':
    rospy.init_node('position_calculator', anonymous=True)
    
    
    
    rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, left_callback)
    
    
    rospy.Subscriber('/drive_system_right_motors_feedbacks', Float64MultiArray, right_callback)
    
    
    rospy.Timer(rospy.Duration(time_elapsed), update_position)
    
    rospy.spin()
