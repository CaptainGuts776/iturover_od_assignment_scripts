#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import math


wheel_diameter = 0.135  
time_elapsed = 1.0    


total_distance = 0.0


is_moving = False


left_front_rpm = 0.0
left_rear_rpm = 0.0
right_front_rpm = 0.0
right_rear_rpm = 0.0

def calculate_distance(rpm):
    
    wheel_circumference = math.pi * wheel_diameter
    distance = (rpm * time_elapsed / 60.0) * wheel_circumference
    return distance

def left_callback(data):
    global left_front_rpm, left_rear_rpm
    left_front_rpm = data.data[0]
    left_rear_rpm = data.data[1]

def right_callback(data):
    global right_front_rpm, right_rear_rpm
    right_front_rpm = data.data[0]
    right_rear_rpm = data.data[1]

def calculate_and_log_distance():
    global total_distance, is_moving
    
    
    left_front_distance = calculate_distance(left_front_rpm)
    left_rear_distance = calculate_distance(left_rear_rpm)
    left_distance = (left_front_distance + left_rear_distance) / 2.0
    
    
    right_front_distance = calculate_distance(right_front_rpm)
    right_rear_distance = calculate_distance(right_rear_rpm)
    right_distance = (right_front_distance + right_rear_distance) / 2.0
        
    current_distance = (left_distance + right_distance) / 2.0
    
    if current_distance > 0:
        total_distance += current_distance
        if not is_moving:
            rospy.loginfo("Araç hareket etmeye başladı.")
            is_moving = True
        rospy.loginfo("Alınan Mesafe (Bu Adım): %.2f metre" % current_distance)
        rospy.loginfo("Toplam Alınan Mesafe: %.2f metre" % total_distance)
    else:
        if is_moving:
            rospy.loginfo("Araç durdu. Toplam alınan mesafe: %.2f metre" % total_distance)
            is_moving = False

if __name__ == '__main__':
    rospy.init_node('distance_calculator', anonymous=True)
    
    
    rospy.Subscriber('/drive_system_left_motors_feedbacks', Float64MultiArray, left_callback)
    
    
    rospy.Subscriber('/drive_system_right_motors_feedbacks', Float64MultiArray, right_callback)
    
    
    rospy.Timer(rospy.Duration(time_elapsed), lambda event: calculate_and_log_distance())
    
    rospy.spin()
