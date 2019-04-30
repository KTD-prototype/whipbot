#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
#

import rospy
import serial
import time
import signal
import sys,math
from nav_msgs.msg import Odometry
from whipbot.msg import Posture_angle
from kondo_b3mservo_rosdriver.msg import Multi_servo_info

# variables to store timings of calculation
current_time = time.time()
last_time = time.time()

# constants for wheel odometry of the robot
PULSE_PER_ROUND = 4096 #pulse per a round of the wheel
WHEEL_DIAMETER = 0.1524 # wheel diameter of the robot[m] : 6 inch
TREAD = 0.26 # tread width of the robot[m] 294mm - 34.925mm

# variables for wheel odometry of the robot
current_encoder_count = [0, 0]
last_encoder_count = [0, 0]
motor_velocity = [0, 0]
velocity_left = 0 # velocity to ground [m/s]
velocity_right = 0 # velocity to ground [m/s]
odometry_count = 0
robot_location = [0.0, 0.0] #relative location from a start point: [x, y]
robot_orientation = 0.0 #relative orientation from a  start orientation
robot_velocity = [0.0, 0.0] #robot velocity : [linear vel, angular vel]

# variables and constants to watch battery boltage
battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14200 #publish a warning when battery voltage is lower than 14200 [mV] (14.2V or 3.725V/cell)
BATTERY_VOLTAGE_FATAL = 13900 #publish warnings when battery voltage is lower than 13900 [mV] (13.9V or 3.475V/cell)


def main_function():
    global odometry_count, current_time, last_time, robot_location, robot_orientation, robot_velocity
    rospy.init_node('wheel_odometry')

    wheel_odometry_pub = rospy.Publisher(
        'wheel_odometry', Odometry, queue_size=1)

    rospy.Subscriber('multi_servo_info', Multi_servo_info,
                     callback_get_servo_info, queue_size=1)
    rospy.Subscriber('posture_angle', Posture_angle,
                     callback_get_posture, queue_size=1)

    wheel_odometry = Odometry()
    wheel_odometry.header.frame_id = '1'
    wheel_odometry.child.frame_id = '1'

    current_time = time.time()
    last_time = time.time()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            current_time = time.time()
            dt = current_time - last_time
            wheel_odometry.header.seq = odometry_count
            wheel_odometry.pose.pose.position.x = robot_location[0]
            wheel_odometry.pose.pose.position.y = robot_location[1]
            wheel_odometry.pose.pose.orientation.w = robot_orientation
            wheel_odometry.twist.twist.linear.x = robot_velocity[0]
            wheel_odometry.twist.twist.angular.z = robot_velocity[1]
            wheel_odometry_pub.publisht(wheel_odometry)
            odometry_count = odometry_count + 1
            last_time = time.time()

        except IOError:
            pass

        rate.sleep()



def callback_get_servo_info(servo_info):
    global battery_voltage_warn_flag, battery_voltage_fatal_flag, current_encoder_count, motor_velocity
    current_encoder_count = servo_info.encoder_count
    battery_voltage = servo_info.input_voltage
    motor_velocity = servo_info.motor_velocity

    if battery_voltage[0] < BATTERY_VOLTAGE_WARN and battery_voltage_warn_flag == 0:
        rospy.logwarn('battery voltage is low !')
        battery_voltage_warn_flag_left = 1
    elif battery_voltage[0] < BATTERY_VOLTAGE_FATAL:
        rospy.logfatal('battery voltage is fatally low !')

    calculate_odometry()


def calculate_odometry):
    global robot_location, robot_orientation, robot_velocity, velocity_left, velocity_right, current_encoder_count, last_encoder_count, motor_velocity, current_time, last_time, dt

    delta_encoder = [x-y for (x,y) in zip(current_encoder_count, last_encoder_count)]
    velocity_left = ((delta_encoder[0] / PULSE_PER_ROUND) * pi * WHEEL_DIAMETER) / dt
    velocity_right = ((delta_encoder[1] / PULSE_PER_ROUND) * pi * WHEEL_DIAMETER) / dt

    robot_velocity[0] = (velocity_left + velocity_right) / 2.0
    robot_velocity[1] = (velocity_right - velocity_left) / (TREAD / 2.0)



if __name__ == '__main__':
    main_function()
