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
encoder_count = []
motor_velocity = []
velocity_left = 0 # velocity to ground [m/s]
velocity_right = 0 # velocity to ground [m/s]
count = 0
odometry_count = 0


# variables and constants to watch battery boltage
battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14200 #publish a warning when battery voltage is lower than 14200 [mV] (14.2V or 3.725V/cell)
BATTERY_VOLTAGE_FATAL = 13900 #publish warnings when battery voltage is lower than 13900 [mV] (13.9V or 3.475V/cell)


def callback_get_servo_info(servo_info):
    global battery_voltage_warn_flag, battery_voltage_fatal_flag, encoder_count, motor_velocity
    encoder_count = servo_info.encoder_count
    battery_voltage = servo_info.input_voltage
    motor_velocity = servo_info.motor_velocity

    if battery_voltage[0] < BATTERY_VOLTAGE_WARN and battery_voltage_warn_flag == 0:
        rospy.logwarn('battery voltage is low !')
        battery_voltage_warn_flag_left = 1
    elif battery_voltage[0] < BATTERY_VOLTAGE_FATAL:
        rospy.logfatal('battery voltage is fatally low !')

    calculate_odometry()


def calculate_odometry():
    global encoder_count, motor_velocity, velocity_left, velocity_right, count, odometry_count
    wheel_odometry.header.frame_id = '1'
    wheel_odometry.child.frame_id = '1'

    count = count + 1
    current_time = time.time()
    dt = current_time - last_time




if __name__ == '__main__':
    rospy.init_node('wheel_odometry')

    wheel_odometry_pub = rospy.Publisher(
        'wheel_odometry', Odometry, queue_size=1)

    rospy.Subscriber('multi_servo_info', Multi_servo_info,
                     callback_get_servo_info, queue_size=1)
    rospy.Subscriber('posture_angle', Posture_angle,
                     callback_get_posture, queue_size=1)

    wheel_odometry = Odometry()

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            culc_wheel_odometry()

        except IOError:
            pass

        rate.sleep()
