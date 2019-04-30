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

# variables and constants for wheel odometry of the robot
PULSE_PER_ROUND = 4096 #pulse per a round of the wheel
WHEEL_DIAMETER = 0.1524 # wheel diameter of the robot[m] : 6 inch
TREAD = 0.26 # tread width of the robot[m] 294mm - 34.925mm

# variables and constants to watch battery boltage
battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14200
BATTERY_VOLTAGE_FATAL = 13900

def callback_get_servo_info(servo_info):
    global battery_voltage_warn_flag, battery_voltage_fatal_flag
    encoder_count = servo_info.encoder_count
    battery_voltage = servo_info.input_voltage
    motor_velocity = servo_info.motor_velocity

    if battery_voltage[0] < BATTERY_VOLTAGE_WARN and battery_voltage_warn_flag == 0:
        rospy.logwarn('battery voltage is low !')
        battery_voltage_warn_flag_left = 1
    elif battery_voltage[0] < BATTERY_VOLTAGE_FATAL:
        rospy.logfatal('battery voltage is fatally low !')




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
