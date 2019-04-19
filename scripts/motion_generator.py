"""
this node is a motor command generator node for whip bot
it also works as interface node for servo.
"""

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

import rospy
import serial
import time
import signal
import sys
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from whipbot.msg import Posture_angle
from kondo_b3mservo_rosdriver.msg import Servo_command
from kondo_b3mservo_rosdriver.msg import Servo_info

battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14.2
BATTERY_VOLTAGE_FATAL = 13.8

teleop_lenear_vel = 0
teleop_angular_vel = 0

pitch = 0
roll = 0


def callback_get_servo_info(info):
    global battery_voltage_warn_flag, battery_voltage_fatal_flag
    encoder_count = info.encoder_count
    battery_voltage = 14.0
    velocity = info.motor_velocity

    if battery_voltage < BATTERY_VOLTAGE_WARN and battery_voltage_warn_flag == 0:
        rospy.logwarn('battery voltage is low !')
        battery_voltage_warn_flag = 1
    elif battery_voltage < BATTERY_VOLTAGE_FATAL and battery_voltage_fatal_flag == 0:
        rospy.logfatal('battery voltage is low !')
        battery_voltage_fatal_flag = 1


def callback_get_posture(posture):
    global pitch, roll
    roll = posture.roll
    pitch = posture.pitch
    heading = posture.heading


def callback_get_command_from_main(twist_command):
    linear_vel = twist_command.linear.x
    angular_vel = twist_command.angular.z
    # rospy.loginfo('hello')


def callback_get_command_from_joy(joy_msg):
    global teleop_lenear_vel, teleop_angular_vel
    teleop_lenear_vel = joy_msg.axes[1]
    teleop_angular_vel = joy_msg.axes[3]


def generate_command():
    global teleop_lenear_vel, teleop_angular_vel
    if teleop_lenear_vel != 0 or teleop_angular_vel != 0:
        whipbot_motion.linear.x = teleop_lenear_vel
        whipbot_motion.angular.z = teleop_angular_vel
        pub_motor_control.publish(whipbot_motion)


if __name__ == '__main__':
    rospy.init_node('motion_generator')

    pub_motor_control = rospy.Publisher(
        'whipbot_motion', Twist, queue_size=1)

    rospy.Subscriber('servo_info', Servo_info,
                     callback_get_servo_info, queue_size=1)
    rospy.Subscriber('twist_command', Twist,
                     callback_get_command_from_main, queue_size=1)
    rospy.Subscriber('posture_angle', Posture_angle,
                     callback_get_posture, queue_size=1)
    rospy.Subscriber('joy', Joy, callback_get_command_from_joy, queue_size=5)

    whipbot_motion = Twist()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        try:
            generate_command()

        except IOError:
            pass

        rate.sleep()
