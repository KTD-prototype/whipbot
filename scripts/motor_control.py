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
from whipbot.msg import Posture_angle
from kondo_b3kondo_b3mservo_rosdriver.msg import Servo_command
from kondo_b3kondo_b3mservo_rosdriver.msg import Servo_info

battery_voltage_warn_flag = 0
battery_voltage_fatal_flag = 0
BATTERY_VOLTAGE_WARN = 14.2
BATTERY_VOLTAGE_FATAL = 13.8


def callback_get_servo_info(info):
    global battery_voltage_warn_flag, battery_voltage_fatal_flag
    encoder_count = info.encoder_count
    battery_voltage = info.motor_voltage
    velocity = info.motor_velocity

    if battery_voltage < BATTERY_VOLTAGE_WARN and battery_voltage_warn_flag == 0:
        rospy.logwarn('battery voltage is low !')
        battery_voltage_warn_flag = 1
    elif battery_voltage < BATTERY_VOLTAGE_FATAL and battery_voltage_fatal_flag == 0:
        rospy.logfatal('battery voltage is low !')
        battery_voltage_fatal_flag = 1


def callback_get_command_from_main(twist_command):
    linear_vel = twist_command.linear.x
    angular_vel = twist_command.angular.z


def callback_get_command_from_joy(joy_msg):
    teleop_lenear_vel = joy_msg.axes[1]
    teleop_angular_vel = joy_msg.axes[3]


if __name__ == '__main__':
    rospy.init_node('motor_control')

    pub_motor_control = rospy.Publisher(
        'servo_command', Servo_command, queue_size=1)

    rospy.Subscriber('servo_info', Servo_info,
                     callback_get_servo_info, queue_size=1)

    rospy.Subscriber('twist_command', Twist,
                     callback_get_command_from_main, queue_size=1)

    rospy.Subscriber('joy', Joy, callback_get_command_from_joy, queue_size=5)

    servo_command = Servo_command()

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            generate_command()
            pub_command.publish(servo_command)

        except IOError:
            pass

        rate.sleep()
