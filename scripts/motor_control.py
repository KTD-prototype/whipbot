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


def callback_get_servo_info(info):
    encoder_count = info.encoder_count
    voltage = info.voltage


def callback_get_command(twist.command):


if __name__ == '__main__':
    rospy.init_node('motor_control')

    pub_motor_control = rospy.Publisher(
        'servo_command', Servo_command, queue_size=1)

    rospy.Subscriber('servo_info', Servo_info,
                     callback_get_servo_info, queue_size=1)

    rospy.Subscriber('twist_command', Twist,
                     callback_get_command, queue_size=1)

    servo_command = Servo_command()

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        try:
            generate_command()
            pub_command.publish(servo_command)

        except IOError:
            pass

        rate.sleep()
