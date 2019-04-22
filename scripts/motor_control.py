#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is a motor command generator node for whip bot maneuver.
# It works as interface node for servo.
#


import rospy
import serial
import time
import signal
import sys
from geometry_msgs.msg import Twist
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info

vel = 0
ang = 0


def callback_get_motion(motion):
    global vel, ang
    vel = motion.linear.x
    ang = motion.angular.z
    command_servo()


def command_servo():
    global vel, ang

    servo_command_right.target_torque = vel * 3000
    pub_motor_control_right.publish(servo_command_right)


if __name__ == '__main__':
    rospy.init_node('motor_control')

    pub_motor_control = rospy.Publisher(
        'servo_command', Multi_servo_command, queue_size=1)
    servo_command = Multi_servo_command()

    rospy.Subscriber('whipbot_motion', Twist,
                     callback_get_motion, queue_size=1)
    rospy.spin()
