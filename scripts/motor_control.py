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
from kondo_b3mservo_rosdriver.msg import Servo_command
from kondo_b3mservo_rosdriver.msg import Servo_info

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

    servo_command_left.target_torque = vel * 3000
    pub_motor_control_left.publish(servo_command_left)


if __name__ == '__main__':
    rospy.init_node('motor_control')

    pub_motor_control_right = rospy.Publisher(
        'servo_command_right', Servo_command, queue_size=1)
    pub_motor_control_left = rospy.Publisher(
        'servo_command_left', Servo_command, queue_size=1)
    servo_command_right = Servo_command()
    servo_command_left = Servo_command()

    rospy.Subscriber('whipbot_motion', Twist,
                     callback_get_motion, queue_size=1)

    rospy.spin()
