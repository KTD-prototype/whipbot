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
num = 0


def set_the_num_of_servo():
    global num
    num = rospy.get_param('num_of_servo')
    try:
        if num < 0:
            raise Exception()
    except:
        rospy.logerr("value error: the number of servos")
        sys.exit(1)
    return num
#


def callback_get_motion(motion):
    global vel, ang
    vel = motion.linear.x
    ang = motion.angular.z
    command_servo()


def command_servo():
    global vel, ang, num
    multi_servo_command = Multi_servo_command()
    for i in range(num):
        multi_servo_command.target_torque.append(vel * 3000)
    pub_motor_control.publish(multi_servo_command)
    del multi_servo_command


if __name__ == '__main__':
    rospy.init_node('motor_control')

    pub_motor_control = rospy.Publisher(
        'multi_servo_command', Multi_servo_command, queue_size=1)

    rospy.Subscriber('whipbot_motion', Twist,
                     callback_get_motion, queue_size=1)
    rospy.spin()
