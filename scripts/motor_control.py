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
from sensor_msgs.msg import Imu
from whipbot.msg import Posture_angle
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info

vel = 0
ang = 0
num = 0

def callback_init(number):
    global num
    num = number.data
    for i in range(num):
        target_position.append(0)
        target_velocity.append(0)
        target_torque.append(0)


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

    rospy.Subscriber('the_number_of_servo', Int16, callback_init, queue_size=1)
    rospy.Subscriber('whipbot_motion', Twist,
                     callback_get_motion, queue_size=1)
    rospy.spin()
