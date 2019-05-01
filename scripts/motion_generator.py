#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2.7

#
# this node is a motion generator node for whip bot
# it also works as interface node for servo controller node.
#


import rospy
import serial
import time
import signal
import sys,math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from whipbot.msg import Posture_angle


# variables for velocity command from other nodes or joypad.
lenear_vel = 0
angular_vel = 0
joy_lenear_vel = 0
joy_angular_vel = 0

# variables for posture angle of the robot
pitch = 0
roll = 0
heading = 0

# variables for position and velocity of the robot
current_robot_location = [0.0, 0.0, 0.0] #current relative location from a start point : [x, y, theta]
last_robot_location = [0.0, 0.0, 0.0] #last relative location from a start point : [x, y, theta]
robot_velocity = [0.0, 0.0] #robot velocity : [linear vel, angular vel]

def callback_get_posture(posture):
    global pitch, roll, heading
    roll = posture.roll
    pitch = posture.pitch
    heading = posture.heading


def callback_get_command_from_main(twist_command):
    global linear_vel, angular_vel
    linear_vel = twist_command.linear.x
    angular_vel = twist_command.angular.z
    # rospy.loginfo('hello')


def callback_get_command_from_joy(joy_msg):
    global joy_lenear_vel, joy_angular_vel
    joy_lenear_vel = joy_msg.axes[1]
    joy_angular_vel = joy_msg.axes[3]


def generate_command():
    global joy_lenear_vel, joy_angular_vel, linear_vel, angular_vel
    if joy_lenear_vel != 0 or joy_angular_vel != 0:
        whipbot_motion.linear.x = joy_lenear_vel
        whipbot_motion.angular.z = joy_angular_vel
        pub_motion_control.publish(whipbot_motion)

    else:
        whipbot_motion.linear.x = lenear_vel
        whipbot_motion.angular.z = angular_vel
        pub_motion_control.publish(whipbot_motion)


if __name__ == '__main__':
    rospy.init_node('motion_generator')

    pub_motion_control = rospy.Publisher(
        'whipbot_motion', Twist, queue_size=1)

    rospy.Subscriber('wheel_odometry', Odometry,
                     callback_get_odometry, queue_size=1)
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
